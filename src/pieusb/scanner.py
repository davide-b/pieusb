from pieusb.option import (
    generate_options,
    set_options,
    DEFAULT_EXPOSURE_FACTOR,
    DEFAULT_FOCUS,
    EXPOSURE_FACTOR_OPTIONS,
    FOCUS_MIN,
    DEFAULT_RELATIVE_EXPOSURE,
    MAX_RELATIVE_EXPOSURE,
    MODE_PLANES,
    max_relative_exposure,
    Parameter,
)
from pieusb.postprocess import (
    apply_shading_correction,
    build_width_to_loc,
    calculate_shading,
    line_sharpness
)
from pieusb.types import (
    AutofocusResult,
    DeviceInfo,
    EjectDirection,
    UpdateData,
    ScanResult,
    ScanPhase
)
from pieusb.transport import (
    UASDevice,
    SCSI_READ,
    SCSI_READ_STATE,
    SCSI_WRITE,
    SCSI_SCAN,
    SCSI_TEST_UNIT_READY,
    SCSI_CALIBRATION_INFO,
    SCSI_READ_GAIN_OFFSET,
    SCSI_PARAM,
    SCSI_COPY,
    SCSI_SLIDE,
    SLIDE_EJECT_DOWN,
    SLIDE_EJECT_UP,
    SLIDE_INIT,
    SLIDE_NEXT,
    SLIDE_OFFSET_BACK,
    SLIDE_OFFSET_FORWARD,
    SLIDE_PREV
)
from pieusb.exceptions import (
    CheckCondition,
    PieusbError,
    WarmingUp,
    DeviceNotReady,
    EndOfStrip,
    ScanInProgress
)

import logging
import time
import threading
import struct

import numpy

from pieusb.framing import FilmBase, film_base, find_bands

from collections.abc import Callable, Iterator
from contextlib import contextmanager
from dataclasses import dataclass, replace

log = logging.getLogger(__name__)

# Hard cap per READ (pieusb_specific.c:2248). A maximum, not a fixed size;
# requesting more than 255 lines in one call returns banded images.
MAX_LINES_PER_READ = 255

# Every line the scanner returns in INDEX colour format is prefixed by a two-byte
# channel tag.
TAG_TO_CHANNEL = {b'RR': 0, b'GG': 1, b'BB': 2, b'II': 3}
CHANNEL_NAMES = ('R', 'G', 'B', 'I')
# Channel index -> the suffix its gain_*/exp_time_*/offset_* options carry.
CHANNEL_SUFFIX = ('r', 'g', 'b', 'i')

# The options auto_exp derives, and therefore restores afterwards.
METERED_OPTIONS = EXPOSURE_FACTOR_OPTIONS

MM_PER_INCH = 25.4

# Where auto_exp aims the metered channel, as a fraction of full scale. Below 1.0
# to leave room for specular highlights and for the shading correction's per-column
# gain, which clips edge columns first (postprocess.apply_shading_correction).
METERING_TARGET = 0.85

# Percentile of each colour plane taken as "the top of the image", so that a
# handful of dust specks or scratches cannot veto the whole exposure.
METERING_PERCENTILE = 99

# Steps forward meter_base() moves to bring a gap into view when the frame in
# the gate shows none: 7.41 mm nominal, CyberView's own centring move. On an
# RPS 10M every such scan showed a whole gap.
BASE_OFFSET_STEPS = 70

# Used for the metering pass if the device's own reported preview resolution is
# unusable. Matches the 'resolution' option default.
PREVIEW_RESOLUTION_FALLBACK = 300

# The autofocus pass's resolution: what CyberView uses on the RPS 10M (5000) and
# on the ProScan 4000, whose maximum is 3600.
AUTOFOCUS_RESOLUTION = 5000

# The autofocus pass's bit depth. 8 is what the vendor software reads.
AUTOFOCUS_COLOR_DEPTH = 8

# Autofocus scores each position by the mean of the line scores within this
# window around it, narrowed at the ends of the range so the first and last
# positions are judged on their own side. A single line's score is noisy: on
# the plateau of in-focus positions neighbouring lines differ by up to 5%, more
# than the positions themselves do.
AUTOFOCUS_SMOOTHING = 5

# The smoothed curve's best over its worst below which the sweep has nothing to
# trust. Sweeps of detailed film reach 1.13-1.35 (RPS 10M and ProScan 4000,
# filmstrip); one of an RPS 10M's default frame reached 1.10, a slow rise to the
# end of the range from something other than the film's picture.
AUTOFOCUS_MIN_DEPTH = 1.115

# START SCAN is retried while the device reports warming up (pieusb.c:1088-1093).
START_SCAN_ATTEMPTS = 30
START_SCAN_RETRY_S = 5

# How long close() waits for a cancelled worker to notice. Cancellation lands at
# the next chunk boundary, so the bound is one outstanding command.
CLOSE_WAIT_S = 90

# READ STATE reply length. The smallest that covers every field TransportState
# decodes; SANE asks 12 and CyberView 11, and the device pads either way.
READ_STATE_SIZE = 8

# READ STATE byte 6. These two hold across every state observed so far, carriers
# and ejects included. Bits 3, 4 and 5 also move with the frame index but not
# monotonically, and nothing yet explains them; read TransportState.flags for those.
STATE_MEDIUM_PRESENT = 0x01
STATE_FILMSTRIP_CARRIER = 0x04

# SCSI_SLIDE offset steps are 1/240 inch.
OFFSET_STEPS_PER_INCH = 240
OFFSET_STEPS_PER_MM = OFFSET_STEPS_PER_INCH / 25.4

SLIDE_ACTION_NAMES = {
    SLIDE_OFFSET_FORWARD: 'OFFSET FORWARD',
    SLIDE_OFFSET_BACK: 'OFFSET BACK',
    SLIDE_EJECT_DOWN: 'EJECT DOWN',
    SLIDE_EJECT_UP: 'EJECT UP',
    SLIDE_NEXT: 'NEXT',
    SLIDE_PREV: 'PREV',
    SLIDE_INIT: 'INIT',
}


def smooth_scores(scores: numpy.ndarray, width: int = AUTOFOCUS_SMOOTHING) -> numpy.ndarray:
    """Each score averaged with its neighbours within `width`, the window
    narrowed at the ends."""
    scores = numpy.asarray(scores, dtype=numpy.float64)
    kernel = numpy.ones(width)
    return (numpy.convolve(scores, kernel, 'same')
            / numpy.convolve(numpy.ones_like(scores), kernel, 'same'))


def autofocus_pick(scores, positions) -> tuple[int, float, numpy.ndarray]:
    """The position autofocus would pick, the sweep's depth, and the smoothed scores.

    The pick is the best smoothed score's position. The depth is the smoothed
    curve's best over its worst; under AUTOFOCUS_MIN_DEPTH the pick is not to be
    trusted.
    """
    smoothed = smooth_scores(scores)
    worst = smoothed.min()
    depth = float(smoothed.max() / worst) if worst > 0 else 0.0
    return positions[int(numpy.argmax(smoothed))], depth, smoothed


def autofocus_positions(lines: int, focus_min: int, focus_max: int) -> list[int]:
    """The focus position each line of an autofocus pass was taken at.

    One line per position when the counts agree. When they do not the mapping is
    unknown, so the lines are spread evenly over the range and a warning says so.
    """
    span = focus_max - focus_min + 1
    if lines == span:
        return list(range(focus_min, focus_max + 1))
    log.warning(f"[autofocus] the pass returned {lines} lines for {span} focus "
                f"positions; spreading them evenly over {focus_min}..{focus_max}")
    if lines == 1:
        return [focus_min]
    return [focus_min + round(i * (span - 1) / (lines - 1)) for i in range(lines)]


@dataclass(frozen=True)
class TransportState:
    '''READ STATE, decoded. Only meaningful on a model with a film transport.

    `focus_max` is holder-dependent -- 80 with a slide carrier, 30 with a
    filmstrip carrier on a ProScan 4000 -- so it is read, never assumed.
    '''
    frame: int
    focus: int
    focus_max: int
    flags: int
    raw: bytes

    medium_present = property(lambda s: bool(s.flags & STATE_MEDIUM_PRESENT))
    filmstrip_carrier = property(lambda s: bool(s.flags & STATE_FILMSTRIP_CARRIER))

@dataclass(frozen=True)
class _ShadingReference:
    """A shading reference read from the device, ready to correct with.

    Describes the CCD's per-column response under the scanner's current
    settings, so it outlives a single scan but not the device session -- hence
    cached on the Scanner, as the C caches it on the open handle
    (pieusb_specific.h:292-294).

    `pixels_per_line` is the CCD-native width it was read at, kept so a reference
    cannot be applied to a pass the device sized differently.
    """
    ref: dict[int, numpy.ndarray]
    mean: dict[int, float]
    pixels_per_line: int

class Scanner:
    def __init__(self, info: DeviceInfo) -> None:
        self.dev = UASDevice(info.dev)
        self.scan_thread: threading.Thread | None = None
        self.on_update: Callable[[UpdateData], None] | None = None
        self.on_complete: Callable[[ScanResult], None] | None = None
        self.scan_in_progress: bool = False
        # Set by cancel(), polled by the worker between chunk reads. An Event
        # because it crosses from the caller's thread to the worker's.
        self.cancel_requested = threading.Event()
        self.shading_params: list[dict] | None = None
        self.closed: bool = False
        # The most recent shading reference a pass acquired. What
        # 'reuse_calibration' reuses.
        self._shading: _ShadingReference | None = None
        # Whether SLIDE INIT has been sent since this Scanner was opened.
        self._transport_initialized: bool = False
        # The R/G/B exposure multipliers in force, inside Scanner._exposure().
        self._applied_factors: tuple[float, float, float] | None = None
        # The film base the last metering exposed for; None when it metered the
        # picture's percentile instead.
        self.film_base: FilmBase | None = None
        self.info = info
        self.params = generate_options(info.inquiry, info.capabilities)
        self.dev.open()

    def __enter__(self) -> "Scanner":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def _option(self, name: str) -> Parameter | None:
        """The option called `name`, or None if it is ordinary object state.

        Reads `params` out of __dict__ directly: __setattr__ runs for every
        assignment in __init__, including `self.params` itself, so normal
        attribute lookup would recurse into __getattr__ before the table exists.
        """
        params = self.__dict__.get('params')
        if params is None:
            return None
        try:
            return params[name]
        except KeyError:
            return None

    def __setattr__(self, key, value) -> None:
        par = self._option(key)
        if par is None:
            # Not an option -- real attribute (self.dev, self.scan_thread, ...).
            object.__setattr__(self, key, value)
            return
        if self.scan_in_progress:
            raise ScanInProgress(f"cannot set option '{key}' while a scan is running")
        if type(value) is not par.opt.type:
            raise TypeError(f"Invalid type provided to option '{par.opt.name}' (got {type(value)}, expected {par.opt.type})")
        if not par.opt.validate(value):
            raise ValueError(f"Invalid value provided to option '{par.opt.name}'")
        par.value = value
        par.assigned = True

    def __getattr__(self, name):
        """Read an option's value. Only called when normal lookup already failed.

        Returns the value, not the Parameter, so `scanner.resolution` round-trips
        through `scanner.resolution = ...`; `scanner.params` remains the table for
        introspection. Unknown names raise AttributeError, not KeyError, so
        hasattr()/getattr(default) probes keep working.
        """
        par = self._option(name)
        if par is None:
            raise AttributeError(
                f"{type(self).__name__!r} object has no attribute or option {name!r}"
            )
        return par.value

    def _why_not_ready(self) -> CheckCondition | None:
        """None if the device is ready, else the NOT READY sense explaining why.

        Any other sense is a real error and propagates.
        """
        try:
            self.dev.command(SCSI_TEST_UNIT_READY, cdb_length=0)
            return None
        except CheckCondition as e:
            if e.not_ready:
                return e
            raise

    def ready(self) -> bool:
        """Whether the device can start a scan right now.

        Every NOT READY sense maps to False, warming up included; use
        wait_ready() if you need to know which.
        """
        return self._why_not_ready() is None

    def wait_ready(self, timeout_s: float = 180) -> None:
        """Poll until the device is ready, or raise once timeout_s has passed.

        Raises WarmingUp if the lamp was still warming up when time ran out, and
        DeviceNotReady for any other reason. WarmingUp is a subclass of the
        latter, so `except DeviceNotReady` catches both.
        """
        log.debug("  [wait_ready] polling TEST UNIT READY...")
        deadline = time.time() + timeout_s
        while True:
            reason = self._why_not_ready()
            if reason is None:
                log.debug("  [wait_ready] ready")
                return
            if time.time() >= deadline:
                break
            time.sleep(1.0)

        if reason.warming_up:
            raise WarmingUp(f"scanner still warming up after {timeout_s}s")
        raise DeviceNotReady(f"scanner did not become ready within {timeout_s}s ({reason})")

    def _slide(self, action: int, value: int = 0x01, byte3: int = 0x00) -> None:
        """Issue one slide-transport command, returning once the mechanism is idle.

        `value` is the action's parameter byte: a focus position for SLIDE_INIT, a
        frame count for SLIDE_NEXT and SLIDE_PREV, a step count for the offsets.
        `byte3` has no known meaning; both vendors leave it 0 except on SLIDE_PREV,
        where the one captured payload sets it to 1.
        """
        log.debug(f"[slide] {SLIDE_ACTION_NAMES.get(action, f'0x{action:02x}')} "
                  f"value {value} byte3 {byte3}")
        started = time.monotonic()
        self.dev.command(SCSI_SLIDE, out_data=bytes([action, value, 0x00, byte3]))
        if action == SLIDE_INIT:
            self._transport_initialized = True
        self.wait_ready()
        log.debug(f"[slide] idle after {time.monotonic() - started:.1f}s")

    def transport_state(self) -> TransportState | None:
        """READ STATE, decoded, or None on a scanner without a film transport.

        Raises CheckCondition if the device is not ready -- a warming lamp
        answers this command with NOT READY rather than reporting state.
        """
        if not self.info.capabilities.film_transport:
            return None
        raw = self.dev.command(SCSI_READ_STATE, in_size=READ_STATE_SIZE,
                               cdb_length=READ_STATE_SIZE)
        state = TransportState(
            frame=raw[2],
            focus=raw[3],
            focus_max=raw[4],
            flags=raw[6],
            raw=bytes(raw),
        )
        log.debug(f"[transport] READ STATE {state.raw.hex(' ')}: frame {state.frame}, "
                  f"focus {state.focus}/{state.focus_max}, flags 0x{state.flags:02x}, "
                  f"medium present {state.medium_present}, "
                  f"filmstrip carrier {state.filmstrip_carrier}")
        return state

    def focus_range(self) -> tuple[int, int] | None:
        """The focus positions this scanner accepts, or None if it has no focus.

        The upper bound depends on the carrier currently loaded, so it is read
        from the device on every call.
        """
        if not self.info.capabilities.focus:
            return None
        state = self.transport_state()
        if state is None or state.focus_max < FOCUS_MIN:
            return None
        return (FOCUS_MIN, state.focus_max)

    def set_focus(self, value: int) -> None:
        """Move the focus motor, blocking until it is idle.

        Raises PieusbError on a scanner without focus, ScanInProgress while a scan
        is running, and ValueError outside the range the device reports.
        """
        if not self.info.capabilities.focus:
            raise PieusbError(f"{self.info.model} has no focus control")
        if self.scan_in_progress:
            raise ScanInProgress("cannot move the focus while a scan is running")
        limits = self.focus_range()
        if limits is not None and not (limits[0] <= value <= limits[1]):
            raise ValueError(
                f"focus {value} is outside the {limits[0]}..{limits[1]} this "
                f"scanner reports"
            )
        self._slide(SLIDE_INIT, value)

    def _focus_for_scan(self) -> int:
        """The focus byte for this pass's SLIDE INIT."""
        if not self.info.capabilities.focus:
            return DEFAULT_FOCUS
        wanted = self.params['focus'].value
        limits = self.focus_range()
        if limits is not None and wanted > limits[1]:
            log.warning(f"[scan] focus {wanted} exceeds the {limits[1]} this "
                        f"carrier allows; using {limits[1]}")
            return limits[1]
        return wanted

    def _require_slide_transport(self) -> None:
        """Raise unless the caller may drive the slide transport right now."""
        if not self.info.capabilities.film_transport:
            raise PieusbError(f"{self.info.model} has no slide transport")
        if self.scan_in_progress:
            raise ScanInProgress("cannot drive the slide transport while a scan is running")

    def init_transport(self) -> None:
        """Prepare the slide transport, blocking until it is idle again.

        Each scan pass does this for itself, so calling it directly is for priming
        the transport before any scan has run, or for recovering after the
        magazine has been handled by hand.
        """
        self._require_slide_transport()
        self._slide(SLIDE_INIT, self._focus_for_scan())

    def advance(self, frames: int = 1) -> int:
        """Advance the medium by `frames`, blocking until the transport is idle.

        scan() never moves the medium; the caller decides when it does. A batch can
        therefore keep one frame in the gate for as many passes as it wants, and
        advance once it is done with it.

        Returns how many frames the medium moved, read off the READ STATE frame
        counter either side of the move: fewer than `frames` when the strip ended
        on the way. Raises EndOfStrip when it moved none. At the end of a strip a
        ProScan 4000 or RPS 10M accepts NEXT, refuses it within 0.2 s, and leaves
        the counter and the film where they were. A slide magazine is unmeasured.

        NEXT moves a fixed distance rather than looking for the next gap, so an
        offset still in force when it is sent carries over to the next frame.

        Raises ScanInProgress while a scan is running -- the worker thread owns the
        device -- and PieusbError on a scanner without a slide transport.
        """
        self._require_slide_transport()
        before, after = self._step_frames(SLIDE_NEXT, frames)
        moved = after - before
        if moved <= 0:
            raise EndOfStrip(after)
        if moved < frames:
            log.info(f"[transport] NEXT {frames} moved {moved}: the strip ended on the way")
        return moved

    def rewind(self, frames: int = 1) -> int:
        """Move the medium back by `frames`, blocking until the transport is idle.

        Returns how many frames the medium moved back, read off the frame counter:
        fewer than `frames`, down to 0, when it reached the first frame first.
        """
        self._require_slide_transport()
        before, after = self._step_frames(SLIDE_PREV, frames)
        return before - after

    def _step_frames(self, action: int, frames: int) -> tuple[int, int]:
        """NEXT or PREV, returning the frame counter before and after."""
        if not 1 <= frames <= 0xFF:
            raise ValueError(f"frames must be 1..255, got {frames}")
        if not self._transport_initialized:
            self._slide(SLIDE_INIT, self._focus_for_scan())
        before = self.transport_state().frame
        self._slide(action, frames, byte3=0x01 if action == SLIDE_PREV else 0x00)
        return before, self.transport_state().frame

    def offset_frame(self, mm: float) -> None:
        """Shift the medium off its frame position by `mm` of nominal travel.

        The step is 1/240 inch and the move is relative; a positive `mm` moves
        the film the way NEXT does. What that does to the image depends on the
        model: shift_image() takes a shift in image terms instead. Out-and-back
        moves do not quite return: each round trip ends 0.2-0.6 mm off, always
        the same way, so avoid undoing and redoing a correction.
        """
        self._require_slide_transport()
        steps = int(abs(mm) * OFFSET_STEPS_PER_MM)
        if not 1 <= steps <= 0xFF:
            raise ValueError(
                f"{mm} mm is {steps} steps of 1/{OFFSET_STEPS_PER_INCH} inch; "
                f"the field carries 1..255"
            )
        self._slide(SLIDE_OFFSET_FORWARD if mm > 0 else SLIDE_OFFSET_BACK, steps)

    def shift_image(self, mm: float) -> float:
        """Move the film so that the image content shifts `mm` towards +x, as
        `pieusb.framing.FramePosition.shift_mm` asks, and return the shift the
        whole steps sent amount to.

        Converts with the model's measured `Capabilities.offset_mm_per_step`.
        Raises PieusbError on a model where it is unmeasured, and ValueError for
        a shift under half a step or over 255 steps.
        """
        self._require_slide_transport()
        per_step = self.info.capabilities.offset_mm_per_step
        if per_step is None:
            raise PieusbError(f"{self.info.model}: the offset step is unmeasured; "
                              f"use offset_frame()")
        steps = round(mm / per_step)
        if not 1 <= abs(steps) <= 0xFF:
            raise ValueError(f"a {mm} mm shift is {abs(steps)} steps of {abs(per_step)} mm; "
                             f"the field carries 1..255")
        self._slide(SLIDE_OFFSET_FORWARD if steps > 0 else SLIDE_OFFSET_BACK, abs(steps))
        return steps * per_step

    def eject(self, direction: EjectDirection = EjectDirection.UP) -> None:
        """Eject the medium, blocking until the transport is idle."""
        self._require_slide_transport()
        action = (SLIDE_EJECT_UP if direction is EjectDirection.UP
                  else SLIDE_EJECT_DOWN)
        self._slide(action, 0x00)

    def autofocus(self) -> AutofocusResult:
        """Run the device's focus sweep, then adopt the sharpest position as 'focus'.

        This is the vendor's autofocus pass: MODE SELECT byte 10 set, the focus
        parked at its minimum, and a pass that returns as many lines as the focus
        range -- 30 with a filmstrip carrier on a ProScan 4000. The host scores
        each line's sharpness in red, as the vendor software does, smooths the
        scores over neighbouring positions (AUTOFOCUS_SMOOTHING) and picks the
        best. A sweep too flat to trust (AUTOFOCUS_MIN_DEPTH) leaves 'focus'
        alone, with a warning.

        The sweep covers the lower half of the current frame, full width, which on
        the default frame is the region CyberView sweeps. It runs with the
        device's own shading analysis and leaves every option but 'focus' as it
        found it, the cached shading reference included.

        Experimental. On a ProScan 4000 with a filmstrip carrier, line i is within
        one step of focus minimum + i up to about focus 20, where the sweep stops;
        later lines repeat that position. On an RPS 10M with a filmstrip carrier,
        line i is focus minimum + i to within the 13-16 a manual sweep found
        sharpest. Untested with a slide carrier and on other models.

        Raises PieusbError on a scanner without focus or that reports no focus
        range, and ScanInProgress while a scan is running.
        """
        if not self.info.capabilities.focus:
            raise PieusbError(f"{self.info.model} has no focus control")
        if self.scan_in_progress:
            raise ScanInProgress("cannot autofocus while a scan is running")
        limits = self.focus_range()
        if limits is None:
            raise PieusbError("the scanner reports no focus range")

        tl_y, br_y = self.params['tl_y'].value, self.params['br_y'].value
        cached = self._shading
        try:
            with self._overridden_options(
                resolution=min(AUTOFOCUS_RESOLUTION, self.info.inquiry.max_resolution_x),
                mode='rgb',
                color_depth=AUTOFOCUS_COLOR_DEPTH,
                sharpen=False,
                fast_infrared=False,
                reuse_calibration=False,
                tl_y=(tl_y + br_y) // 2,
                focus=limits[0],
            ):
                sweep = self._scan_pass(time.monotonic(), lambda update: None,
                                        autofocus=True)
        finally:
            self._shading = cached
        if sweep.rgb is None:
            raise PieusbError("the autofocus pass returned no image data")

        positions = autofocus_positions(sweep.rgb.shape[0], *limits)
        scores = line_sharpness(sweep.rgb)
        best, depth, smoothed = autofocus_pick(scores, positions)
        peaked = depth >= AUTOFOCUS_MIN_DEPTH
        if peaked:
            self.params['focus'].value = best
            self.params['focus'].assigned = True
            log.info(f"[autofocus] {len(positions)} lines over focus "
                     f"{limits[0]}..{limits[1]}; sharpest at {best}")
        else:
            log.warning(f"[autofocus] the sweep over focus {limits[0]}..{limits[1]} is "
                        f"too flat to trust (best {depth:.3f}x the worst, below "
                        f"{AUTOFOCUS_MIN_DEPTH}); keeping focus "
                        f"{self.params['focus'].value}. Sweep a region with more detail")
        return AutofocusResult(focus=self.params['focus'].value, peaked=peaked,
                               positions=tuple(positions),
                               scores=tuple(float(v) for v in scores),
                               smoothed=tuple(float(v) for v in smoothed),
                               depth=depth, lines=sweep.rgb)

    def _start_scan(self) -> None:
        self.dev.command(SCSI_SCAN, cdb_length=1)

    def _get_scanned_lines_cmd(self, lines, size) -> bytes:
        return self.dev.command(SCSI_READ, in_size=size, cdb_length=lines)

    def _iter_scanned_lines(self, total_lines, bytes_per_line, label="lines") -> Iterator[tuple[bytes, int]]:
        """Read total_lines in <=MAX_LINES_PER_READ chunks, yielding (bytes, n_lines).

        A generator so the pixel read can decode and discard each chunk as it
        arrives: a full-resolution 16-bit RGBI scan is gigabytes, and holding the
        raw bytes alongside the decoded image would double that.
        """
        remaining = total_lines
        done = 0
        while remaining > 0:
            lines_this_read = min(MAX_LINES_PER_READ, remaining)
            n_bytes = lines_this_read * bytes_per_line
            chunk = self._get_scanned_lines_cmd(lines=lines_this_read, size=n_bytes)
            done += lines_this_read
            remaining -= lines_this_read
            log.debug(f"[scan]   {label}: {done}/{total_lines} lines read (remaining {remaining})")
            yield chunk, lines_this_read

    def _get_scanned_lines(self, total_lines, bytes_per_line, label="lines",
                           report: Callable[[float], None] | None = None) -> bytes:
        collected = bytearray()
        done = 0
        for chunk, n_lines in self._iter_scanned_lines(total_lines, bytes_per_line, label):
            collected.extend(chunk)
            done += n_lines
            if report is not None:
                report(done / total_lines)
        return bytes(collected)

    def _get_shading_parms(self) -> list[dict]:
        prep = bytes([SCSI_CALIBRATION_INFO | 0x80]) + bytes(5)
        self.dev.command(SCSI_WRITE, out_data=prep, cdb_length=6)
        raw = self.dev.command(SCSI_READ, in_size=32, cdb_length=32)
        n_entries = raw[4]
        entry_size = raw[5]
        entries = []
        for k in range(n_entries):
            off = 8 + entry_size * k
            entries.append({
                "type": raw[off],
                "n_lines": raw[off + 3],
                "pixels_per_line": struct.unpack_from("<H", raw, off + 4)[0],
            })
        return entries

    def _get_scan_parameters(self):
        raw = self.dev.command(SCSI_PARAM, in_size=18, cdb_length=18)
        return {
            "width": struct.unpack_from("<H", raw, 0)[0],
            "lines": struct.unpack_from("<H", raw, 2)[0],
            "bytes_per_line": struct.unpack_from("<H", raw, 4)[0],
            # Lines sitting in the device's own buffer, non-zero only while a scan
            # is in progress (pieusb_scancmd.h:140).
            "available_lines": struct.unpack_from("<H", raw, 14)[0],
        }

    def _get_ccd_mask(self, mask_size):
        return self.dev.command(SCSI_COPY, in_size=mask_size, cdb_length=mask_size)

    def _get_gain_offset(self):
        raw = self.dev.command(SCSI_READ_GAIN_OFFSET, in_size=103, cdb_length=103)
        # Saturation levels: the average R/G/B the firmware reached while
        # optimising its exposure times, targeting >=90% of full scale for R and B
        # and >=80% for G (pieusb_scancmd.h:188-192). Known to read 0-0-0 on a PIE
        # ProScan 10T, warm or cold; nothing here depends on them.
        saturation_rgb = struct.unpack_from("<3H", raw, 54)
        exposure_rgb = struct.unpack_from("<3H", raw, 60)
        offset_rgb = tuple(raw[66:69])
        gain_rgb = tuple(raw[72:75])
        light = raw[75]
        exposure_i = struct.unpack_from("<H", raw, 98)[0]
        offset_i = raw[100]
        gain_i = raw[102]
        return {
            "saturation_level": saturation_rgb,
            "exposure_time": exposure_rgb + (exposure_i,),
            "offset": offset_rgb + (offset_i,),
            "gain": gain_rgb + (gain_i,),
            "light": light,
        }

    def stop_scan(self) -> None:
        try:
            self.dev.command(SCSI_SCAN, cdb_length=0)
        except CheckCondition as e:
            # Expected to raise key=0x0b, code=0x00, qualifier=0x06.
            if e.sense_key != 0x0b or e.sense_code != 0 or e.sense_qualifier != 0x06:
                raise

    def cancel(self) -> None:
        """Ask a running scan to stop; returns immediately, no-op if idle.

        Does not touch the device -- pyusb is not thread-safe and the worker owns
        it -- so this takes effect at the next chunk boundary (<=255 lines), where
        the worker issues STOP SCAN and fires on_complete with result.cancelled
        set. Pair with wait() to block until that happens.
        """
        if self.scan_in_progress:
            self.cancel_requested.set()

    def wait(self, timeout: float | None = None) -> bool:
        """Block until the running scan finishes, is cancelled or fails.

        Returns True if the worker is done (or none was running), False if it is
        still going when `timeout` expires. Never raises the scan's own failure:
        that arrives as `result.error` through on_complete.
        """
        thread = self.scan_thread
        if thread is None:
            return True
        if thread is threading.current_thread():
            # join() would deadlock. Only reachable from a callback, which runs
            # on the worker.
            raise PieusbError(
                "wait() called from the scan worker thread; a callback cannot wait on its own scan"
            )
        thread.join(timeout)
        return not thread.is_alive()

    def close(self, timeout: float | None = CLOSE_WAIT_S) -> None:
        """Cancel any running scan, wait for the worker, then release the interface.

        Idempotent. The wait matters: releasing the USB interface while the worker
        still owns the device would pull it out from under a running scan. If the
        worker does not stop within `timeout` the interface is released anyway and
        the overrun is logged as an error.
        """
        if self.closed:
            return
        if self.scan_in_progress:
            log.debug("[close] cancelling the running scan...")
            self.cancel()
            if not self.wait(timeout):
                log.error(f"[close] worker still running after {timeout}s; releasing the "
                          f"interface anyway -- the device may be left mid-scan")
        self.closed = True
        self.dev.close()

    def _emit(self, update: UpdateData) -> None:
        """Fire on_update, never letting a caller's callback abort the scan."""
        try:
            self.on_update(update)
        except Exception:
            log.exception("[scan] on_update callback raised; continuing the scan")

    def _emit_metering(self, update: UpdateData) -> None:
        """Relabel a metering pass's updates, so a progress bar sees one sweep for
        the metering pass and a second for the real scan rather than two
        indistinguishable SCANNING sweeps."""
        self._emit(replace(update, phase=ScanPhase.METERING))

    def _result(self, rgb=None, ir=None, *, started: float, width: int = 0, height: int = 0,
                shading_corrected: bool = False, shading_acquired: bool = False,
                cancelled: bool = False,
                error: Exception | None = None) -> ScanResult:
        """Build a ScanResult, filling in the settings the scan actually ran with."""
        factors = self._applied_factors
        return ScanResult(
            rgb=rgb,
            ir=ir,
            width=width,
            height=height,
            mode=self.params['mode'].value,
            color_depth=self.params['color_depth'].value,
            resolution=self.params['resolution'].value,
            shading_corrected=shading_corrected,
            shading_acquired=shading_acquired,
            # Read here rather than passed in: _result is called from inside the
            # _exposure() context, so these are the translated values.
            exposure=tuple(self.params[f'exp_time_{s}'].value for s in CHANNEL_SUFFIX),
            gain=tuple(self.params[f'gain_{s}'].value for s in CHANNEL_SUFFIX),
            exposure_factor=factors,
            cancelled=cancelled,
            error=error,
            duration_s=time.monotonic() - started,
        )

    def _scan_worker(self) -> None:
        """Thread entry point: run the scan and deliver exactly one ScanResult.

        Nothing may escape this method: there is no caller left on the stack, so
        every failure is caught here, STOP SCAN is attempted, and the exception
        reaches the caller as result.error.
        """
        started = time.monotonic()
        try:
            try:
                result = self._run_scan(started)
            except Exception as e:
                log.exception("[scan] scan failed")
                try:
                    self.stop_scan()
                except Exception:
                    log.exception("[scan] STOP SCAN after the failure also failed")
                result = self._result(started=started, error=e)
        finally:
            # Cleared before on_complete so a callback may start the next scan.
            self.scan_in_progress = False

        try:
            self.on_complete(result)
        except Exception:
            log.exception("[scan] on_complete callback raised")

    @contextmanager
    def _overridden_options(self, **values):
        """Temporarily force option values, restoring them on the way out.

        Writes Parameter.value directly rather than through __setattr__, which
        refuses assignment while a scan is in progress. Type and range checks are
        therefore skipped; callers pass constants, not user input.
        """
        saved = {name: self.params[name].value for name in values}
        for name, value in values.items():
            self.params[name].value = value
        try:
            yield
        finally:
            for name, value in saved.items():
                self.params[name].value = value

    @contextmanager
    def _saved_options(self, *names):
        """Restore these options on the way out, whatever ran in between.

        _overridden_options() for when the new values are not known up front:
        auto_exp reads them off the device.
        """
        saved = {name: self.params[name].value for name in names}
        try:
            yield
        finally:
            for name, value in saved.items():
                self.params[name].value = value

    def _relative_ceilings(self, exposure_time) -> list[int]:
        """The largest exp_rel_* per channel before the device's Timer 1 overflows.

        exp_rel scales the exposure time the scanner reports for itself into a
        16-bit count, and past the point where that product overflows the exposure
        drops instead of rising (see option.max_relative_exposure). A scanner that
        has not finished warming up reports 0, which says nothing about the
        register it uses, so the ceiling for its usual exposure time is assumed.
        """
        if not all(exposure_time[c] > 0 for c in range(3)):
            log.warning(f"[exposure] the scanner reports no exposure time "
                        f"({exposure_time}), so where exp_rel overflows its Timer 1 is "
                        f"unknown; assuming the ceiling of {MAX_RELATIVE_EXPOSURE} that "
                        f"its usual exposure time implies")
        return [max_relative_exposure(exposure_time[c]) if exposure_time[c] > 0
                else MAX_RELATIVE_EXPOSURE for c in range(3)]

    @contextmanager
    def _exposure(self, factors: tuple[float, float, float] | None = None,
                  skip: bool = False):
        """Translate the generic exposure into exp_rel_*, for the duration of one pass.

        exp_rel_* = 100 x the factor, clamped to what the device can do. It is
        honoured whether or not the pass runs the shading analysis, on every
        model measured (ProScan 10T, ProScan 4000), so every exposure takes
        effect in the pass that asks for it. The device clamps below 100 -- 50
        behaves as 100 -- so a factor under 1.0 cannot shorten the exposure.
        `factors` are the R/G/B multipliers, by default what 'exposure' and
        'exposure_r/g/b' ask for. A device field the caller set directly is left
        alone and wins.

        A pass that runs the shading analysis uses the device's own exposure,
        gain and offset, whatever is sent. One that skips it (`skip`) scans with
        the options as sent, and anything but the settings the cached reference
        was taken with shifts level and colour (ProScan 10T: 0.42, 0.57, 0.39 of
        the analysing pass, sent the option defaults). So a skipping pass is sent
        the scanner's own settings, unscaled. The device is read only then, or
        when some channel asks for more than 1.0, which needs its Timer 1 ceiling.

        Every option is restored on the way out. The multipliers actually sent are
        held in self._applied_factors meanwhile, for _result().
        """
        if factors is None:
            factors = self.params.exposure_factors()
        reported = self._get_gain_offset() if skip else None
        values, kept = ({}, []) if reported is None else self._device_settings(
            reported, reported["exposure_time"][:3])
        applied = []
        ceilings = None
        for c, (suffix, factor) in enumerate(zip(('r', 'g', 'b'), factors)):
            name = f'exp_rel_{suffix}'
            par = self.params[name]
            if par.assigned:
                kept.append(f"{name}={par.value}")
                applied.append(par.value / DEFAULT_RELATIVE_EXPOSURE)
                continue
            wanted = round(DEFAULT_RELATIVE_EXPOSURE * factor)
            if wanted > DEFAULT_RELATIVE_EXPOSURE and ceilings is None:
                reported = reported or self._get_gain_offset()
                ceilings = self._relative_ceilings(reported["exposure_time"])
            ceiling = ceilings[c] if ceilings else MAX_RELATIVE_EXPOSURE
            value = min(max(wanted, DEFAULT_RELATIVE_EXPOSURE), ceiling)
            if wanted > ceiling:
                log.warning(f"[exposure] {CHANNEL_NAMES[c]}: {factor:g} is above the "
                            f"{ceiling / DEFAULT_RELATIVE_EXPOSURE:.2f} this scanner "
                            f"reaches; using that")
            elif wanted < DEFAULT_RELATIVE_EXPOSURE:
                log.warning(f"[exposure] {CHANNEL_NAMES[c]}: {factor:g} is below 1.0, "
                            f"and this scanner can only lengthen its exposure; using 1.0")
            values[name] = value
            applied.append(value / DEFAULT_RELATIVE_EXPOSURE)
        if kept:
            log.info(f"[exposure] set directly, overriding 'exposure' or the "
                     f"scanner's own settings: {', '.join(kept)}")
        self._applied_factors = tuple(applied)
        try:
            with self._overridden_options(**values):
                yield
        finally:
            self._applied_factors = None

    def _device_settings(self, reported, exposure) -> tuple[dict, list[str]]:
        """Option values seeding the scanner's own settings, with `exposure` as
        exp_time_r/g/b, and the options kept because the caller assigned them.

        Nothing is seeded if the scanner has not run a shading analysis yet, which
        is when it picks them.
        """
        values, kept = {}, []
        if not all(reported["exposure_time"][c] > 0 for c in range(3)):
            return values, kept

        def seed(name, value):
            # An option the caller assigned is theirs, not the device's;
            # seeding over it would make gain and exposure unsettable.
            par = self.params[name]
            if not par.assigned:
                values[name] = value
            else:
                kept.append(f"{name}={par.value}")

        for c, suffix in enumerate(('r', 'g', 'b')):
            seed(f'exp_time_{suffix}', exposure[c])
        if reported["exposure_time"][3] > 0:
            seed('exp_time_i', reported["exposure_time"][3])
        for c, suffix in enumerate(CHANNEL_SUFFIX):
            if reported["gain"][c]:
                seed(f'gain_{suffix}', reported["gain"][c])
            if reported["offset"][c]:
                seed(f'offset_{suffix}', reported["offset"][c])
        if reported["light"]:
            seed('light', reported["light"])
        return values, kept

    def _meter_exposure(self, started: float) -> ScanResult | None:
        """Run a metering pass and set exposure_r/g/b from what it measured.

        Each channel's factor puts a reference level at METERING_TARGET of full
        scale, clamped to 1.0 below and the Timer 1 ceiling above with the
        shortfall reported. The reference is the film base where the preview
        shows a negative's gap (pieusb.framing.film_base): the thinnest film on
        the strip, so nothing on any frame can clip, and the same from frame to
        frame to within 1-3% on an RPS 10M. Otherwise it is the
        METERING_PERCENTILE of each colour plane: on a slide, or where a holder
        masks the gaps. self.film_base records which.

        Returns None once exposure_r/g/b have been set, or the metering pass's
        own ScanResult if it was cancelled. Failures propagate.
        """
        preview = self._metering_preview(started, self._emit_metering)
        if preview.cancelled:
            return preview
        self._set_metered_exposure(preview, self._base_in(preview))
        return None

    def _metering_preview(self, started: float, emit) -> ScanResult:
        """The metering pass: the frame at preview resolution, at the scanner's
        own exposure."""
        preview_resolution = self.info.inquiry.preview_scan_resolution
        if not self.params['resolution'].opt.validate(preview_resolution):
            fallback = min(self.params['resolution'].value, PREVIEW_RESOLUTION_FALLBACK)
            log.warning(f"[autoexp] the scanner reports an unusable preview resolution "
                        f"({preview_resolution}); metering at {fallback} dpi instead")
            preview_resolution = fallback
        log.info(f"[autoexp] metering pass at {preview_resolution} dpi")

        with self._overridden_options(
            resolution=preview_resolution,
            sharpen=False,
            fast_infrared=False,
        ), self._exposure((DEFAULT_EXPOSURE_FACTOR,) * 3):
            preview = self._scan_pass(started, emit)
        if not preview.cancelled and preview.rgb is None:
            raise PieusbError("metering pass returned no image data")
        return preview

    def _base_in(self, preview: ScanResult) -> FilmBase | None:
        """The film base in a metering preview, or None where it shows none or
        the picture outshines what was taken for it."""
        px_per_mm = preview.resolution / MM_PER_INCH
        base = film_base(preview.rgb, find_bands(preview.rgb, px_per_mm))
        if base is None:
            return None
        if not base.consistent:
            log.warning(f"[autoexp] a gap at {base.level} of full scale is darker than "
                        f"the picture ({base.picture_top}), so it is not the film base; "
                        f"metering the picture instead")
            return None
        return base

    def _set_metered_exposure(self, preview: ScanResult, base: FilmBase | None) -> None:
        """exposure_r/g/b from a metering preview, on `base` if given, else on
        each plane's METERING_PERCENTILE."""
        ceilings = self._relative_ceilings(self._get_gain_offset()["exposure_time"])
        # What the preview ran at, normally 1.0; a device field set directly moves it.
        metered_at = preview.exposure_factor or (DEFAULT_EXPOSURE_FACTOR,) * 3
        full_scale = numpy.iinfo(preview.rgb.dtype).max
        target = METERING_TARGET * full_scale
        self.film_base = base
        if base is not None:
            log.info(f"[autoexp] metering the film base, read off {base.columns} "
                     f"columns of gap")
        for c, name in enumerate(METERED_OPTIONS):
            if base is not None:
                measured = base.level[c] * full_scale
            else:
                measured = float(numpy.percentile(preview.rgb[:, :, c], METERING_PERCENTILE))
            if measured <= 0:
                log.warning(f"[autoexp] channel {CHANNEL_NAMES[c]} metered 0; leaving "
                            f"{name} at {self.params[name].value}")
                continue
            wanted = metered_at[c] * target / measured
            low, high = 1.0, ceilings[c] / DEFAULT_RELATIVE_EXPOSURE
            value = min(max(wanted, low), high)
            log.info(f"[autoexp] {CHANNEL_NAMES[c]}: metered {measured:.0f}/{full_scale} "
                     f"({measured / full_scale * 100:.1f}%), {name} -> {value:.3f}")
            if wanted > high:
                log.warning(f"[autoexp] channel {CHANNEL_NAMES[c]} wants {wanted:.3f}, "
                            f"above the {high:.3f} this scanner reaches; it will be "
                            f"{wanted / high:.2f}x under the target")
            elif wanted < low:
                log.warning(f"[autoexp] channel {CHANNEL_NAMES[c]} is already brighter "
                            f"than the target and this scanner cannot shorten its "
                            f"exposure; it will be over-exposed by {low / wanted:.2f}x")
            self.params[name].value = value

    def meter_base(self) -> FilmBase:
        """Meter exposure_r/g/b on a negative's film base and keep them: an
        exposure to hold for the whole strip, which no frame on it can clip.

        Meters the frame in the gate if a gap shows there. Otherwise moves the
        film BASE_OFFSET_STEPS forward, meters there and moves it back; the
        return is a reversal and lands within about 0.35 mm on an RPS 10M. Sets
        and assigns exposure_r/g/b, so leave auto_exp off for the scans that
        follow, and returns the base it metered (also self.film_base).

        Raises PieusbError on a scanner without a film transport or when no gap
        shows either way (a slide, a blank stretch), ScanInProgress while a scan
        is running, or if a cancel() stopped a metering pass.
        """
        self._require_slide_transport()
        started = time.monotonic()
        preview = self._metering_preview(started, lambda update: None)
        base = None if preview.cancelled else self._base_in(preview)
        if base is None and not preview.cancelled:
            log.info(f"[autoexp] no film base in the gate; looking "
                     f"{BASE_OFFSET_STEPS} steps forward")
            self._slide(SLIDE_OFFSET_FORWARD, BASE_OFFSET_STEPS)
            try:
                preview = self._metering_preview(started, lambda update: None)
            finally:
                self._slide(SLIDE_OFFSET_BACK, BASE_OFFSET_STEPS)
            base = None if preview.cancelled else self._base_in(preview)
        if preview.cancelled:
            raise PieusbError("meter_base() was cancelled")
        if base is None:
            raise PieusbError("no film base in view, in the gate or "
                              f"{BASE_OFFSET_STEPS} steps forward")
        self._set_metered_exposure(preview, base)
        for name in METERED_OPTIONS:
            self.params[name].assigned = True
        return base

    def _skips_analysis(self, reuse: bool | None) -> bool:
        """Whether a pass asks to skip the shading analysis: `reuse`, or the
        'reuse_calibration' option when None, and only with a reference cached."""
        if reuse is None:
            reuse = self.params['reuse_calibration'].value
        return reuse and self._shading is not None

    def _run_scan(self, started: float) -> ScanResult:
        """Run the passes a scan needs: a metering pass for auto_exp, then the
        real one. Raises; _scan_worker catches. The metered exposure_r/g/b belong
        to this scan alone and are restored afterwards."""
        with self._saved_options(*METERED_OPTIONS):
            if self.params['auto_exp'].value:
                cancelled = self._meter_exposure(started)
                if cancelled is not None:
                    return cancelled
            with self._exposure(skip=self._skips_analysis(None)):
                return self._scan_pass(started, self._emit)

    def _scan_pass(self, started: float, emit: Callable[[UpdateData], None],
                   autofocus: bool = False, reuse: bool | None = None) -> ScanResult:
        """One pass over the medium, from configuration to a finished ScanResult.

        Shared by the real scan and the metering pass, which differ only in the
        options in force and in how their progress is labelled -- hence `emit`.

        Whether the pass acquires its own shading reference or reuses the cached
        one is decided here, from 'reuse_calibration' and what the cache holds,
        with the scanner getting the last word (the must_calibrate branch below).
        Either way the image is corrected from self._shading, so the choice costs a
        calibration pass, never a correction.

        Raises; _scan_worker turns that into result.error. Follows sane_start()
        (pieusb.c:865-1140).
        """
        emit(UpdateData(phase=ScanPhase.CONFIGURING))

        self.shading_params = self._get_shading_parms()
        if not self.shading_params:
            raise PieusbError(
                "GET SHADING PARMS returned no entries; cannot size the shading read or the CCD mask"
            )
        log.debug(f"[scan] shading params: {self.shading_params}")
        # The shading reference is wider than the image: it still holds the CCD's
        # unused pixels, which is what the CCD mask later maps away.
        shading_ppl = self.shading_params[0]["pixels_per_line"]

        if self._shading is not None and self._shading.pixels_per_line != shading_ppl:
            log.warning(f"[scan] the device now reports a {shading_ppl}-pixel shading "
                        f"reference, not {self._shading.pixels_per_line}; dropping the "
                        f"cached one")
            self._shading = None

        # Skipping is what 'reuse_calibration' asks for, but it can only be
        # honoured with a reference already cached: correction happens on the host,
        # so a granted skip with an empty cache would mean raw pixels.
        acquire = not self._skips_analysis(reuse)
        reuse = self.params['reuse_calibration'].value if reuse is None else reuse
        if acquire and reuse:
            log.info("[scan] reuse_calibration is set but no shading reference has been "
                     "acquired on this Scanner yet; running the shading analysis on this pass. Keep the "
                     "Scanner open across scans for later ones to reuse it")

        self.wait_ready()
        # Logged rather than returned on the ScanResult because auto_exp restores
        # these afterwards, making this the only place the values a pass ran with
        # can be observed.
        log.info("[scan] sending exp_rel %s, exposure %s, gain %s, offset %s, light %d",
                 tuple(self.params[f'exp_rel_{s}'].value for s in ('r', 'g', 'b')),
                 tuple(self.params[f'exp_time_{s}'].value for s in CHANNEL_SUFFIX),
                 tuple(self.params[f'gain_{s}'].value for s in CHANNEL_SUFFIX),
                 tuple(self.params[f'offset_{s}'].value for s in CHANNEL_SUFFIX),
                 self.params['light'].value)
        # The quality bit only asks. The scanner answers below.
        set_options(self.dev, self.params, skip_shading_analysis=not acquire,
                    autofocus=autofocus)
        self.wait_ready()

        # Sent before every START SCAN on a transport- or focus-equipped scanner,
        # matching the SANE backend's ordering (pieusb.c:1062-1072).
        if self.info.capabilities.film_transport or self.info.capabilities.focus:
            log.debug("[scan] initialising the slide transport...")
            self._slide(SLIDE_INIT, self._focus_for_scan())

        log.debug("[scan] starting scan...")
        for attempt in range(START_SCAN_ATTEMPTS):
            try:
                self._start_scan()
                break
            except CheckCondition as e:
                if e.warming_up:
                    # Progress against the retry budget, which is all we know: the
                    # scanner does not say how much warming up it has left.
                    emit(UpdateData(phase=ScanPhase.WARMING_UP,
                                    progress=attempt / START_SCAN_ATTEMPTS))
                    log.debug(f"[scan]   still warming up, waiting {START_SCAN_RETRY_S}s "
                              f"(attempt {attempt + 1}/{START_SCAN_ATTEMPTS})...")
                    time.sleep(START_SCAN_RETRY_S)
                    continue
                if e.must_calibrate:
                    # "Calibration disable not granted", not an error: the scanner
                    # will calibrate regardless, so a fresh reference is waiting to
                    # be read. This is what keeps a reused reference from going
                    # stale -- the device says when it wants a new one
                    # (pieusb.c:1091-1092).
                    log.info("[scan]   the scanner refused to skip the shading analysis; "
                             "acquiring a fresh shading reference")
                    acquire = True
                    break
                raise
        else:
            raise WarmingUp(
                f"scanner still warming up after {START_SCAN_ATTEMPTS} START SCAN attempts "
                f"({START_SCAN_ATTEMPTS * START_SCAN_RETRY_S}s)"
            )
        self.wait_ready()

        # --- Calibration: shading reference, CCD mask, GET PARAMETERS ---------
        # The last cancellation point before the pixel read; the shading-reference
        # reads below are short enough not to be worth interrupting mid-way.
        if self.cancel_requested.is_set():
            log.debug("[scan] cancelled before the shading-reference read; stopping")
            self.stop_scan()
            return self._result(started=started, cancelled=True)

        shading_raw = None
        if acquire:
            emit(UpdateData(phase=ScanPhase.CALIBRATING))

            # Shading data is always 16-bit, with the same two-byte channel tag
            # per line as the image data. The C reads 4 * entry[0].nLines
            # (pieusb_specific.c:2078); summing over the entries agrees with that
            # without assuming there are four.
            total_shading_lines = sum(e["n_lines"] for e in self.shading_params)
            shading_bytes_per_line = 2 + shading_ppl * 2
            log.debug(f"[scan] reading shading reference ({total_shading_lines} lines, "
                      f"{total_shading_lines * shading_bytes_per_line} bytes)...")
            shading_raw = self._get_scanned_lines(
                total_shading_lines, shading_bytes_per_line, label="shading",
                report=lambda frac: emit(
                    UpdateData(phase=ScanPhase.CALIBRATING, progress=frac)
                ),
            )
            self.wait_ready()
        else:
            # The scanner granted the skip: nothing to read, no CALIBRATING phase,
            # and the cached reference corrects the image instead.
            log.debug("[scan] shading analysis skipped; correcting from the reference "
                      "cached on this Scanner")

        # Read unconditionally, as sane_start() does (pieusb.c:1149). Only shading
        # correction consumes it, so it may go unused.
        log.debug("[scan] reading CCD mask...")
        ccd_mask = self._get_ccd_mask(shading_ppl)

        # The device's own geometry, rather than one computed from the frame.
        scan_params = self._get_scan_parameters()
        width = scan_params["width"]
        height = scan_params["lines"]
        log.debug(f"[scan] scan parameters: {scan_params}")
        if width <= 0 or height <= 0:
            raise PieusbError(f"GET PARAMETERS reported an empty image ({width}x{height})")
        self.wait_ready()

        # --- Pixel data -------------------------------------------------------
        mode = self.params['mode'].value
        n_planes = MODE_PLANES[mode]
        # '<u1'/'<u2' rather than uint8/uint16: byteOrder is set to Intel in
        # SET MODE, so the samples are little-endian regardless of this host.
        sample_dtype = numpy.dtype('<u1' if self.params['color_depth'].value <= 8 else '<u2')
        raw_bytes_per_line = 2 + width * sample_dtype.itemsize
        total_lines = n_planes * height

        log.debug(f"[scan] reading {n_planes} planes x {height} lines "
                  f"({raw_bytes_per_line} bytes/line incl. 2-byte tag)...")
        emit(UpdateData(phase=ScanPhase.SCANNING, progress=0.0))

        # Lines arrive one per channel in rotation, not as per-plane blocks, and
        # neither the first nor the last read need start or end on a cycle
        # boundary. Every row is placed by its own tag.
        planes = numpy.zeros((n_planes, height, width), dtype=sample_dtype)
        rows_seen = [0, 0, 0, 0]
        unknown_tags: dict[bytes, int] = {}
        overflow = 0
        done = 0

        for chunk, n_lines in self._iter_scanned_lines(
            total_lines, raw_bytes_per_line, label="scan data"
        ):
            for k in range(n_lines):
                off = k * raw_bytes_per_line
                tag = bytes(chunk[off:off + 2])
                channel = TAG_TO_CHANNEL.get(tag)
                if channel is None:
                    unknown_tags[tag] = unknown_tags.get(tag, 0) + 1
                    continue
                row = rows_seen[channel]
                rows_seen[channel] += 1
                if channel >= n_planes or row >= height:
                    overflow += 1
                    continue
                planes[channel, row] = numpy.frombuffer(
                    chunk, dtype=sample_dtype, count=width, offset=off + 2
                )
            done += n_lines
            emit(UpdateData(phase=ScanPhase.SCANNING, progress=done / total_lines))

            if self.cancel_requested.is_set():
                log.debug(f"[scan] cancelled after {done}/{total_lines} lines; stopping")
                self.stop_scan()
                return self._result(started=started, width=width, height=height, cancelled=True)

        if unknown_tags:
            log.warning(f"[scan] unrecognized line tags, rows dropped: {unknown_tags} "
                        f"(expected only {list(TAG_TO_CHANNEL)})")
        if overflow:
            log.warning(f"[scan] {overflow} lines arrived beyond the {n_planes}x{height} "
                        f"the device reported; dropped")
        for c in range(n_planes):
            if rows_seen[c] != height:
                log.warning(f"[scan] channel {CHANNEL_NAMES[c]}: {rows_seen[c]} rows, "
                            f"expected {height}")

        # Build from what arrived, at the smallest common row count, rather than
        # padding a channel that came up short.
        usable_height = min(rows_seen[:n_planes])
        if usable_height == 0:
            raise PieusbError(
                f"no complete channel data recovered from {done} lines read "
                f"(rows per channel: {rows_seen[:n_planes]})"
            )
        if usable_height < height:
            log.warning(f"[scan] image truncated to {usable_height} of {height} lines")
            planes = planes[:, :usable_height, :]

        # --- Post-processing ---------------------------------------------------
        emit(UpdateData(phase=ScanPhase.PROCESSING))

        # A pass that acquired a reference replaces the cache; a pass that skipped
        # leaves the previous one in place. The correction below reads either.
        if shading_raw is not None:
            shading_ref, shading_mean = calculate_shading(shading_raw, shading_ppl)
            if shading_ref is None:
                # Keep the older reference rather than dropping to raw pixels over
                # one bad read: it describes the same CCD.
                log.warning("[scan] no usable shading reference lines in what the device "
                            "returned; keeping the reference already cached, if any")
            else:
                self._shading = _ShadingReference(shading_ref, shading_mean, shading_ppl)

        shading_corrected = False
        if self._shading is None:
            log.warning("[scan] no shading reference available; returning raw pixel data")
        else:
            log.debug(f"[scan] shading mean per channel = "
                      f"{[round(self._shading.mean[c], 1) for c in range(n_planes)]}")
            apply_shading_correction(
                planes, self._shading.ref, self._shading.mean,
                build_width_to_loc(ccd_mask, width)
            )
            shading_corrected = True

        # (planes, h, w) -> (h, w, planes); infrared stays two-dimensional.
        rgb = numpy.ascontiguousarray(numpy.moveaxis(planes[:3], 0, -1))
        ir = numpy.ascontiguousarray(planes[3]) if n_planes > 3 else None

        return self._result(
            rgb, ir,
            started=started,
            width=width,
            height=usable_height,
            shading_corrected=shading_corrected,
            shading_acquired=acquire,
        )

    def scan(self, on_update: Callable[[UpdateData], None], on_complete: Callable[[ScanResult], None]) -> None:
        if self.closed:
            raise PieusbError('Scanner is closed')

        # Raises ParamError if the options are invalid or contradictory.
        self.params.validate()

        if self.scan_in_progress:
            raise ScanInProgress

        reason = self._why_not_ready()
        if reason is not None:
            if reason.warming_up:
                raise WarmingUp
            if reason.not_ready:
                raise DeviceNotReady

        if on_update is None or on_complete is None:
            raise PieusbError('Must set on_update and on_complete callbacks to start a scan')
            
        self.on_update = on_update
        self.on_complete = on_complete
        self.cancel_requested.clear()
        self.scan_thread = threading.Thread(target=self._scan_worker)
        self.scan_in_progress = True
        self.scan_thread.start()
