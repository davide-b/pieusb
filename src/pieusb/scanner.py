from pieusb.calibration import (
    gain_increase,
    percentile_bounds,
    update_gain,
)
from pieusb.option import generate_options, set_options, MODE_PLANES, Parameter
from pieusb.postprocess import (
    apply_shading_correction,
    build_width_to_loc,
    calculate_shading
)
from pieusb.types import (
    DeviceInfo,
    UpdateData,
    ScanResult,
    ScanPhase
)
from pieusb.transport import (
    UASDevice,
    SCSI_READ,
    SCSI_WRITE,
    SCSI_SCAN,
    SCSI_TEST_UNIT_READY,
    SCSI_CALIBRATION_INFO,
    SCSI_READ_GAIN_OFFSET,
    SCSI_PARAM,
    SCSI_COPY
)
from pieusb.exceptions import (
    CheckCondition,
    PieusbError,
    WarmingUp,
    DeviceNotReady,
    ScanInProgress
)

import logging
import time
import threading
import struct

import numpy

from collections.abc import Callable, Iterator
from contextlib import contextmanager
from dataclasses import dataclass, replace

log = logging.getLogger(__name__)

# Hard cap per READ, mirroring sanei_pieusb_get_scan_data() (pieusb_specific.c:2248).
# It is a maximum, not a fixed size; requesting more than 255 lines in one call
# produced corrupted/banded images on real hardware (poc:519-525).
MAX_LINES_PER_READ = 255

# Every line the scanner returns in INDEX colour format is prefixed by a two-byte
# channel tag (poc:820).
TAG_TO_CHANNEL = {b'RR': 0, b'GG': 1, b'BB': 2, b'II': 3}
CHANNEL_NAMES = ('R', 'G', 'B', 'I')
# Channel index -> the suffix its gain_*/exp_time_*/offset_* options carry.
CHANNEL_SUFFIX = ('r', 'g', 'b', 'i')

# Used for the auto-exposure metering pass only if the device's own reported
# preview resolution is unusable. Matches the 'resolution' option default.
PREVIEW_RESOLUTION_FALLBACK = 300

# START SCAN is retried while the device reports warming up, as in
# pieusb.c:1088-1093. 30 attempts at 5s is the PoC's proven budget (poc:719).
START_SCAN_ATTEMPTS = 30
START_SCAN_RETRY_S = 5

# How long close() waits for a cancelled worker to notice. Cancellation lands at
# the next chunk boundary, so the bound is one outstanding command -- a little
# over transport.COMMAND_TIMEOUT_S.
CLOSE_WAIT_S = 90

@dataclass(frozen=True)
class _ShadingReference:
    """A shading reference read from the device, ready to correct with.

    Cached on the Scanner because that is the lifetime it belongs to: it
    describes the CCD's per-column response under the current calibration, which
    outlives a single scan but not the device session. The same choice the C
    backend makes -- shading_ref/shading_mean live on Pieusb_Scanner, the open
    device handle, and shading_data_present starts false at sane_open
    (pieusb_specific.h:292-294, pieusb.c:418).

    `pixels_per_line` is the CCD-native width the reference was read at, which
    the image's own width is mapped onto through that pass's CCD mask. It is kept
    so a reference cannot be applied to a pass the device sized differently.
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
        # rather than a bool because it is the one piece of state written from
        # the caller's thread and read from the worker's.
        self.cancel_requested = threading.Event()
        self.shading_params: list[dict] | None = None
        self.closed: bool = False
        # The most recent shading reference any pass on this Scanner acquired, or
        # None if none has yet. What 'reuse_calibration' reuses.
        self._shading: _ShadingReference | None = None
        self.info = info
        self.params = generate_options(info.inquiry)
        self.dev.open()

    def __enter__(self) -> "Scanner":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def _option(self, name: str) -> Parameter | None:
        """The option called `name`, or None if it is ordinary object state.

        Reads `params` out of __dict__ directly: __setattr__ runs for every
        assignment in __init__, including `self.params` itself, so going through
        normal attribute lookup here would recurse into __getattr__ before the
        table exists.
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

    def __getattr__(self, name):
        """Read an option's value. Only called when normal lookup already failed.

        Returns the value, not the Parameter, so `scanner.resolution` round-trips
        through `scanner.resolution = ...`; `scanner.params` remains the table
        for introspection. Unknown names must raise AttributeError, not KeyError,
        or every hasattr()/getattr(default) probe against a Scanner breaks.
        """
        par = self._option(name)
        if par is None:
            raise AttributeError(
                f"{type(self).__name__!r} object has no attribute or option {name!r}"
            )
        return par.value

    def _why_not_ready(self) -> CheckCondition | None:
        """None if the device is ready, else the NOT READY sense explaining why.

        Anything other than a NOT READY sense is a real error and propagates --
        swallowing it here would hide bugs behind a bland 'not ready'.
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

    def _start_scan(self) -> None:
        self.dev.command(SCSI_SCAN, cdb_length=1)

    def _get_scanned_lines_cmd(self, lines, size) -> bytes:
        return self.dev.command(SCSI_READ, in_size=size, cdb_length=lines)

    def _iter_scanned_lines(self, total_lines, bytes_per_line, label="lines") -> Iterator[tuple[bytes, int]]:
        """Read total_lines in <=MAX_LINES_PER_READ chunks, yielding (bytes, n_lines).

        A generator rather than one buffer so the pixel read can decode, report
        and discard each chunk as it arrives -- a full-resolution 16-bit RGBI
        scan is gigabytes, and holding the raw bytes alongside the decoded image
        would double that.
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

    def _get_scanned_lines(self, total_lines, bytes_per_line, label="lines") -> bytes:
        collected = bytearray()
        for chunk, _ in self._iter_scanned_lines(total_lines, bytes_per_line, label):
            collected.extend(chunk)
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
            "available_lines": struct.unpack_from("<H", raw, 14)[0],
        }

    def _get_ccd_mask(self, mask_size):
        return self.dev.command(SCSI_COPY, in_size=mask_size, cdb_length=mask_size)

    def _get_gain_offset(self):
        raw = self.dev.command(SCSI_READ_GAIN_OFFSET, in_size=103, cdb_length=103)
        # Saturation levels are the average R/G/B the firmware reached while
        # optimising its own exposure times during warm-up, targeting >=90% of
        # full scale for R and B and >=80% for G (pieusb_scancmd.h:188-192).
        # They are the reference auto-exposure meters against, and they are only
        # refreshed on warm-up -- not after a scan, and not after gain changes.
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
            # This command is supposed to raise key=0x0b, code=0x00, qualifier=0x06
            if e.sense_key != 0x0b or e.sense_code != 0 or e.sense_qualifier != 0x06:
                raise

    def cancel(self) -> None:
        """Ask a running scan to stop; returns immediately, no-op if idle.

        Does not touch the device -- pyusb is not thread-safe and the worker owns
        it -- so cancellation takes effect at the next chunk boundary (<=255
        lines), where the worker issues STOP SCAN itself and fires on_complete
        with result.cancelled set. Pair with wait() to block until that happens.
        """
        if self.scan_in_progress:
            self.cancel_requested.set()

    def wait(self, timeout: float | None = None) -> bool:
        """Block until the running scan finishes, is cancelled or fails.

        Returns True if the worker is done (or none was running), False if it is
        still going when `timeout` expires. Never raises the scan's own failure:
        that arrives as `result.error` through on_complete, so a caller who only
        uses wait() learns nothing about the outcome by design.
        """
        thread = self.scan_thread
        if thread is None:
            return True
        if thread is threading.current_thread():
            # join() would deadlock. Reachable only from a callback, which by
            # definition runs on the worker -- and is a bug worth naming.
            raise PieusbError(
                "wait() called from the scan worker thread; a callback cannot wait on its own scan"
            )
        thread.join(timeout)
        return not thread.is_alive()

    def close(self, timeout: float | None = CLOSE_WAIT_S) -> None:
        """Cancel any running scan, wait for the worker, then release the interface.

        Idempotent. The wait is the point: releasing the USB interface while the
        worker still owns the device would pull it out from under a running scan.
        If the worker does not stop within `timeout` the interface is released
        anyway -- hanging forever on close() is the worse failure -- and that is
        logged as an error rather than passed over.
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
        """Emit a pass's update relabelled as the metering pass.

        The metering pass runs the same sequence as the real one, so its updates
        would otherwise arrive labelled SCANNING and sweep 0->100% before the
        real scan started again from 0. Line counts are kept: the pass is a real
        scan and a progress bar can follow it.
        """
        self._emit(replace(update, phase=ScanPhase.METERING))

    def _result(self, rgb=None, ir=None, *, started: float, width: int = 0, height: int = 0,
                shading_corrected: bool = False, cancelled: bool = False,
                error: Exception | None = None) -> ScanResult:
        """Build a ScanResult, filling in the settings the scan actually ran with."""
        return ScanResult(
            rgb=rgb,
            ir=ir,
            width=width,
            height=height,
            mode=self.params['mode'].value,
            color_depth=self.params['color_depth'].value,
            resolution=self.params['resolution'].value,
            shading_corrected=shading_corrected,
            cancelled=cancelled,
            error=error,
            duration_s=time.monotonic() - started,
        )

    def _scan_worker(self) -> None:
        """Thread entry point: run the scan and deliver exactly one ScanResult.

        Nothing may escape this method. Once the worker has started there is no
        caller left on the stack, so every failure is caught here, STOP SCAN is
        attempted, and it reaches the caller as result.error -- the threading
        contract in DESIGN.md.
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

        Writes Parameter.value directly rather than going through __setattr__:
        that path refuses any assignment while a scan is in progress, and this
        only ever runs on the worker with the scan already marked in progress.
        Type and range checks are therefore skipped -- callers pass constants and
        device-reported values, not user input.
        """
        saved = {name: self.params[name].value for name in values}
        for name, value in values.items():
            self.params[name].value = value
        try:
            yield
        finally:
            for name, value in saved.items():
                self.params[name].value = value

    def _meter_from_preview(self, started: float) -> ScanResult | None:
        """Run a preview pass and rewrite gain_*/exp_time_* from what it measured.

        Ports the SANE flow: a preview scan (pieusb.c:1284-1288 ->
        sanei_pieusb_analyze_preview) followed by
        sanei_pieusb_set_gain_offset(..., "from preview")
        (pieusb_specific.c:1912). The preview pass is _scan_pass() itself, only at
        the device's own preview resolution and with the quality options that SANE
        ignores during preview (pieusb_specific.c:1521-1546) turned off.

        Returns None once the settings have been updated, or the preview's own
        ScanResult if it was cancelled -- that result is what the caller should
        deliver, since there is no point starting the real scan afterwards.
        Failures propagate as exceptions, exactly as the real pass's do.
        """
        # Read before the preview: the levels are fixed at warm-up, and this also
        # fails early if the device will not answer, before spending a scan.
        saturation_levels = self._get_gain_offset()["saturation_level"]
        log.debug(f"[autoexp] saturation levels {saturation_levels}")

        # SANE trusts previewScanResolution unconditionally (pieusb_specific.c:386,
        # 1840). A zero would reach SET MODE and be rejected there, well after the
        # metering pass looked like it was working, so it is checked here instead.
        preview_resolution = self.info.inquiry.preview_scan_resolution
        if not self.params['resolution'].opt.validate(preview_resolution):
            fallback = min(self.params['resolution'].value, PREVIEW_RESOLUTION_FALLBACK)
            log.warning(f"[autoexp] the scanner reports an unusable preview resolution "
                        f"({preview_resolution}); metering at {fallback} dpi instead")
            preview_resolution = fallback
        log.debug(f"[autoexp] metering pass at {preview_resolution} dpi")
        with self._overridden_options(
            resolution=preview_resolution,
            sharpen=False,
            fast_infrared=False,
            advance=False,
        ):
            preview = self._scan_pass(started, self._emit_metering)

        if preview.cancelled:
            return preview
        if preview.rgb is None:
            raise PieusbError("metering pass returned no image data")

        # Which colour channels the pass actually measured, mirroring the switch
        # on mode.passes (pieusb_specific.c:1916-1958). Infrared is excluded even
        # in rgbi: updateGain2 is only ever called for indices 0-2. Gray is the
        # green filter alone, so its single plane meters channel 1.
        channels = (1,) if self.params['mode'].value == 'gray' else (0, 1, 2)

        # gain_increase() takes one entry per channel it should consider, so a
        # single-filter pass passes a 1-tuple of each.
        bounds = tuple(
            percentile_bounds(preview.rgb[:, :, plane])[1]
            for plane in range(len(channels))
        )
        levels = tuple(saturation_levels[c] for c in channels)
        log.debug(f"[autoexp] preview 99% bounds {bounds} for channels {channels}")

        dg = gain_increase(bounds, levels)
        log.info(f"[autoexp] applying a uniform gain increase of {dg:.3f}")
        if dg == 1.0:
            return None

        # exp_time_*, the absolute integration time -- never exp_rel_*, which
        # auto-exposure's saturation reference assumes is left at 100%.
        for c in channels:
            gain_name = f'gain_{CHANNEL_SUFFIX[c]}'
            exp_name = f'exp_time_{CHANNEL_SUFFIX[c]}'
            new_gain, new_exp = update_gain(
                self.params[gain_name].value, self.params[exp_name].value, dg
            )
            self._set_metered(gain_name, new_gain)
            self._set_metered(exp_name, new_exp)
        return None

    def _set_metered(self, name: str, value: int) -> None:
        """Store a metered option value, warning if it left the device's range.

        updateGain2 does not clamp, and neither do we -- clamping silently would
        break the invariant that gain and exposure together deliver exactly dg.
        But an out-of-range value is worth saying out loud, because the next
        set_options() will send it to the scanner regardless.
        """
        par = self.params[name]
        if not par.opt.validate(value):
            log.warning(f"[autoexp] metered {name}={value} is outside the range the "
                        f"scanner reports; sending it anyway")
        par.value = value

    def _run_scan(self, started: float) -> ScanResult:
        """Orchestrate the passes one scan() needs. Raises; _scan_worker catches.

        Auto-exposure costs an extra pass, so a scan is one or two runs of
        _scan_pass(); which phases each of them reports is decided here, by the
        emit callback handed to it.
        """
        if self.params['auto_exp'].value:
            cancelled = self._meter_from_preview(started)
            if cancelled is not None:
                return cancelled
            # The metering pass just calibrated, moments ago, on this medium and
            # this lamp -- as fresh a reference as the real pass could acquire for
            # itself. So reuse it and pay for one calibration rather than two. The
            # scanner still gets the last word: if it refuses the skip, the real
            # pass acquires anyway.
            with self._overridden_options(reuse_calibration=True):
                return self._scan_pass(started, self._emit)

        return self._scan_pass(started, self._emit)

    def _scan_pass(self, started: float, emit: Callable[[UpdateData], None]) -> ScanResult:
        """One pass over the medium, from configuration to a finished ScanResult.

        Shared by the real scan and the auto-exposure metering pass: both need the
        identical device sequence, and differ only in the options in force (see
        _meter_from_preview) and in how their progress is labelled -- hence `emit`
        rather than self._emit, which is what keeps the two passes distinguishable
        to a caller without this method knowing which one it is running.

        Whether the pass acquires its own shading reference or reuses the one
        cached on the Scanner is decided here, from 'reuse_calibration' and what
        the cache holds -- but the scanner gets the last word (see the
        must_calibrate branch below). Either way the image is corrected from
        self._shading, so the choice costs a calibration pass, never a correction.

        Raises; _scan_worker turns that into result.error. Follows sane_start()
        (pieusb.c:865-1140) and the PoC (poc:645-893), which is the sequence
        actually verified against hardware.
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

        # Whether this pass acquires a shading reference of its own. Skipping is
        # what 'reuse_calibration' asks for, but it can only be honoured with one
        # already cached to correct from -- shading correction is applied on the
        # host, so a granted skip and an empty cache would mean raw pixels. Never
        # producing an uncorrected image outranks honouring the option.
        acquire = not self.params['reuse_calibration'].value or self._shading is None
        if acquire and self.params['reuse_calibration'].value:
            log.info("[scan] reuse_calibration is set but no shading reference has been "
                     "acquired on this Scanner yet; calibrating this pass. Keep the "
                     "Scanner open across scans for later ones to reuse it")

        self.wait_ready()
        # The quality bit only asks. The scanner answers below.
        set_options(self.dev, self.params, skip_shading_analysis=not acquire)
        self.wait_ready()

        log.debug("[scan] starting scan...")
        for attempt in range(START_SCAN_ATTEMPTS):
            try:
                self._start_scan()
                break
            except CheckCondition as e:
                if e.warming_up:
                    emit(UpdateData(phase=ScanPhase.WARMING_UP))
                    log.debug(f"[scan]   still warming up, waiting {START_SCAN_RETRY_S}s "
                              f"(attempt {attempt + 1}/{START_SCAN_ATTEMPTS})...")
                    time.sleep(START_SCAN_RETRY_S)
                    continue
                if e.must_calibrate:
                    # NOT an error: "calibration disable not granted". The scanner
                    # refuses the skip and will calibrate regardless, so a fresh
                    # reference is waiting to be read whatever we asked for. This is
                    # the mechanism that keeps a reused reference from going stale:
                    # the device says when its own drift checks want a new one. The
                    # SANE backend reads the sense identically (pieusb.c:1091-1092).
                    log.info("[scan]   the scanner refused to skip calibration; "
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
        # The last cancellation point before the pixel read; the calibration
        # reads below are short enough not to be worth interrupting mid-way.
        if self.cancel_requested.is_set():
            log.debug("[scan] cancelled before the calibration read; stopping")
            self.stop_scan()
            return self._result(started=started, cancelled=True)

        shading_raw = None
        if acquire:
            emit(UpdateData(phase=ScanPhase.CALIBRATING))

            # Shading data is always 16-bit, with the same two-byte channel tag per
            # line as the image data. The C backend reads 4 * entry[0].nLines
            # (pieusb_specific.c:2078); summing nLines over all entries agrees with
            # that as long as the entries share a line count, and does not assume
            # four of them.
            total_shading_lines = sum(e["n_lines"] for e in self.shading_params)
            shading_bytes_per_line = 2 + shading_ppl * 2
            log.debug(f"[scan] reading shading reference ({total_shading_lines} lines, "
                      f"{total_shading_lines * shading_bytes_per_line} bytes)...")
            shading_raw = self._get_scanned_lines(
                total_shading_lines, shading_bytes_per_line, label="shading"
            )
            self.wait_ready()
        else:
            # Nothing to read and nothing to drain: the scanner granted the skip,
            # so no CALIBRATING is reported and this pass saves the calibration it
            # would have cost. The cached reference corrects the image instead.
            log.debug("[scan] calibration skipped; correcting from the reference "
                      "cached on this Scanner")

        # Read unconditionally, as sane_start() does (pieusb.c:1149) -- it sits
        # outside the calibration branch there. Only shading correction consumes
        # it, so with no reference read it goes unused rather than unread.
        log.debug("[scan] reading CCD mask...")
        ccd_mask = self._get_ccd_mask(shading_ppl)

        # The device's own authoritative geometry, rather than one computed from
        # the frame coordinates and hoped for.
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
        emit(UpdateData(phase=ScanPhase.SCANNING, scanned_lines=0, total_lines=total_lines))

        # Planes arrive as sequential blocks -- all R lines, then all G, and so
        # on -- but the 255-line read cap does not align to those boundaries, so
        # a single read can straddle a channel transition. Rows are therefore
        # placed by their own tag, not by arrival order or position.
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
            emit(UpdateData(
                phase=ScanPhase.SCANNING, scanned_lines=done, total_lines=total_lines
            ))

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

        # Build from what actually arrived, using the smallest common row count,
        # rather than guessing padding for a channel that came up short.
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

        # A pass that acquired one replaces the cache; a pass that skipped leaves
        # the reference the previous one left there, which is the point of the
        # cache. Either way the correction below reads it from the same place.
        if shading_raw is not None:
            shading_ref, shading_mean = calculate_shading(shading_raw, shading_ppl)
            if shading_ref is None:
                # Keep the older reference rather than dropping to raw pixels over
                # one bad read; it describes the same CCD under a calibration the
                # device was, until this pass, content with.
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

        # (planes, h, w) -> (h, w, planes) for the colour channels; infrared is a
        # single plane and stays two-dimensional.
        rgb = numpy.ascontiguousarray(numpy.moveaxis(planes[:3], 0, -1))
        ir = numpy.ascontiguousarray(planes[3]) if n_planes > 3 else None

        return self._result(
            rgb, ir,
            started=started,
            width=width,
            height=usable_height,
            shading_corrected=shading_corrected,
        )

    def scan(self, on_update: Callable[[UpdateData], None], on_complete: Callable[[ScanResult], None]) -> None:
        if self.closed:
            raise PieusbError('Scanner is closed')

        # This will raise id there is a problem with the parameters
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
