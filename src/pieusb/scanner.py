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
from dataclasses import dataclass

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

# The options auto_exp overwrites with the device's own calibration -- the whole
# settings struct at once, as the C adopts it (pieusb_specific.c:1988, 2046).
CALIBRATION_OPTIONS = tuple(
    f'{field}_{suffix}'
    for field in ('gain', 'exp_time', 'offset')
    for suffix in CHANNEL_SUFFIX
) + ('light',)

# START SCAN is retried while the device reports warming up (pieusb.c:1088-1093).
START_SCAN_ATTEMPTS = 30
START_SCAN_RETRY_S = 5

# How long close() waits for a cancelled worker to notice. Cancellation lands at
# the next chunk boundary, so the bound is one outstanding command.
CLOSE_WAIT_S = 90

@dataclass(frozen=True)
class _ShadingReference:
    """A shading reference read from the device, ready to correct with.

    Describes the CCD's per-column response under the current calibration, so it
    outlives a single scan but not the device session -- hence cached on the
    Scanner, as the C caches it on the open handle (pieusb_specific.h:292-294).

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

    def _adopt_device_calibration(self) -> None:
        """Overwrite the calibration options with the scanner's own, if it has any.

        SCAN_CALIBRATION_AUTO (pieusb_specific.c:1988), the C's default calibration
        mode (:733): read GET GAIN OFFSET and send the settings struct straight
        back. The firmware optimises gain and exposure per channel while warming
        up, measured on this lamp at its current temperature.

        Not the "calibration from preview" path (pieusb_specific.c:1912), which
        divides by settings.saturationLevel. A PIE ProScan 10T reports 0-0-0 there
        warm or cold, and a zero makes updateGain2 produce gain 0 and exposure 0,
        so there is nothing for a preview pass to meter against.

        Leaves every option untouched on a cold scanner, which reports zeros --
        exposure (0, 0, 0, 4100), gain (0, 0, 0, 15), light 0 is the observed cold
        state. The firmware fills them in during its first scan.
        """
        settings = self._get_gain_offset()
        exposure = settings["exposure_time"]
        gain = settings["gain"]
        offset = settings["offset"]
        light = settings["light"]

        # Both are zero on a cold scanner: the exposure times it optimises, and
        # the lamp level it settles at 4 once warm (pieusb_scancmd.h:208-213).
        if not all(exposure[c] > 0 for c in range(3)) or light <= 0:
            log.warning(
                f"[autoexp] the scanner reports no calibration of its own yet "
                f"(exposure {exposure}, gain {gain}, light {light}); it fills these in "
                f"during its first scan. Scanning with the options as set instead -- "
                f"a second scan in this session will pick up the real values"
            )
            return

        log.info(f"[autoexp] adopting the scanner's calibration: gain {gain}, "
                 f"exposure {exposure}, offset {offset}, light {light}")

        self._set_from_device('light', light)
        for c, suffix in enumerate(CHANNEL_SUFFIX):
            # Infrared is optimised with the others but often reset afterwards
            # (pieusb_scancmd.h:194-199), so it alone may come back unset.
            if exposure[c] > 0:
                self._set_from_device(f'exp_time_{suffix}', exposure[c])
            self._set_from_device(f'gain_{suffix}', gain[c])
            self._set_from_device(f'offset_{suffix}', offset[c])

    def _set_from_device(self, name: str, value: int) -> None:
        """Store a device-reported option value, warning if it left its own range.

        Not clamped -- the C sends these back unexamined -- but a scanner
        contradicting its own advertised range is worth a warning, because
        set_options() will send it regardless.
        """
        par = self.params[name]
        if not par.opt.validate(value):
            log.warning(f"[autoexp] device-reported {name}={value} is outside the range "
                        f"the scanner itself reports; sending it anyway")
        par.value = value

    def _run_scan(self, started: float) -> ScanResult:
        """Run the one pass a scan needs. Raises; _scan_worker catches.

        With auto_exp the scanner's own calibration is adopted first, inside a
        restore so those values belong to this scan alone. Without it the options
        are sent exactly as set.
        """
        if self.params['auto_exp'].value:
            with self._saved_options(*CALIBRATION_OPTIONS):
                self._adopt_device_calibration()
                return self._scan_pass(started)

        return self._scan_pass(started)

    def _scan_pass(self, started: float) -> ScanResult:
        """The one pass over the medium, from configuration to a finished ScanResult.

        Whether the pass acquires its own shading reference or reuses the cached
        one is decided here, from 'reuse_calibration' and what the cache holds,
        with the scanner getting the last word (the must_calibrate branch below).
        Either way the image is corrected from self._shading, so the choice costs a
        calibration pass, never a correction.

        Raises; _scan_worker turns that into result.error. Follows sane_start()
        (pieusb.c:865-1140).
        """
        self._emit(UpdateData(phase=ScanPhase.CONFIGURING))

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
        acquire = not self.params['reuse_calibration'].value or self._shading is None
        if acquire and self.params['reuse_calibration'].value:
            log.info("[scan] reuse_calibration is set but no shading reference has been "
                     "acquired on this Scanner yet; calibrating this pass. Keep the "
                     "Scanner open across scans for later ones to reuse it")

        self.wait_ready()
        # Logged rather than returned on the ScanResult because auto_exp restores
        # these afterwards, making this the only place the values a pass ran with
        # can be observed.
        log.info("[scan] sending exposure %s, gain %s, offset %s, light %d",
                 tuple(self.params[f'exp_time_{s}'].value for s in CHANNEL_SUFFIX),
                 tuple(self.params[f'gain_{s}'].value for s in CHANNEL_SUFFIX),
                 tuple(self.params[f'offset_{s}'].value for s in CHANNEL_SUFFIX),
                 self.params['light'].value)
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
                    # Progress against the retry budget, which is all we know: the
                    # scanner does not say how much warming up it has left.
                    self._emit(UpdateData(phase=ScanPhase.WARMING_UP,
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
            self._emit(UpdateData(phase=ScanPhase.CALIBRATING))

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
                report=lambda frac: self._emit(
                    UpdateData(phase=ScanPhase.CALIBRATING, progress=frac)
                ),
            )
            self.wait_ready()
        else:
            # The scanner granted the skip: nothing to read, no CALIBRATING phase,
            # and the cached reference corrects the image instead.
            log.debug("[scan] calibration skipped; correcting from the reference "
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
        self._emit(UpdateData(phase=ScanPhase.SCANNING, progress=0.0))

        # Planes arrive as sequential blocks -- all R lines, then all G -- but the
        # 255-line read cap does not align to those boundaries, so a single read can
        # straddle a channel transition. Rows are placed by their own tag.
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
            self._emit(UpdateData(phase=ScanPhase.SCANNING, progress=done / total_lines))

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
        self._emit(UpdateData(phase=ScanPhase.PROCESSING))

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
