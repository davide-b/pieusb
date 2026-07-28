from pieusb.transport import UASDevice

from pieusb.option import generate_options, set_options
from pieusb.types import (
    DeviceInfo,
    UpdateData,
    ScanResult,
    ScanPhase
)
from pieusb.exceptions import (
    CheckCondition,
    DeviceNotReady,
    WarmingUp
)
from pieusb.transport import (
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
    PieusbError,
    WarmingUp,
    DeviceNotReady,
    ScanInProgress
)

import logging
import time
import threading
import struct

from collections.abc import Callable

log = logging.getLogger(__name__)

class Scanner:
    def __init__(self, info: DeviceInfo) -> None:
        self.dev = UASDevice(info.dev)
        self.scan_thread: threading.Thread | None = None
        self.on_update: Callable[[UpdateData], None] | None = None
        self.on_complete: Callable[[ScanResult], None] | None = None
        self.scan_in_progress: bool = False
        self.shading_params: list[dict] | None = None
        self.params = generate_options(info.inquiry)
        self.dev.open()

    def __enter__(self) -> "Scanner":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.dev.close()

    def __setitem__(self, key, value) -> None:
        par = self.params[key]
        if type(value) is not par.opt.type:
            raise TypeError(f"Invalid type provided to option '{par.opt.name}' (got {type(value)}, expected {par.opt.type})")
        if not par.opt.validate(value):
            raise ValueError(f"Invalid value provided to option '{par.opt.name}'")
        par.value = value

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

    def _get_scanned_lines(self, total_lines, bytes_per_line, label="lines") -> bytes:
        MAX_LINES_PER_READ = 255
        collected = bytearray()
        remaining = total_lines
        done = 0
        while remaining > 0:
            lines_this_read = min(MAX_LINES_PER_READ, remaining)
            n_bytes = lines_this_read * bytes_per_line
            chunk = self._get_scanned_lines_cmd(lines=lines_this_read, size=n_bytes)
            collected.extend(chunk)
            done += lines_this_read
            remaining -= lines_this_read
            log.debug(f"[scan]   {label}: {done}/{total_lines} lines read (remaining {remaining})")
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
        exposure_rgb = struct.unpack_from("<3H", raw, 60)
        offset_rgb = tuple(raw[66:69])
        gain_rgb = tuple(raw[72:75])
        light = raw[75]
        exposure_i = struct.unpack_from("<H", raw, 98)[0]
        offset_i = raw[100]
        gain_i = raw[102]
        return {
            "exposure_time": exposure_rgb + (exposure_i,),
            "offset": offset_rgb + (offset_i,),
            "gain": gain_rgb + (gain_i,),
            "light": light,
        }

    def stop_scan(self) -> None:
        self.dev.command(SCSI_SCAN, cdb_length=0)

    def _scan_worker(self) -> None:
        self.on_update(UpdateData(phase=ScanPhase.CONFIGURING))

        self.wait_ready()
        
        set_options(self.dev, self.params)

        self.wait_ready()

        self.shading_params = self._get_shading_parms()

        log.debug("[scan] starting scan...")
        for attempt in range(30):
            try:
                self._start_scan()
                break
            except CheckCondition as e:
                if e.warming_up:
                    log.debug("[scan]   still warming up, waiting 5s...")
                    time.sleep(5)
                    continue
                if e.must_calibrate:
                    # NOT an error: the scanner is telling us it will calibrate and
                    # expects us to read the shading reference data below. The SANE
                    # backend treats MUST_CALIBRATE identically -- it proceeds into
                    # the calibration phase (pieusb.c:1091). Only reachable if
                    # set_mode(..., skip_shading_analysis=True) was used; with the
                    # default (skip=False) START SCAN normally returns OK.
                    log.debug("[scan]   scanner requires calibration, proceeding to "
                            "read shading reference data...")
                    break
                raise
        self.wait_ready()

        # TODO self.on_complete(...)
        self.scan_in_progress = False

    def scan(self, on_update: Callable[[UpdateData], None], on_complete: Callable[[ScanResult], None]) -> None:
        self.params.validate()

        # TODO this could probably be checked with a hardware call
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
        self.scan_thread = threading.Thread(target=self._scan_worker)
        self.scan_in_progress = True
        self.scan_thread.start()
