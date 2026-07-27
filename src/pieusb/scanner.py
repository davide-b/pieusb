from pieusb.transport import UASDevice
import usb.core

from pieusb.option import generate_options, set_options
from pieusb.types import DeviceInfo
from pieusb.exceptions import (
    CheckCondition,
    DeviceNotReady,
    WarmingUp
)
from pieusb.transport import (
    SCSI_READ,
    SCSI_SCAN,
    SCSI_TEST_UNIT_READY
)

import logging
import numpy
import time

log = logging.getLogger(__name__)

class Scanner:
    def __init__(self, info: DeviceInfo) -> None:
        self.dev = UASDevice(info.dev)
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

    def stop_scan(self) -> None:
        self.dev.command(SCSI_SCAN, cdb_length=0)

    def scan(self) -> numpy.ndarray:
        self.wait_ready()

        set_options(self.dev, self.params)

        self.wait_ready()

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

        