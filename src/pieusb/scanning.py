import logging
import time

import usb.core

from pieusb.transport import UASDevice, CheckCondition
from pieusb.transport import (
    SCSI_TEST_UNIT_READY,
    SCSI_REQUEST_SENSE,
    SCSI_READ,
    SCSI_WRITE,
    SCSI_PARAM,
    SCSI_INQUIRY,
    SCSI_MODE_SELECT,
    SCSI_COPY,
    SCSI_MODE_SENSE,
    SCSI_SCAN,
    SCSI_READ_GAIN_OFFSET,
    SCSI_WRITE_GAIN_OFFSET,
    SCSI_READ_STATE,
    SCSI_SCAN_FRAME,
    SCSI_EXPOSURE,
    SCSI_HIGHLIGHT_SHADOW,
    SCSI_CALIBRATION_INFO,
)

log = logging.getLogger(__name__)

class ScannerDevice:
    def __init__(self, dev: usb.core.Device):
        self.uas = UASDevice(dev)
        self.uas.open()

    def ready(self) -> bool:
        try:
            self.uas.command(SCSI_TEST_UNIT_READY, cdb_length=0)
            return True
        except CheckCondition as e:
            if e.warming_up:
                return False
            raise

    def wait_ready(self, timeout_s: int =180) -> None:
        log.debug("  [wait_ready] polling TEST UNIT READY...")
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if self.ready():
                log.debug("  [wait_ready] ready")
                return
            time.sleep(1.0)
        raise TimeoutError("scanner did not become ready in time")

    def start_scan(self) -> None:
        self.uas.command(SCSI_SCAN, cdb_length=1)
    
    def stop_scan(self) -> None:
        self.uas.command(SCSI_SCAN, cdb_length=0)

    def get_ccd_mask(self, mask_size) -> bytes:
        return self.uas.command(SCSI_COPY, in_size=mask_size, cdb_length=mask_size)

    def get_scanned_lines(self, lines, size) -> bytes:
        # NOTE: cdb_length is the *line count*, in_size is the byte count --
        # this divergence is real, taken directly from
        # sanei_pieusb_cmd_get_scanned_lines() in pieusb_scancmd.c.
        return self.uas.command(SCSI_READ, in_size=size, cdb_length=lines)