'''
USB-attached SCSI device
'''

import usb.core
import usb.util

import logging
import time
import struct

log = logging.getLogger(__name__)

REQUEST_TYPE_OUT = 0x40  # vendor, host-to-device
REQUEST_TYPE_IN = 0xC0   # vendor, device-to-host
REQUEST_REGISTER = 0x0C
REQUEST_BUFFER = 0x04
ANYINDEX = 0x0000

C1284_NSTROBE = 0x01
C1284_NINIT = 0x04
IEEE1284_RESET = 0x30
IEEE1284_SCSI = 0xE0

PORT_SCSI_SIZE = 0x0082
PORT_SCSI_STATUS = 0x0084
PORT_SCSI_CMD = 0x0085
PORT_PAR_CTRL = 0x0087
PORT_PAR_DATA = 0x0088

SCSI_COMMAND_LEN = 6  # standard 6-byte CDB
SCSI_REQUEST_SENSE = 0x03

WAKEUP_SEQUENCE = bytes([0xFF, 0xAA, 0x55, 0x00, 0xFF, 0x87, 0x78])

STATUS_OK = 0x00
STATUS_READ = 0x01
STATUS_CHECK = 0x02
STATUS_BUSY = 0x03
STATUS_AGAIN = 0x08
STATUS_FAIL = 0x88
STATUS_ERROR = 0xFF

CTRL_TIMEOUT_MS = 5_000
BULK_TIMEOUT_MS = 30_000
BULK_CHUNK = 0x4000
COMMAND_TIMEOUT_S = 60

class CheckCondition(Exception):
    def __init__(self, sense_key, sense_code, sense_qualifier):
        self.sense_key = sense_key
        self.sense_code = sense_code
        self.sense_qualifier = sense_qualifier
        super().__init__(
            f"CHECK CONDITION key=0x{sense_key:02x} code=0x{sense_code:02x} "
            f"qualifier=0x{sense_qualifier:02x}"
        )

    @property
    def warming_up(self) -> bool:
        return self.sense_key == 0x02 and self.sense_code == 4 and self.sense_qualifier == 1

    @property
    def must_calibrate(self) -> bool:
        # "Calibration disable not granted" -- returned by START SCAN when
        # skipShadingAnalysis was requested but the scanner insists on
        # calibrating. In the SANE backend (sanei_pieusb_decode_sense) this maps
        # to PIEUSB_STATUS_MUST_CALIBRATE, which is NOT an error: it is the
        # signal to enter the calibration phase and read shading reference data
        # (pieusb.c:1091-1121). We treat it the same way.
        return self.sense_key == 0x06 and self.sense_code == 0x82 and self.sense_qualifier == 0

class UASDevice:
    def __init__(self, dev: usb.core.Device) -> None:
        self.dev = dev
        self.intf_number: int | None = None
        self.ep_in: usb.core.Endpoint | None = None

    def __enter__(self) -> "UASDevice":
        self.open()
        return self

    def __exit__(self) -> None:
        self.close()

    def _ensure_open(self) -> tuple[int, usb.core.Endpoint]:
        if self.intf_number is None or self.ep_in is None:
            return self.open()

        return self.intf_number, self.ep_in

    def _ctrl_out_byte(self, port, b):
        _, _ = self._ensure_open()
        
        self.dev.ctrl_transfer(
            REQUEST_TYPE_OUT, REQUEST_REGISTER, port, ANYINDEX,
            bytes([b & 0xFF]), timeout=CTRL_TIMEOUT_MS,
        )


    def _ctrl_in_byte(self, port=PORT_SCSI_STATUS):
        _, _ = self._ensure_open()

        for attempt in range(5):
            data = self.dev.ctrl_transfer(
                REQUEST_TYPE_IN, REQUEST_REGISTER, port, ANYINDEX, 1,
                timeout=CTRL_TIMEOUT_MS,
            )
            if len(data) >= 1:
                return data[0]
            time.sleep(0.05 * (attempt + 1))
        raise IOError(f"ctrl_in_byte(port=0x{port:04x}) returned zero-length response after retries")


    def _bulk_size(self, size):
        _, _ = self._ensure_open()

        payload = bytes(4) + struct.pack("<I", size)
        self.dev.ctrl_transfer(
            REQUEST_TYPE_OUT, REQUEST_BUFFER, PORT_SCSI_SIZE, ANYINDEX,
            payload, timeout=CTRL_TIMEOUT_MS,
        )


    def _bulk_in(self, size):
        _, ep_in = self._ensure_open()

        out = bytearray()
        remaining = size
        while remaining > 0:
            part = min(BULK_CHUNK, remaining)
            data = ep_in.read(part, timeout=BULK_TIMEOUT_MS)
            if not data:
                break
            out.extend(data)
            remaining -= len(data)
        return bytes(out)


    def ieee_command(self, command_byte):
        for b in WAKEUP_SEQUENCE:
            self._ctrl_out_byte(PORT_PAR_DATA, b)
        self._ctrl_out_byte(PORT_PAR_DATA, command_byte)
        time.sleep(0.003)
        self._ctrl_out_byte(PORT_PAR_CTRL, C1284_NINIT | C1284_NSTROBE)
        self._ctrl_out_byte(PORT_PAR_CTRL, C1284_NINIT)
        self._ctrl_out_byte(PORT_PAR_DATA, 0xFF)


    def reset(self):
        self.ieee_command(IEEE1284_RESET)

    def open(self) -> tuple[int, usb.core.Endpoint]:
        log.debug("[open] set_configuration()...")
        self.dev.set_configuration()
        log.debug("[open] set_configuration() ok")
    
        log.debug("[open] get_active_configuration()...")
        cfg = self.dev.get_active_configuration()
        self.intf_number = cfg[(0, 0)].bInterfaceNumber
        log.debug(f"[open] interface number = {self.intf_number}")
    
        self.ep_in = usb.util.find_descriptor(
            self.intf_number,
            custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress)
            == usb.util.ENDPOINT_IN,
        )
        log.debug(f"[open] bulk IN endpoint = {self.ep_in}")
    
        log.debug(f"[open] claim_interface({self.intf_number})...")
        usb.util.claim_interface(self.dev, self.intf_number)
        log.debug("[open] claim_interface() ok")

        self.reset()

        return self.intf_number, self.ep_in

    def close(self) -> None:
        log.debug("[close] releasing interface...")
        try:
            usb.util.release_interface(self.dev, self.intf_number)
        except usb.core.USBError:
            pass
        usb.util.dispose_resources(self.dev)
        self.ep_in = None
        self.intf_number = None
        log.debug("[close] done")

    def scsi_transaction(self, opcode, cdb_length, out_data, in_size):
        self.ieee_command(IEEE1284_SCSI)

        cdb = bytearray(SCSI_COMMAND_LEN)
        cdb[0] = opcode & 0xFF
        cdb[3] = (cdb_length >> 8) & 0xFF
        cdb[4] = cdb_length & 0xFF
        for b in cdb:
            self._ctrl_out_byte(PORT_SCSI_CMD, b)

        status = self._ctrl_in_byte(PORT_SCSI_STATUS)
        result = b""

        if status == STATUS_OK and len(out_data) > 0:
            for b in out_data:
                self._ctrl_out_byte(PORT_SCSI_CMD, b)
            status = self._ctrl_in_byte(PORT_SCSI_STATUS)
        elif status == STATUS_READ:
            self._bulk_size(in_size)
            result = self._bulk_in(in_size)
            status = self._ctrl_in_byte(PORT_SCSI_STATUS)

        return status, result

    def command(self, opcode, out_data=b"", in_size=0, cdb_length=None):
        """
        Full command with retry/busy/check-condition handling, per
        sanei_pieusb_command() in pieusb_usb.c.
        """
        if cdb_length is None:
            cdb_length = len(out_data) if out_data else in_size

        status = STATUS_AGAIN
        result = b""
        deadline = time.time() + COMMAND_TIMEOUT_S
        data_phase_retries = 0
        MAX_DATA_PHASE_RETRIES = 5

        while time.time() < deadline:
            log.debug(f"    [cmd 0x{opcode:02x}] status=0x{status:02x}")
            if status == STATUS_AGAIN:
                status, result = self.scsi_transaction(opcode, cdb_length, out_data, in_size)
                log.debug(f"    [cmd 0x{opcode:02x}]   -> new status=0x{status:02x}, {len(result)} bytes")
                continue
            if status == STATUS_OK:
                data_expected = (not out_data) and in_size > 0
                if data_expected and len(result) != in_size:
                    if data_phase_retries < MAX_DATA_PHASE_RETRIES:
                        data_phase_retries += 1
                        status = STATUS_AGAIN
                        continue
                    raise IOError(
                        f"cmd 0x{opcode:02x}: OK but got {len(result)} bytes, expected {in_size}"
                    )
                return result
            if status == STATUS_READ:
                raise IOError("unexpected second READ status (protocol violation)")
            if status == STATUS_CHECK:
                sense = self.get_sense()
                raise CheckCondition(sense["sense_key"], sense["sense_code"], sense["sense_qualifier"])
            if status == STATUS_BUSY:
                status = self._ctrl_in_byte(PORT_SCSI_STATUS)
                if status == STATUS_AGAIN:
                    time.sleep(1.0)
                continue
            if status in (STATUS_FAIL, STATUS_ERROR):
                self.reset()
                raise IOError(f"USB status 0x{status:02x}")
            raise IOError(f"unhandled USB status 0x{status:02x}")

        raise TimeoutError(f"command 0x{opcode:02x} timed out after {COMMAND_TIMEOUT_S}s")

    def get_sense(self):
        raw = self.command(SCSI_REQUEST_SENSE, in_size=14)
        return {"sense_key": raw[2], "sense_code": raw[12], "sense_qualifier": raw[13]}