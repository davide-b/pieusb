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

WAKEUP_SEQUENCE = bytes([0xFF, 0xAA, 0x55, 0x00, 0xFF, 0x87, 0x78])

CTRL_TIMEOUT_MS = 5_000
BULK_TIMEOUT_MS = 30_000
BULK_CHUNK = 0x4000
COMMAND_TIMEOUT_S = 60

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