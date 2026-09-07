#!/usr/bin/env python3
"""Write a usbmon capture of one INQUIRY transaction, for testing pieusb.lua.

The capture corpus contains no INQUIRY reply -- both vendor applications ask the
device only once, before capturing started -- so the dissector's largest decoder
has nothing real to run against. This replays the 184 bytes recorded in
tests/reference/proscan4000_inquiry.txt as the transaction that would have
carried them: wakeup, IEEE1284 SCSI, the six CDB bytes, status READ, the size
announce, the bulk IN reply, status OK.

    python3 wireshark/make_inquiry_capture.py [out.pcapng]
    tshark -X lua_script:wireshark/pieusb.lua -r out.pcapng
"""
import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
LINKTYPE_USB_LINUX_MMAPPED = 220
HDR = struct.Struct("<QBBBBHccqiiII8sIIII")

BUS, DEV = 1, 12
WAKEUP = bytes([0xFF, 0xAA, 0x55, 0x00, 0xFF, 0x87, 0x78])
IEEE_SCSI = 0xE0
P_SIZE, P_STATUS, P_CMD, P_CTRL, P_DATA = 0x0082, 0x0084, 0x0085, 0x0087, 0x0088


class Writer:
    def __init__(self, path):
        self.f = open(path, "wb")
        self.t = 0
        self.urb = 0xFFFF000000000000
        self._shb()

    def _block(self, btype, body):
        pad = (-len(body)) % 4
        total = 12 + len(body) + pad
        self.f.write(struct.pack("<II", btype, total) + body + b"\x00" * pad
                     + struct.pack("<I", total))

    def _shb(self):
        self._block(0x0A0D0D0A, struct.pack("<IHHq", 0x1A2B3C4D, 1, 0, -1))
        self._block(0x00000001, struct.pack("<HHI", LINKTYPE_USB_LINUX_MMAPPED, 0, 0))

    def _packet(self, payload):
        self.t += 100  # microseconds
        self._block(0x00000006, struct.pack(
            "<IIIII", 0, self.t >> 32, self.t & 0xFFFFFFFF, len(payload), len(payload))
            + payload)

    def _urb(self, kind, xfer, ep, setup, data, urb_len, urb_id):
        setup_flag = b"\x00" if setup else b"-"
        data_flag = b"\x00" if data else (b"<" if ep & 0x80 else b">")
        hdr = HDR.pack(urb_id, ord(kind), xfer, ep, DEV, BUS, setup_flag, data_flag,
                       0, self.t, 0 if kind == "C" else -115, urb_len, len(data),
                       setup or bytes(8), 0, 0, 0x200 if ep & 0x80 else 0, 0)
        self._packet(hdr + data)

    def transfer(self, xfer, ep, setup=None, out=b"", reply=b"", urb_len=None):
        """One URB as usbmon records it: a submit frame and a completion frame."""
        self.urb += 1
        n = urb_len if urb_len is not None else max(len(out), len(reply))
        self._urb("S", xfer, ep, setup, out, n, self.urb)
        self._urb("C", xfer, ep, None, reply, n, self.urb)

    def reg_out(self, request, value, payload):
        self.transfer(2, 0x00, struct.pack("<BBHHH", 0x40, request, value, 0, len(payload)),
                      out=payload)

    def reg_in(self, value, reply):
        self.transfer(2, 0x80, struct.pack("<BBHHH", 0xC0, 0x0C, value, 0, len(reply)),
                      reply=reply)

    def close(self):
        self.f.close()


def ieee_command(w, command):
    for b in WAKEUP:
        w.reg_out(0x0C, P_DATA, bytes([b]))
    w.reg_out(0x0C, P_DATA, bytes([command]))
    w.reg_out(0x0C, P_CTRL, bytes([0x05]))
    w.reg_out(0x0C, P_CTRL, bytes([0x04]))
    w.reg_out(0x0C, P_DATA, b"\xff")


def inquiry_reply():
    text = (ROOT / "tests/reference/proscan4000_inquiry.txt").read_text()
    hexs = " ".join(l for l in text.splitlines() if not l.startswith("#"))
    return bytes.fromhex(hexs)


def main():
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("proscan4000_inquiry.pcapng")
    reply = inquiry_reply()
    w = Writer(out)

    ieee_command(w, IEEE_SCSI)
    cdb = bytes([0x12, 0x00, 0x00, len(reply) >> 8, len(reply) & 0xFF, 0x00])
    for b in cdb:
        w.reg_out(0x0C, P_CMD, bytes([b]))
    w.reg_in(P_STATUS, b"\x01")                      # READ: reply follows
    w.reg_out(0x04, P_SIZE, bytes(4) + struct.pack("<I", len(reply)))
    w.transfer(3, 0x81, reply=reply)
    w.reg_in(P_STATUS, b"\x00")                      # OK
    w.close()
    print(f"{out}: INQUIRY of {len(reply)} bytes")


if __name__ == "__main__":
    main()
