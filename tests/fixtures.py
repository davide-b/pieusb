'''INQUIRY fixtures for the offline tests.

`PROSCAN_10T` is parsed by the library's own `parse_inquiry()` from the raw
184-byte reply in `tests/reference/proscan10t_sane.txt`, so the fixture tracks
the parser. Bytes 128-183 are zero-filled where SANE's dump clipped; nothing
`generate_options()` reads lives above offset 104, so only `signature` and the
tail of `timestamp` are lost.

`PROSCAN_4000` is real too, parsed the same way from
`tests/reference/proscan4000_inquiry.txt` -- all 184 bytes, nothing clipped.
'''

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from pieusb.inquiry import parse_inquiry
from pieusb.types import (
    ColorFormat,
    Filter,
    ImageFormat,
    InquiryResponse,
    OptionalDevices,
    ScanCapabilities,
)
from support import load_inquiry_hex, load_sane_reference

# 4 header bytes plus the 180 the device reports as additional_length.
INQUIRY_LENGTH = 184


def _proscan_10t():
    '''Parse the reply out of the SANE reference, zero-padding the clip.'''
    for record in load_sane_reference():
        if record.opcode == 0x12 and record.cdb_len == INQUIRY_LENGTH:
            return parse_inquiry(record.data.ljust(INQUIRY_LENGTH, b'\x00'))
    raise LookupError('no full INQUIRY reply in the SANE reference')


PROSCAN_10T = _proscan_10t()

PROSCAN_4000 = parse_inquiry(load_inquiry_hex('proscan4000_inquiry.txt'))
