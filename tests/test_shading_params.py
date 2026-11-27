#!/usr/bin/env python3
'''Shading entries, without hardware.

    python tests/test_shading_params.py

The reply is the calibration info a ProScan 3600 returned in
captures/ProScan3600 (four entries of ten lines, 5172 pixels per line).
'''

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

from pieusb.inquiry import parse_filters
from pieusb.scanner import Scanner

REPLY = bytes.fromhex('95001c0004060400000c0c0a3414080c0c0a3414'
                      '100c0c0a3414200c0c0a3414')
PROSCAN_3600_FILTERS = 0x8E       # R, G, B; no infrared bit
INFRARED_FILTERS = 0x9E           # the same plus the infrared bit


def shading_params(filter_byte):
    scanner = Scanner.__new__(Scanner)
    scanner.dev = SimpleNamespace(command=lambda *a, **k: REPLY)
    scanner.info = SimpleNamespace(
        inquiry=SimpleNamespace(filters=parse_filters(filter_byte)))
    return scanner._get_shading_parms()


class ShadingParams(unittest.TestCase):

    def test_a_scanner_without_infrared_keeps_the_three_colour_entries(self):
        entries = shading_params(PROSCAN_3600_FILTERS)
        self.assertEqual(len(entries), 3)
        self.assertEqual(sum(e['n_lines'] for e in entries), 30)

    def test_an_infrared_scanner_keeps_every_entry(self):
        entries = shading_params(INFRARED_FILTERS)
        self.assertEqual(len(entries), 4)
        self.assertEqual(sum(e['n_lines'] for e in entries), 40)

    def test_the_entry_layout_is_parsed(self):
        self.assertEqual(shading_params(INFRARED_FILTERS)[0],
                         {'type': 0x00, 'n_lines': 10, 'pixels_per_line': 5172})


if __name__ == '__main__':
    unittest.main(verbosity=2)
