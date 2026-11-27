#!/usr/bin/env python3
'''The ProScan 3600 shading uploads, without hardware.

    python tests/test_shading_upload.py
'''

import struct
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

from pieusb.inquiry import capabilities_for
from pieusb.scanner import Scanner
from pieusb.transport import SCSI_WRITE


class ShadingUpload(unittest.TestCase):

    def test_only_the_3600_uploads(self):
        self.assertTrue(capabilities_for(0x0027).shading_upload)
        self.assertFalse(capabilities_for(0x002C).shading_upload)

    def test_the_four_tables_are_written_in_order(self):
        sent = []
        scanner = Scanner.__new__(Scanner)
        scanner.dev = SimpleNamespace(
            command=lambda op, out_data=b'', cdb_length=None, **k: sent.append((op, out_data, cdb_length)))
        scanner.wait_ready = lambda *a, **k: None
        # Ten lines per channel at raw level 3000, tagged R, G, B as the device sends them.
        line = lambda tag: tag * 2 + struct.pack('<5172H', *[3000] * 5172)
        scanner._upload_shading_tables(b''.join(line(t) for t in (b'R', b'G', b'B') * 10), 5172)

        self.assertEqual([s[0] for s in sent], [SCSI_WRITE] * 4)
        self.assertEqual([len(s[1]) for s in sent], [31042, 8198, 8198, 8198])
        self.assertEqual([s[1][:2].hex() for s in sent], ['1600', '1000', '1000', '1000'])
        # The shading table: length field, then three tagged lines of
        # SLOPE * (LEVEL - raw) = 1.4 * (4000 - 3000) = 1400.
        table = sent[0][1]
        self.assertEqual(table[2:4].hex(), '3e79')
        self.assertEqual([table[4 + k * 10346:6 + k * 10346].hex() for k in range(3)],
                         ['0200', '0400', '0800'])
        for k in range(3):
            self.assertEqual(set(struct.unpack('<5172H', table[6 + k * 10346:4 + (k + 1) * 10346])), {1400})
        # The curve tables differ only in the filter field: R, G, B.
        self.assertEqual([s[1][4:6].hex() for s in sent[1:]], ['0200', '0400', '0800'])
        self.assertEqual([s[1][2:4].hex() for s in sent[1:]], ['0220'] * 3)
        # The curve is linear, the same for R, G, B.
        curves = [struct.unpack('<4096H', s[1][6:]) for s in sent[1:]]
        self.assertEqual(curves[0], tuple(range(4096)))
        self.assertTrue(curves[0] == curves[1] == curves[2])
        self.assertTrue(all(s[2] == len(s[1]) for s in sent))

    def test_a_pass_without_shading_reads_sends_the_curve_tables_only(self):
        sent = []
        scanner = Scanner.__new__(Scanner)
        scanner.dev = SimpleNamespace(
            command=lambda op, out_data=b'', cdb_length=None, **k: sent.append(out_data))
        scanner.wait_ready = lambda *a, **k: None
        scanner._upload_shading_tables(None, 5172)
        self.assertEqual([s[:2].hex() for s in sent], ['1000'] * 3)


if __name__ == '__main__':
    unittest.main(verbosity=2)
