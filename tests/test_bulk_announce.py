#!/usr/bin/env python3
'''A READ is announced to the bridge in pieces where the model needs it, without hardware.

    python tests/test_bulk_announce.py
'''

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

from pieusb import transport
from pieusb.inquiry import capabilities_for
from pieusb.transport import STATUS_OK, STATUS_READ


def read(announce_max, size, bulk_in=bytes):
    '''Run a READ of `size` bytes; the announced sizes and the data come back.'''
    dev = transport.UASDevice.__new__(transport.UASDevice)
    announced = []
    dev.bulk_announce_max = announce_max
    dev.ieee_command = lambda *a: None
    dev._ctrl_out_byte = lambda *a: None
    statuses = iter([STATUS_READ, STATUS_OK])
    dev._ctrl_in_byte = lambda *a: next(statuses)
    dev._bulk_size = announced.append
    dev._bulk_in = bulk_in
    _, data = dev.scsi_transaction(0x08, 255, b'', size)
    return announced, data


class BulkAnnounce(unittest.TestCase):

    def test_a_large_read_is_announced_in_pieces(self):
        announced, data = read(65520, 310380)
        self.assertEqual(announced, [65520] * 4 + [310380 - 4 * 65520])
        self.assertEqual(len(data), 310380)

    def test_a_model_without_a_limit_announces_the_whole_read(self):
        announced, data = read(None, 310380)
        self.assertEqual(announced, [310380])
        self.assertEqual(len(data), 310380)

    def test_a_read_within_the_limit_is_announced_once(self):
        self.assertEqual(read(65520, 65520)[0], [65520])

    def test_a_piece_that_comes_back_short_ends_the_read(self):
        # The device stops sending: the READ ends with what arrived, it does not announce again.
        announced, data = read(65520, 310380, bulk_in=lambda size: bytes(100))
        self.assertEqual(announced, [65520])
        self.assertEqual(len(data), 100)

    def test_an_empty_piece_ends_the_read(self):
        announced, data = read(65520, 310380, bulk_in=lambda size: b'')
        self.assertEqual(announced, [65520])
        self.assertEqual(data, b'')

    def test_only_the_proscan_3600_limits_the_announce(self):
        self.assertEqual(capabilities_for(0x0027).bulk_announce_max, 65520)
        self.assertIsNone(capabilities_for(0x002C).bulk_announce_max)


if __name__ == '__main__':
    unittest.main(verbosity=2)
