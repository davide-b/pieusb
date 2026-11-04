#!/usr/bin/env python3
'''Device identification and enumeration, without hardware.

    python tests/test_identity.py
    python -m unittest discover tests        # this and test_wire_format.py

`parse_inquiry()` is a pure function; `get_devices()` becomes one once
`find_device` and `UASDevice` are replaced.
'''

import sys
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))

from fixtures import PROSCAN_10T, PROSCAN_4000
from support import load_sane_reference

from pieusb import _device, inquiry
from pieusb.exceptions import TransportError
from pieusb.inquiry import (
    DEVICE_CAPABILITIES,
    KNOWN_PIDS,
    MODEL_MOTORS,
    MODEL_NAMES,
    PIE_VENDOR_ID,
    capabilities_for,
    parse_inquiry,
)
from pieusb.types import Capabilities, OptionalDevices

PROSCAN_4000_PID = 0x0143


def inquiry_bytes():
    '''The 184-byte ProScan 10T reply, zero-padded past SANE's 128-byte clip.'''
    for record in load_sane_reference():
        if record.opcode == 0x12 and record.cdb_len == 184:
            return bytearray(record.data.ljust(184, b'\x00'))
    raise LookupError('no full INQUIRY reply in the reference')


class ModelIdentification(unittest.TestCase):

    def test_proscan_4000_product_id_is_probed(self):
        '''Without 0x0143, get_devices() cannot see a ProScan 4000 at all.'''
        self.assertIn(PROSCAN_4000_PID, KNOWN_PIDS)

    def test_known_pids_still_cover_the_verified_models(self):
        '''The ProScan 10T (0x0145) and the RPS 10M (0x0144).'''
        self.assertIn(0x0145, KNOWN_PIDS)
        self.assertIn(0x0144, KNOWN_PIDS)

    def test_10t_is_still_named_from_the_real_reply(self):
        response = parse_inquiry(bytes(inquiry_bytes()))
        self.assertEqual(response.model, 0x47)
        self.assertEqual(response.model_str, 'ProScan 10T')

    def test_proscan_4000_inquiry(self):
        """The scanner's own 184-byte reply, parsed."""
        p = PROSCAN_4000
        self.assertEqual((p.model, p.model_str), (0x2C, 'ProScan 4000'))
        self.assertEqual((p.vendor, p.product, p.revision), ('PIE', 'MF Scanner', '1.70'))
        self.assertEqual((p.max_resolution_x, p.max_resolution_y), (3600, 3600))
        self.assertEqual((p.max_scan_w, p.max_scan_h), (5172, 3444))
        self.assertEqual((p.minimum_exposure, p.maximum_exposure), (100, 2500))
        self.assertEqual(p.firmware_version, '1.98')

    def test_the_two_models_are_single_and_multi_frame(self):
        """`product` is the clearest signal of which carrier a model takes."""
        self.assertEqual(PROSCAN_10T.product, 'SF Scanner')
        self.assertEqual(PROSCAN_4000.product, 'MF Scanner')

    def test_a_film_transport_is_not_advertised_as_multipageload(self):
        """The bit that looks like it should detect a transport does not.

        The ProScan 4000 drives one and reports ADF alone; deriving the
        capability from `optional_devices` would report False.
        """
        self.assertEqual(PROSCAN_4000.optional_devices, (OptionalDevices.DEV_ADF,))
        self.assertNotIn(OptionalDevices.DEV_MPCL, PROSCAN_4000.optional_devices)
        self.assertTrue(capabilities_for(PROSCAN_4000.model).film_transport)

    def test_transparency_corners_are_in_sane_field_order(self):
        """x0, y0, x1, y1 at offsets 108, 110, 112, 114.

        SANE reports the 10T's window as offset 14900,9650 with width -0.3075 in
        and length 0.4222 in at 10000 dpi, which is x1-x0 and y1-y0 in that order.
        """
        p = PROSCAN_10T
        self.assertEqual((p.x0, p.y0), (14900, 9650))
        self.assertEqual(p.x1 - p.x0, -3075)
        self.assertEqual(p.y1 - p.y0, 4222)

    def test_unknown_model_reports_its_number(self):
        '''An unrecognised scanner reports its model number through model_str,
        which is what reaches the user via DeviceInfo.model.'''
        unlisted = 0x00FF
        self.assertNotIn(unlisted, MODEL_NAMES)

        raw = inquiry_bytes()
        raw[116:118] = unlisted.to_bytes(2, 'little')
        response = parse_inquiry(bytes(raw))
        self.assertEqual(response.model, unlisted)
        self.assertIn('0x00ff', response.model_str)
        # Everything else still decodes: an unknown model is not an unknown protocol.
        self.assertEqual(response.max_resolution_x, 10000)
        self.assertEqual(response.vendor, 'PIE')


class DeviceCapabilities(unittest.TestCase):

    def test_the_proscan_10t_has_no_capabilities_beyond_the_defaults(self):
        '''The ProScan 10T code path must stay exactly as it is.'''
        self.assertEqual(capabilities_for(0x0047), Capabilities())

    def test_rps_10m_capabilities(self):
        '''Transport and focus, checked by the hardware suite; its offset step
        moves the image towards -x.'''
        cap = capabilities_for(0x0048)
        self.assertTrue(cap.film_transport)
        self.assertTrue(cap.focus)
        self.assertLess(cap.offset_mm_per_step, 0)

    def test_an_unlisted_model_gets_the_conservative_defaults(self):
        self.assertEqual(capabilities_for(0x00FF), Capabilities())
        self.assertEqual(Capabilities(), Capabilities(film_transport=False, focus=False,
                                                      offset_mm_per_step=None))

    def test_proscan_4000_capabilities(self):
        cap = capabilities_for(0x002C)
        self.assertTrue(cap.film_transport)
        self.assertTrue(cap.focus)
        self.assertGreater(cap.offset_mm_per_step, 0)

    def test_only_the_expected_model_declares_focus(self):
        '''No other model may reach the focus code path by accident.'''
        focus = {m for m, c in DEVICE_CAPABILITIES.items() if c.focus}
        self.assertEqual(focus, {0x002C, 0x0048})

    def test_no_model_is_enabled_for_a_motor_it_does_not_have(self):
        '''DEVICE_CAPABILITIES may lag MODEL_MOTORS, never lead it.'''
        for model, motors in MODEL_MOTORS.items():
            cap = capabilities_for(model)
            enabled = {m for m in ('film_transport', 'focus') if getattr(cap, m)}
            self.assertLessEqual(enabled, motors, hex(model))
        self.assertEqual(MODEL_MOTORS[0x0047], frozenset(), 'the ProScan 10T has no motor')

    def test_slide_transport_mirrors_the_capability_table(self):
        '''`InquiryResponse.slide_transport` is `Capabilities.film_transport`.'''
        raw = inquiry_bytes()
        driven = set()
        for model in sorted(set(MODEL_NAMES) | {0x00FF}):
            raw[116:118] = model.to_bytes(2, 'little')
            reported = parse_inquiry(bytes(raw)).slide_transport
            self.assertEqual(reported, capabilities_for(model).film_transport,
                             hex(model))
            if reported:
                driven.add(model)

        self.assertEqual(driven, {0x002C, 0x002F, 0x003A, 0x0048})
        self.assertNotIn(0x0047, driven, 'the ProScan 10T must never be driven')


class Enumeration(unittest.TestCase):
    '''get_devices() with find_device and UASDevice replaced.'''

    def _run(self, attached, inquiries):
        '''attached: pids present. inquiries: pid -> InquiryResponse or Exception.'''
        def fake_find(vid, pid):
            self.assertEqual(vid, PIE_VENDOR_ID)
            return mock.Mock(name='usbdev:%04x' % pid, pid=pid) if pid in attached else None

        def fake_uas(dev):
            ctx = mock.MagicMock()
            ctx.__enter__.return_value = mock.Mock(pid=dev.pid)
            ctx.__exit__.return_value = False
            return ctx

        def fake_inquiry(d):
            result = inquiries[d.pid]
            if isinstance(result, Exception):
                raise result
            return result

        with mock.patch.object(_device, 'find_device', fake_find), \
             mock.patch.object(_device, 'UASDevice', fake_uas), \
             mock.patch.object(_device, '_get_inquiry', fake_inquiry):
            return _device.get_devices()

    def test_a_working_scanner_is_returned(self):
        found = self._run({0x0145}, {0x0145: PROSCAN_10T})
        self.assertEqual([d.model for d in found], ['ProScan 10T'])
        self.assertEqual([d.vendor for d in found], ['PIE'])

    def test_capabilities_come_from_the_reported_model_number(self):
        found = self._run({0x0145}, {0x0145: PROSCAN_10T})
        self.assertEqual(found[0].capabilities, capabilities_for(PROSCAN_10T.model))
        self.assertEqual(found[0].capabilities, Capabilities())

    def test_an_unidentifiable_scanner_does_not_hide_a_working_one(self):
        '''A scanner that fails INQUIRY must not hide one attached beside it.'''
        found = self._run(
            {PROSCAN_4000_PID, 0x0145},
            {PROSCAN_4000_PID: TransportError('no reply to a 5-byte INQUIRY'),
             0x0145: PROSCAN_10T},
        )
        self.assertEqual([d.model for d in found], ['ProScan 10T'])

    def test_a_lone_failing_scanner_still_raises(self):
        '''With nothing to return, the error is more useful than an empty list.'''
        error = TransportError('no reply to a 5-byte INQUIRY')
        with self.assertRaises(TransportError) as caught:
            self._run({PROSCAN_4000_PID}, {PROSCAN_4000_PID: error})
        self.assertIs(caught.exception, error)

    def test_no_scanners_attached_is_not_an_error(self):
        self.assertEqual(self._run(set(), {}), [])


if __name__ == '__main__':
    unittest.main(verbosity=2)
