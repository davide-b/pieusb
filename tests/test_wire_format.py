#!/usr/bin/env python3
'''What this library puts on the wire, pinned three ways.

    python tests/test_wire_format.py            # no dependencies beyond the venv
    python tests/test_wire_format.py -v

No hardware, no USB: `set_options()` and the option table are pure functions of
an INQUIRY reply, so they can be driven against a recording fake device.

Three kinds of test:

1. **Conformance against the ProScan 10T.** `tests/reference/proscan10t_sane.txt`
   is a SANE debug log recorded on that hardware; SANE is an independent
   implementation of the same protocol. Deliberate divergences are marked.

2. **Golden masters.** `tests/golden/*.txt` holds what `set_options()` emits for
   a set of configurations on the real 10T INQUIRY. They assert the bytes have
   not moved, not that they are right. `--regenerate-golden` moves one.

3. **Conformance against the ProScan 4000 captures.** Skipped if `captures/` is
   absent.
'''

import logging
import struct
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parent))

from fixtures import PROSCAN_10T, PROSCAN_4000
from support import (
    CheckCondition,
    PieusbError,
    TransportError,
    FakeDevice,
    FakeStrip,
    CAPTURE_DIR,
    GOLDEN_DIR,
    REFERENCE_DIR,
    describe,
    load_capture,
    load_sane_reference,
)

from pieusb import option, transport
from pieusb.exceptions import EndOfStrip
from pieusb.framing import film_base, find_bands
from pieusb.inquiry import capabilities_for
from pieusb.option import generate_options, set_options
from pieusb.postprocess import line_sharpness
from pieusb.scanner import (
    AUTOFOCUS_MIN_DEPTH,
    MAX_LINES_PER_READ,
    METERED_OPTIONS,
    Scanner,
    autofocus_pick,
    autofocus_positions,
    smooth_scores,
)
from pieusb.transport import UASDevice
from pieusb.types import DeviceInfo, EjectDirection, ScanResult
import numpy
import usb.core

REGENERATE = '--regenerate-golden' in sys.argv
if REGENERATE:
    sys.argv.remove('--regenerate-golden')

REFERENCE = load_sane_reference()


def send(inquiry, overrides=None, skip_shading_analysis=False, **kwargs):
    options = generate_options(inquiry)
    for name, value in (overrides or {}).items():
        options[name].value = value
    options.validate()
    device = FakeDevice(**kwargs)
    set_options(device, options, skip_shading_analysis=skip_shading_analysis)
    return device


def ref(opcode=None, sub=None, index=0):
    hits = [r for r in REFERENCE
            if (opcode is None or r.opcode == opcode)
            and (sub is None or r.sub == sub)]
    return hits[index]


# ---------------------------------------------------------------------------
# 1. Conformance against the ProScan 10T (SANE reference)
# ---------------------------------------------------------------------------

class ProScan10TReference(unittest.TestCase):
    '''Cross-checks against a second implementation on verified hardware.'''

    def test_inquiry_probe_lengths_match_ours(self):
        '''SANE probes 5 bytes then re-asks for raw[4]+4, as _get_inquiry() does.'''
        short, full = ref(0x12, index=0), ref(0x12, index=1)
        self.assertEqual(short.cdb_len, 5)
        self.assertEqual(len(short.data), 5)
        additional_length = short.data[4]
        self.assertEqual(full.cdb_len, additional_length + 4)

    def test_gain_offset_length_tried_first_is_the_one_the_10t_takes(self):
        '''The 10T accepts 29 bytes, so 29 must be tried first: the fallback must
        not cost it a rejected command per scan.'''
        self.assertEqual(ref(0xDC).cdb_len, 29)
        self.assertEqual(option.GAIN_OFFSET_LENGTHS[0], 29)

    def test_read_gain_offset_field_offsets(self):
        '''Our field offsets against SANE's, on the same real reply.

        SANE writes the block straight back unchanged in the first pair, so
        decoding its reply with our offsets must give the values it re-encoded.
        '''
        reply, wrote = ref(0xD7).data, ref(0xDC).out
        self.assertEqual(struct.unpack_from('<3H', reply, 60),
                         struct.unpack_from('<3H', wrote, 0), 'exposure r/g/b')
        self.assertEqual(tuple(reply[66:69]), tuple(wrote[6:9]), 'offset r/g/b')
        self.assertEqual(tuple(reply[72:75]), tuple(wrote[12:15]), 'gain r/g/b')
        self.assertEqual(reply[75], wrote[15], 'light')
        self.assertEqual(struct.unpack_from('<H', reply, 98)[0],
                         struct.unpack_from('<H', wrote, 18)[0], 'exposure i')
        self.assertEqual(reply[100], wrote[20], 'offset i')
        self.assertEqual(reply[102], wrote[22], 'gain i')

    def test_read_gain_offset_offsets_hold_after_calibration(self):
        '''The second pair, where SANE alters exposure and gain: the remaining
        fields still round-trip.'''
        reply, wrote = ref(0xD7, index=1).data, ref(0xDC, index=1).out
        self.assertEqual(tuple(reply[66:69]), tuple(wrote[6:9]), 'offset r/g/b')
        self.assertEqual(reply[75], wrote[15], 'light')
        self.assertEqual(struct.unpack_from('<H', reply, 98)[0],
                         struct.unpack_from('<H', wrote, 18)[0], 'exposure i')
        self.assertEqual(reply[100], wrote[20], 'offset i')
        self.assertEqual(reply[102], wrote[22], 'gain i')

    def test_shading_parms_layout(self):
        '''n_entries at byte 4, entry_size at 5, line count at +3, pixels at +4.

        Both derived quantities appear independently in the log: the shading lines
        SANE reads, and the length of its CCD-mask request.
        '''
        raw = ref(0x08).data
        self.assertEqual(ref(0x08).cdb_len, 32, 'SANE reads 32, as we do')
        n_entries, entry_size = raw[4], raw[5]
        entries = [{
            'n_lines': raw[8 + entry_size * k + 3],
            'pixels_per_line': struct.unpack_from('<H', raw, 8 + entry_size * k + 4)[0],
        } for k in range(n_entries)]

        self.assertEqual(n_entries, 4)
        self.assertEqual(entry_size, 6)

        # Total shading lines == the lines SANE actually reads before the CCD mask.
        ccd_mask_at = next(i for i, r in enumerate(REFERENCE) if r.opcode == 0x18)
        before_mask = [r.cdb_len for r in REFERENCE[:ccd_mask_at]
                       if r.opcode == 0x08 and r.cdb_len != 32]
        self.assertEqual(sum(e['n_lines'] for e in entries), sum(before_mask))

        # Entry 0's pixels-per-line == the CCD mask length we would ask for.
        self.assertEqual(entries[0]['pixels_per_line'], ref(0x18).cdb_len)
        self.assertNotEqual(entries[0]['pixels_per_line'], 5184,
                            'the 10T is 7185 wide; a hard-coded 4000 value would '
                            'break it')

    def test_must_calibrate_sense_is_recognised(self):
        '''must_calibrate matches the sense the 10T answers START SCAN with.

        `_scan_pass()` treats it as the signal to acquire a shading reference
        rather than as an error.
        '''
        sense = ref(0x03).data
        error = CheckCondition(sense[2] & 0x0F, sense[12], sense[13])
        self.assertTrue(error.must_calibrate, sense.hex(' '))
        self.assertFalse(error.warming_up)
        self.assertFalse(error.parameter_list_length_error)

    def test_read_chunking_stays_within_our_cap(self):
        '''SANE tops out at MAX_LINES_PER_READ exactly, not merely below it.'''
        line_reads = [r.cdb_len for r in REFERENCE if r.opcode == 0x08 and r.cdb_len != 32]
        self.assertEqual(max(line_reads), MAX_LINES_PER_READ)

    def test_line_accounting_matches_planes_times_height(self):
        '''Total lines read == n_planes * height, as `_scan_pass()` assumes.

        Height from GET PARAMETERS, four planes for one-pass RGBI, shading reads
        before the CCD mask excluded.
        '''
        params = ref(0x0F).data
        height = struct.unpack_from('<H', params, 2)[0]
        ccd_mask_at = next(i for i, r in enumerate(REFERENCE) if r.opcode == 0x18)
        image_lines = sum(r.cdb_len for r in REFERENCE[ccd_mask_at:]
                          if r.opcode == 0x08 and r.cdb_len != 32)
        self.assertEqual(image_lines, 4 * height)

    def test_scan_frame_encoding_reproduces_the_reference(self):
        '''Our SET SCAN FRAME builder, given SANE's corners, emits SANE's bytes.'''
        want = ref(sub=transport.SCSI_SCAN_FRAME).out
        index, x0, y0, x1, y1 = struct.unpack_from('<HHHHH', want, 4)
        device = send(PROSCAN_10T, {'tl_x': x0, 'tl_y': y0, 'br_x': x1, 'br_y': y1})
        got = device.one(transport.SCSI_WRITE, transport.SCSI_SCAN_FRAME)
        self.assertEqual(got['out'], want)
        self.assertEqual(got['cdb_length'], ref(sub=transport.SCSI_SCAN_FRAME).cdb_len)

    def test_mode_select_encoding_matches_except_line_threshold(self):
        '''Our MODE SELECT, configured as SANE was, differs in exactly one byte.

        Byte 13 is `lineThreshold`: SANE computes 0x7F from a percentage, we send
        a flat 0x80. Consulted only in the lineart and halftone modes, which this
        library refuses, so the difference is inert.
        '''
        want = ref(transport.SCSI_MODE_SELECT).out
        resolution = struct.unpack_from('<H', want, 2)[0]
        device = send(PROSCAN_10T,
                      {'resolution': resolution, 'mode': 'rgbi', 'color_depth': 8},
                      skip_shading_analysis=True)
        got = device.one(transport.SCSI_MODE_SELECT)['out']

        LINE_THRESHOLD = 13
        self.assertEqual(got[:LINE_THRESHOLD], want[:LINE_THRESHOLD])
        self.assertEqual(got[LINE_THRESHOLD + 1:], want[LINE_THRESHOLD + 1:])
        self.assertEqual(want[LINE_THRESHOLD], 0x7F)
        self.assertEqual(got[LINE_THRESHOLD], option.LINE_THRESHOLD)

    def test_exposure_and_highlight_shadow_match_the_reference(self):
        '''All six per-filter writes, byte for byte.'''
        device = send(PROSCAN_10T)
        for sub in (transport.SCSI_EXPOSURE, transport.SCSI_HIGHLIGHT_SHADOW):
            got = [c['out'] for c in device.of(transport.SCSI_WRITE, sub)]
            want = [r.out for r in REFERENCE if r.sub == sub]
            self.assertEqual(got, want, describe(transport.SCSI_WRITE, want[0]))


class TransactionRecovery(unittest.TestCase):
    """What happens when a control transfer is abandoned part-way."""

    class _Aborting(UASDevice):
        def __init__(self):
            self.resets = 0

        def scsi_transaction(self, *args, **kwargs):
            raise usb.core.USBError('Operation timed out', 110)

        def reset(self):
            self.resets += 1

    class _Configured(UASDevice):
        """A device the kernel already configured."""

        def __init__(self, configured):
            self.dev = self
            self.set_calls = 0
            self._cfg = SimpleNamespace(bConfigurationValue=1) if configured else None

        def get_active_configuration(self):
            if self._cfg is None:
                raise usb.core.USBError('Configuration not set')
            return self._cfg

        def set_configuration(self):
            self.set_calls += 1
            self._cfg = SimpleNamespace(bConfigurationValue=1)

    def test_an_aborted_transaction_resets_the_bridge(self):
        """Otherwise the device waits for the rest of a CDB until power-cycled."""
        dev = self._Aborting()
        with self.assertLogs('pieusb.transport', level='WARNING'):
            with self.assertRaises(TransportError):
                dev.command(transport.SCSI_INQUIRY, in_size=5, cdb_length=5)
        self.assertEqual(dev.resets, 1)

    def test_the_control_timeout_covers_the_longest_measured_nak(self):
        """A ProScan 4000 held the control pipe for 30.25 s on one READ STATE."""
        self.assertGreater(transport.CTRL_TIMEOUT_MS, 30_250)

    def test_an_already_configured_device_is_not_reconfigured(self):
        dev = self._Configured(configured=True)
        self.assertEqual(dev._configuration().bConfigurationValue, 1)
        self.assertEqual(dev.set_calls, 0)

    def test_an_unconfigured_device_is_configured_once(self):
        dev = self._Configured(configured=False)
        self.assertEqual(dev._configuration().bConfigurationValue, 1)
        self.assertEqual(dev.set_calls, 1)


class SenseDecoding(unittest.TestCase):
    '''get_sense() over the sense bytes the 10T really returned.'''

    class _Device(UASDevice):
        def __init__(self, reply):
            self._reply = reply

        def command(self, opcode, out_data=b'', in_size=0, cdb_length=None):
            assert opcode == transport.SCSI_REQUEST_SENSE
            return self._reply

    def test_sense_key_is_masked_to_its_low_nibble(self):
        '''Byte 2's top nibble is FILEMARK/EOM/ILI and must not reach sense_key.

        The reference sense has a clear top nibble, so setting one is the only way
        to show the mask working. Left unmasked, must_calibrate stops matching.
        '''
        raw = bytearray(ref(0x03).data)
        self.assertEqual(raw[2] & 0xF0, 0, 'the reference itself sets no flags')
        raw[2] |= 0x80                      # FILEMARK

        sense = self._Device(bytes(raw)).get_sense()
        self.assertEqual(sense['sense_key'], 0x06)
        self.assertTrue(CheckCondition(**{
            'sense_key': sense['sense_key'],
            'sense_code': sense['sense_code'],
            'sense_qualifier': sense['sense_qualifier'],
        }).must_calibrate)

    def test_the_reference_sense_still_decodes_unchanged(self):
        sense = self._Device(ref(0x03).data).get_sense()
        self.assertEqual(sense, {'sense_key': 0x06, 'sense_code': 0x82,
                                 'sense_qualifier': 0x00})


# READ STATE replies as captured on a ProScan 4000: a slide carrier with the focus
# driven to 80, and a filmstrip carrier sitting on frame 1 with the focus parked.
STATE_SLIDE_CARRIER = bytes.fromhex('4f 00 00 50 50 00 01 00'.replace(' ', ''))
STATE_FILMSTRIP = bytes.fromhex('51 00 01 00 1e 00 1d 00'.replace(' ', ''))


def gain_offset_reply(exposure=(10286, 7345, 4405), exposure_i=5874,
                      offset=(15, 12, 13), offset_i=12,
                      gain=(37, 44, 48), gain_i=57, light=7):
    """A GET GAIN OFFSET reply, laid out as `_get_gain_offset` reads it.

    Defaults are what a ProScan 4000 reports for itself in every capture.
    """
    raw = bytearray(103)
    struct.pack_into('<3H', raw, 60, *exposure)
    raw[66:69] = bytes(offset)
    raw[72:75] = bytes(gain)
    raw[75] = light
    struct.pack_into('<H', raw, 98, exposure_i)
    raw[100] = offset_i
    raw[102] = gain_i
    return bytes(raw)


def scanner(inquiry, model, state=None, overrides=None, replies=None, device=None):
    """A Scanner over a FakeDevice, with no USB and no __init__."""
    canned = dict(replies or {})
    if state is not None:
        canned[transport.SCSI_READ_STATE] = state
    s = Scanner.__new__(Scanner)
    s.dev = device or FakeDevice(replies=canned)
    s.scan_in_progress = False
    s._transport_initialized = False
    s._applied_factors = None
    s._shading = None
    s.info = DeviceInfo(dev=None, vendor=inquiry.vendor, model=inquiry.model_str,
                        inquiry=inquiry, capabilities=capabilities_for(model))
    s.params = generate_options(inquiry, s.info.capabilities)
    for name, value in (overrides or {}).items():
        setattr(s, name, value)
    return s


class Focus(unittest.TestCase):
    """SCSI_SLIDE's parameter byte, and the focus that rides on it."""

    def slide_payloads(self, s):
        return [c['out'] for c in s.dev.of(transport.SCSI_SLIDE)]

    def test_focus_option_exists_only_where_the_model_has_focus(self):
        self.assertNotIn('focus', [p.opt.name for p in
                                   generate_options(PROSCAN_10T,
                                                    capabilities_for(0x0047)).table])
        self.assertIn('focus', [p.opt.name for p in
                                generate_options(PROSCAN_4000,
                                                 capabilities_for(0x002C)).table])

    def test_a_model_without_focus_still_gets_the_byte_it_always_got(self):
        """SLIDE_INIT is `10 01 00 00` and SLIDE_NEXT `04 01 00 00` on a DigitDia."""
        s = scanner(PROSCAN_10T, 0x002F)          # DigitDia 4000: transport, no focus
        s._slide(transport.SLIDE_INIT, s._focus_for_scan())
        s._slide(transport.SLIDE_NEXT)
        self.assertEqual(self.slide_payloads(s),
                         [bytes.fromhex('10010000'), bytes.fromhex('04010000')])

    def test_the_focus_option_reaches_the_slide_byte(self):
        s = scanner(PROSCAN_4000, 0x002C, STATE_SLIDE_CARRIER, {'focus': 40})
        s._slide(transport.SLIDE_INIT, s._focus_for_scan())
        self.assertEqual(self.slide_payloads(s), [bytes.fromhex('10280000')])

    def test_transport_state_decodes_a_slide_carrier(self):
        state = scanner(PROSCAN_4000, 0x002C, STATE_SLIDE_CARRIER).transport_state()
        self.assertEqual((state.frame, state.focus, state.focus_max), (0, 80, 80))
        self.assertFalse(state.filmstrip_carrier)

    def test_transport_state_decodes_a_filmstrip_carrier(self):
        state = scanner(PROSCAN_4000, 0x002C, STATE_FILMSTRIP).transport_state()
        self.assertEqual((state.frame, state.focus, state.focus_max), (1, 0, 30))

    def test_transport_state_is_none_without_a_transport(self):
        s = scanner(PROSCAN_10T, 0x0047, STATE_SLIDE_CARRIER)
        self.assertIsNone(s.transport_state())
        self.assertIsNone(s.focus_range())
        self.assertEqual(s.dev.sent, [], 'and READ STATE is not even sent')

    def test_focus_range_follows_the_loaded_carrier(self):
        self.assertEqual(
            scanner(PROSCAN_4000, 0x002C, STATE_SLIDE_CARRIER).focus_range(), (1, 80))
        self.assertEqual(
            scanner(PROSCAN_4000, 0x002C, STATE_FILMSTRIP).focus_range(), (1, 30))

    def test_a_focus_beyond_the_carrier_is_clamped(self):
        """80 is valid on a slide carrier and out of range on a filmstrip one."""
        s = scanner(PROSCAN_4000, 0x002C, STATE_FILMSTRIP, {'focus': 80})
        with self.assertLogs('pieusb.scanner', level='WARNING'):
            self.assertEqual(s._focus_for_scan(), 30)

    def test_set_focus_rejects_a_value_the_carrier_cannot_reach(self):
        s = scanner(PROSCAN_4000, 0x002C, STATE_FILMSTRIP)
        with self.assertRaises(ValueError):
            s.set_focus(80)
        self.assertEqual(self.slide_payloads(s), [], 'and nothing was sent')

    def test_set_focus_is_refused_without_focus(self):
        s = scanner(PROSCAN_10T, 0x0047)
        with self.assertRaises(PieusbError):
            s.set_focus(40)


class Autofocus(unittest.TestCase):
    """The vendor's focus sweep, as captured on an RPS 10M (captures/rps10m)."""

    def mode_select(self, **kwargs):
        options = generate_options(PROSCAN_10T)
        device = FakeDevice()
        set_options(device, options, **kwargs)
        return device.one(transport.SCSI_MODE_SELECT)['out']

    def test_the_autofocus_pass_sets_mode_select_byte_10(self):
        self.assertEqual(self.mode_select(autofocus=True)[10], 0x08)

    def test_every_other_pass_leaves_byte_10_clear(self):
        self.assertEqual(self.mode_select()[10], 0x00)

    def test_one_line_per_focus_position(self):
        self.assertEqual(autofocus_positions(30, 1, 30), list(range(1, 31)))

    def test_a_line_count_off_the_range_is_spread_over_it(self):
        with self.assertLogs('pieusb.scanner', level='WARNING'):
            positions = autofocus_positions(3, 1, 30)
        self.assertEqual(positions, [1, 15, 30])

    @staticmethod
    def sweep(sharp_line, lines=30, width=400):
        """A sweep whose lines are all one texture, crisp at sharp_line and
        blurred more the further a line is from it. None: every line equally soft."""
        crisp = 50.0 + 150.0 * numpy.random.default_rng(1).random(width)

        def blurred(n):
            return numpy.convolve(crisp, numpy.ones(n) / n, mode='same') if n > 1 else crisp

        rows = [blurred(15 if sharp_line is None else 1 + 2 * abs(i - sharp_line))
                for i in range(lines)]
        return numpy.repeat(numpy.array(rows)[:, :, numpy.newaxis], 3,
                            axis=2).astype(numpy.uint8)

    def test_sharpness_ranks_the_crisp_line_first(self):
        self.assertEqual(int(numpy.argmax(line_sharpness(self.sweep(12)))), 12)

    def test_sharpness_ignores_level(self):
        """A brighter copy of a line is not a sharper one."""
        line = self.sweep(0, lines=1).astype(numpy.float32)
        scores = line_sharpness(numpy.concatenate([line, line * 0.5]))
        self.assertAlmostEqual(scores[0], scores[1], places=5)

    def test_sharpness_reads_red_alone(self):
        """Noise in a dim blue channel must not move the pick."""
        sweep = self.sweep(12)
        sweep[:, :, 2] = numpy.random.default_rng(0).integers(0, 16, sweep.shape[:2])
        self.assertEqual(int(numpy.argmax(line_sharpness(sweep))), 12)

    @staticmethod
    def real_sweep(name):
        """The red channel of a sweep recorded on hardware, as (lines, width)."""
        with numpy.load(REFERENCE_DIR / 'autofocus_sweeps_red.npz') as sweeps:
            return sweeps[name]

    def pick(self, name):
        """autofocus_pick() on a recorded sweep: (position, depth)."""
        scores = line_sharpness(self.real_sweep(name))
        best, depth, _ = autofocus_pick(scores, list(range(1, len(scores) + 1)))
        return best, depth

    # Each recorded sweep, with the positions a manual sweep of full scans found
    # sharpest or within about 1% of it.
    SHARP = {
        'rps10m_patch': range(13, 17),           # Laplacian variance, 2500 dpi
        'rps10m_suite': range(1, 14),            # best 9; 1-13 within 1.1%
        'proscan4000_patch': range(1, 5),
        'proscan4000_default_frame': range(1, 5),
        'proscan4000_suite1': range(1, 6),       # best 1; 5 within 0.1%
        'proscan4000_suite2': range(1, 6),       # best 1; 5 within 2%
        'proscan4000_suite3': range(1, 6),       # best 1; 5 within 1%
    }

    def test_recorded_sweeps_pick_a_manually_sharp_focus(self):
        for name, sharp in self.SHARP.items():
            with self.subTest(name):
                best, depth = self.pick(name)
                self.assertIn(best, sharp)
                self.assertGreaterEqual(depth, AUTOFOCUS_MIN_DEPTH)

    def test_a_noisy_first_line_does_not_win(self):
        """The RPS 10M suite sweep's best single line is its first."""
        scores = line_sharpness(self.real_sweep('rps10m_suite'))
        self.assertEqual(int(numpy.argmax(scores)), 0)
        self.assertNotEqual(self.pick('rps10m_suite')[0], 1)

    def test_rps_10m_default_frame_sweep_is_too_flat(self):
        """A slow rise to focus 30 from something other than the picture."""
        self.assertLess(self.pick('rps10m_default_frame')[1], AUTOFOCUS_MIN_DEPTH)

    def test_smoothing_narrows_its_window_at_the_ends(self):
        self.assertEqual(smooth_scores([3.0, 1.0, 2.0, 0.0, 4.0], 5).tolist(),
                         [2.0, 1.5, 2.0, 1.75, 2.0])

    def autofocused(self, sharp_line):
        s = scanner(PROSCAN_4000, 0x002C, STATE_FILMSTRIP, {'resolution': 600})
        s._shading = cached = object()
        before = {p.opt.name: p.value for p in s.params.table}
        seen = {}

        def fake_pass(started, emit, autofocus=False):
            seen.update({name: s.params[name].value for name in
                         ('resolution', 'mode', 'color_depth', 'reuse_calibration',
                          'tl_y', 'br_y', 'focus')}, autofocus=autofocus)
            s._shading = 'replaced by the pass'
            return ScanResult(rgb=self.sweep(sharp_line))

        s._scan_pass = fake_pass
        result = s.autofocus()
        return s, result, seen, (cached, before)

    def test_autofocus_adopts_the_sharpest_position(self):
        s, result, _, _ = self.autofocused(12)
        self.assertTrue(result.peaked)
        self.assertEqual(result.focus, 13)
        self.assertEqual(s.params['focus'].value, 13)
        self.assertEqual(result.positions, tuple(range(1, 31)))

    def test_a_sweep_without_a_peak_leaves_focus_alone(self):
        with self.assertLogs('pieusb.scanner', level='WARNING'):
            s, result, _, _ = self.autofocused(None)
        self.assertFalse(result.peaked)
        self.assertEqual(result.focus, s.params['focus'].value)
        self.assertFalse(s.params['focus'].assigned)

    def test_the_sweep_runs_as_the_vendor_runs_it(self):
        s, _, seen, _ = self.autofocused(0)
        br_y = s.params['br_y'].value
        self.assertEqual(seen, {
            'autofocus': True,
            'resolution': 3600,              # min(5000, the 4000's maximum)
            'mode': 'rgb',
            'color_depth': 8,
            'reuse_calibration': False,
            'tl_y': br_y // 2,               # lower half of the default frame
            'br_y': br_y,
            'focus': 1,
        })

    def test_autofocus_restores_everything_but_focus(self):
        s, _, _, (cached, before) = self.autofocused(0)
        after = {p.opt.name: p.value for p in s.params.table}
        self.assertEqual({k: v for k, v in after.items() if k != 'focus'},
                         {k: v for k, v in before.items() if k != 'focus'})
        self.assertIs(s._shading, cached, 'the cached shading reference survives')

    def test_autofocus_is_refused_without_focus(self):
        with self.assertRaises(PieusbError):
            scanner(PROSCAN_10T, 0x0047).autofocus()


class FilmTransport(unittest.TestCase):
    """SCSI_SLIDE's action and parameter bytes, against the captured vendor ones."""

    def setUp(self):
        # Transport calls log at INFO and WARNING; they would drown the run.
        self._log = logging.getLogger('pieusb.scanner')
        self._level = self._log.level
        self._log.setLevel(logging.ERROR)

    def tearDown(self):
        self._log.setLevel(self._level)

    def slide(self, s):
        return [c['out'] for c in s.dev.of(transport.SCSI_SLIDE)]

    def transport(self, frames=8, frame=1):
        s = scanner(PROSCAN_4000, 0x002C,
                    device=FakeStrip(frames, STATE_FILMSTRIP, frame=frame))
        s._transport_initialized = True          # skip the focus move under test
        return s

    def test_advance_matches_the_vendor_bytes(self):
        """`04 01 00 00` for one frame, `04 02 00 00` for a 1->3 jump."""
        s = self.transport()
        s.advance()
        s.advance(2)
        self.assertEqual(self.slide(s),
                         [bytes.fromhex('04010000'), bytes.fromhex('04020000')])

    def test_rewind_matches_the_vendor_bytes(self):
        """CyberView rewinds five frames to frame 0 with `05 05 00 01`."""
        s = self.transport()
        s.rewind(5)
        self.assertEqual(self.slide(s), [bytes.fromhex('05050001')])

    def test_eject_matches_the_vendor_bytes(self):
        s = self.transport()
        s.eject(EjectDirection.UP)
        s.eject(EjectDirection.DOWN)
        self.assertEqual(self.slide(s),
                         [bytes.fromhex('03000000'), bytes.fromhex('02000000')])

    def test_eject_defaults_to_up(self):
        s = self.transport()
        s.eject()
        self.assertEqual(self.slide(s), [bytes.fromhex('03000000')])

    def test_offset_converts_millimetres_to_240ths_of_an_inch(self):
        """Every offset the captures record, reproduced from its millimetre value."""
        for mm, steps in ((1, 0x09), (2, 0x12), (3.3, 0x1f), (3.5, 0x21), (5, 0x2f)):
            with self.subTest(mm=mm):
                s = self.transport()
                s.offset_frame(mm)
                self.assertEqual(self.slide(s), [bytes([0x00, steps, 0, 0])])

    def test_a_negative_offset_uses_the_other_action(self):
        s = self.transport()
        s.offset_frame(-2)
        self.assertEqual(self.slide(s), [bytes.fromhex('01120000')])

    def test_an_offset_too_small_to_move_is_refused(self):
        s = self.transport()
        with self.assertRaises(ValueError):
            s.offset_frame(0.05)
        self.assertEqual(self.slide(s), [])

    def test_shift_image_uses_the_models_measured_step(self):
        """A shift towards +x is a forward move on a ProScan 4000, a backward
        one on an RPS 10M, each by its own step size."""
        for model, mm, payload, moved in ((0x002C, 0.6, '00050000', 0.582),
                                          (0x0048, 0.6, '01060000', 0.591),
                                          (0x0048, -0.6, '00060000', -0.591)):
            with self.subTest(model=model, mm=mm):
                s = scanner(PROSCAN_4000, model,
                            device=FakeStrip(8, STATE_FILMSTRIP, frame=1))
                s._transport_initialized = True
                self.assertAlmostEqual(s.shift_image(mm), moved, places=3)
                self.assertEqual(self.slide(s), [bytes.fromhex(payload)])

    def test_shift_image_refuses_an_unmeasured_step_or_a_tiny_shift(self):
        s = scanner(PROSCAN_4000, 0x002F, device=FakeStrip(8, STATE_FILMSTRIP, frame=1))
        with self.assertRaises(PieusbError):
            s.shift_image(1.0)
        s = self.transport()
        with self.assertRaises(ValueError):
            s.shift_image(0.05)
        self.assertEqual(self.slide(s), [])

    def test_frame_counts_outside_the_field_are_refused(self):
        s = self.transport()
        for bad in (0, 256):
            with self.subTest(frames=bad), self.assertRaises(ValueError):
                s.advance(bad)
        self.assertEqual(self.slide(s), [])

    def test_the_first_move_primes_the_transport(self):
        """An uninitialised transport gets SLIDE INIT before the move."""
        s = self.transport()
        s._transport_initialized = False
        s.advance()
        self.assertEqual(self.slide(s),
                         [bytes.fromhex('10010000'), bytes.fromhex('04010000')])

    def test_advance_returns_the_frames_the_counter_moved(self):
        s = self.transport(frames=8, frame=1)
        self.assertEqual(s.advance(), 1)
        self.assertEqual(s.advance(2), 2)
        self.assertEqual(s.dev.frame, 4)

    def test_advance_past_the_last_frame_reports_the_shortfall(self):
        """NEXT 3 from frame 5 of 7: the counter stops at 6."""
        s = self.transport(frames=7, frame=5)
        self.assertEqual(s.advance(3), 1)

    def test_advance_at_the_last_frame_raises_end_of_strip(self):
        """The ProScan 4000 at frame 5 of 6: NEXT accepted, counter unchanged."""
        s = self.transport(frames=6, frame=5)
        with self.assertRaises(EndOfStrip) as caught:
            s.advance()
        self.assertEqual(caught.exception.frame, 5)
        self.assertIsInstance(caught.exception, PieusbError)
        self.assertEqual(self.slide(s), [bytes.fromhex('04010000')], 'NEXT was sent')

    def test_rewind_returns_the_frames_the_counter_moved(self):
        s = self.transport(frames=8, frame=3)
        self.assertEqual(s.rewind(2), 2)
        self.assertEqual(s.rewind(5), 1, 'stops at frame 0')
        self.assertEqual(s.rewind(), 0, 'no error at frame 0')

    def test_every_transport_call_is_refused_without_the_capability(self):
        s = scanner(PROSCAN_10T, 0x0047)
        for call in (lambda: s.advance(), lambda: s.rewind(),
                     lambda: s.eject(), lambda: s.offset_frame(2),
                     lambda: s.init_transport()):
            with self.subTest(call=call), self.assertRaises(PieusbError):
                call()
        self.assertEqual(s.dev.sent, [])

    # Byte 6 in every state observed so far: on hardware and in the captures.
    OBSERVED_FLAGS = (
        (0x01, 'slide carrier'),
        (0x0d, 'filmstrip, frame 0'),
        (0x1d, 'filmstrip, mid-strip'),
        (0x35, 'filmstrip, further along'),
        (0x0c, 'after eject up'),
        (0x14, 'after eject down'),
    )

    def state_for(self, byte6):
        return scanner(PROSCAN_4000, 0x002C,
                       bytes([0x51, 0, 0, 0, 0x1e, 0, byte6, 0])).transport_state()

    def test_medium_present_holds_in_every_observed_state(self):
        for byte6, label in self.OBSERVED_FLAGS:
            with self.subTest(state=label):
                self.assertEqual(self.state_for(byte6).medium_present,
                                 'eject' not in label)

    def test_filmstrip_carrier_holds_in_every_observed_state(self):
        for byte6, label in self.OBSERVED_FLAGS:
            with self.subTest(state=label):
                self.assertEqual(self.state_for(byte6).filmstrip_carrier,
                                 'slide' not in label)

    def test_no_meaning_is_claimed_for_the_remaining_bits(self):
        """0x0d and 0x35 both occur at frame 1 on different strips.

        Any boolean read off bits 3-5 would contradict one of them, so
        TransportState exposes only the raw byte for those.
        """
        for name in ('can_advance', 'can_rewind', 'at_last_frame', 'at_limit'):
            self.assertFalse(hasattr(self.state_for(0x35), name), name)
        self.assertEqual(self.state_for(0x35).flags, 0x35)


class ExposureRouting(unittest.TestCase):
    """Exposure rides on exp_rel_* on every model."""

    def test_the_generic_exposure_exists_on_every_model(self):
        for inq, model in ((PROSCAN_10T, 0x0047), (PROSCAN_4000, 0x002C)):
            with self.subTest(model=hex(model)):
                names = [p.opt.name for p in
                         generate_options(inq, capabilities_for(model)).table]
                for name in ('exposure', 'exposure_r', 'exposure_g', 'exposure_b'):
                    self.assertIn(name, names)
                self.assertNotIn('exp_scale_r', names)

    def test_the_device_fields_are_marked_advanced(self):
        table = generate_options(PROSCAN_4000, capabilities_for(0x002C)).table
        advanced = {p.opt.name for p in table if p.opt.advanced}
        self.assertEqual(advanced, {
            *(f'exp_time_{c}' for c in 'rgbi'), *(f'exp_rel_{c}' for c in 'rgb'),
            *(f'gain_{c}' for c in 'rgbi'), *(f'offset_{c}' for c in 'rgbi'),
            'light',
        })

    def test_auto_exp_writes_the_generic_control(self):
        self.assertEqual(METERED_OPTIONS, ('exposure_r', 'exposure_g', 'exposure_b'))

    # --- exp_rel_* = 100 x the factor -------------------------------------------

    def proscan_10t(self, overrides=None, exposure_time=(4100, 4100, 4100)):
        return scanner(PROSCAN_10T, 0x0047, overrides=overrides, replies={
            transport.SCSI_READ_GAIN_OFFSET: gain_offset_reply(exposure=exposure_time)})

    def exp_rel(self, s):
        return [s.params[f'exp_rel_{c}'].value for c in 'rgb']

    def test_at_1_0_nothing_is_touched(self):
        """The line that protects the ProScan 10T, and now every model.

        At the default exposure the context must not read the device, let alone
        move an option.
        """
        for s in (self.proscan_10t(), self.proscan_4000()):
            with self.subTest(model=hex(s.info.inquiry.model)):
                before = {p.opt.name: p.value for p in s.params.table}
                with s._exposure():
                    self.assertEqual({p.opt.name: p.value for p in s.params.table}, before)
                    self.assertEqual(s._applied_factors, (1.0, 1.0, 1.0))
                self.assertEqual(s.dev.sent, [], 'no GET GAIN OFFSET was sent')

    def test_a_relative_factor_becomes_exp_rel(self):
        s = self.proscan_10t({'exposure': 2.0, 'exposure_b': 1.5})
        with s._exposure():
            self.assertEqual(self.exp_rel(s), [200, 200, 300])
        self.assertEqual(self.exp_rel(s), [100, 100, 100], 'and is restored afterwards')

    def test_a_relative_factor_is_clamped_at_the_timer_ceiling(self):
        s = self.proscan_10t({'exposure_r': 20.0})
        with self.assertLogs('pieusb.scanner', level='WARNING'):
            with s._exposure():
                self.assertEqual(self.exp_rel(s)[0], 1598)
                self.assertAlmostEqual(s._applied_factors[0], 15.98)

    def test_a_relative_model_cannot_go_below_1_0(self):
        s = self.proscan_10t({'exposure_g': 0.5})
        with self.assertLogs('pieusb.scanner', level='WARNING'):
            with s._exposure():
                self.assertEqual(self.exp_rel(s), [100, 100, 100])
        self.assertEqual(s.dev.sent, [], 'no ceiling needed, so no device read')

    def test_exp_rel_set_directly_wins(self):
        s = self.proscan_10t({'exposure_r': 2.0, 'exp_rel_r': 300})
        with s._exposure():
            self.assertEqual(self.exp_rel(s)[0], 300)
            self.assertEqual(s._applied_factors[0], 3.0)

    def test_a_relative_pass_that_skips_is_sent_the_device_calibration(self):
        """Unscaled, with exp_rel still carrying the factor."""
        s = self.proscan_10t({'exposure': 2.0})
        with s._exposure(skip=True):
            self.assertEqual([s.params[f'exp_time_{c}'].value for c in 'rgb'],
                             [4100, 4100, 4100])
            self.assertEqual([s.params[f'gain_{c}'].value for c in 'rgb'], [37, 44, 48])
            self.assertEqual([s.params[f'offset_{c}'].value for c in 'rgb'], [15, 12, 13])
            self.assertEqual(s.params['light'].value, 7)
            self.assertEqual(self.exp_rel(s), [200, 200, 200])
        self.assertEqual(s.params['exp_time_r'].value, option.DEFAULT_EXPOSURE_TIME,
                         'and is restored afterwards')

    def test_a_relative_pass_that_skips_keeps_a_gain_set_directly(self):
        s = self.proscan_10t({'gain_r': 27})
        with s._exposure(skip=True):
            self.assertEqual([s.params[f'gain_{c}'].value for c in 'rgb'], [27, 44, 48])

    def test_a_relative_pass_skips_only_with_a_reference_cached(self):
        s = self.proscan_10t({'reuse_calibration': True})
        self.assertFalse(s._skips_analysis(None))
        s._shading = object()
        self.assertTrue(s._skips_analysis(None))
        self.assertFalse(s._skips_analysis(False))

    # --- the ProScan 4000, whose own exposure times are longer ----------------

    def proscan_4000(self, overrides=None, **reply):
        return scanner(PROSCAN_4000, 0x002C, overrides=overrides, replies={
            transport.SCSI_READ_GAIN_OFFSET: gain_offset_reply(**reply)})

    def test_a_proscan_4000_exposure_is_exp_rel_too(self):
        """Its ceiling follows its own, longer exposure times: 10286 on red."""
        s = self.proscan_4000({'exposure': 2.0, 'exposure_r': 4.0})
        with self.assertLogs('pieusb.scanner', level='WARNING'):
            with s._exposure():
                self.assertEqual(self.exp_rel(s), [637, 200, 200])
        self.assertEqual(s.params['exp_time_r'].value, option.DEFAULT_EXPOSURE_TIME,
                         'exp_time is not the control')

    def test_a_skipping_pass_is_sent_the_proscan_4000_settings(self):
        s = self.proscan_4000()
        with s._exposure(skip=True):
            self.assertEqual([s.params[f'exp_time_{c}'].value for c in 'rgbi'],
                             [10286, 7345, 4405, 5874])
            self.assertEqual([s.params[f'gain_{c}'].value for c in 'rgbi'], [37, 44, 48, 57])
            self.assertEqual([s.params[f'offset_{c}'].value for c in 'rgbi'], [15, 12, 13, 12])
            self.assertEqual(s.params['light'].value, 7)

    def test_an_option_the_caller_set_is_not_seeded_over(self):
        """Otherwise gain, offset, light and exp_time are all unsettable."""
        s = self.proscan_4000({'gain_g': 30, 'light': 4, 'exp_time_r': 5000})
        # light=4 is also the default: assigning it is still a choice, and the
        # device reports 7, so this is the case a default comparison would miss.
        self.assertEqual(s.params['light'].value, s.params['light'].opt.default)
        with s._exposure(skip=True):
            self.assertEqual(s.params['gain_g'].value, 30, 'caller wins')
            self.assertEqual(s.params['light'].value, 4, 'caller wins')
            self.assertEqual(s.params['exp_time_r'].value, 5000, 'caller wins')
            self.assertEqual(s.params['gain_r'].value, 37, 'device still seeds this')
            self.assertEqual(s.params['exp_time_g'].value, 7345, 'and this')

    def test_a_device_without_settings_of_its_own_is_left_as_configured(self):
        """A scanner that has not run a shading analysis reports zeros; do not seed them."""
        s = self.proscan_4000(exposure=(0, 0, 0))
        with s._exposure(skip=True):
            self.assertEqual(s.params['exp_time_r'].value, option.DEFAULT_EXPOSURE_TIME)
            self.assertEqual(s.params['gain_r'].value, s.params['gain_r'].opt.default)

    def test_the_result_reports_the_exposure_either_way(self):
        """exp_rel takes effect whether or not the pass analyses."""
        s = self.proscan_4000({'exposure_r': 2.0})
        with s._exposure():
            skipped = s._result(started=0.0, shading_acquired=False)
            analysed = s._result(started=0.0, shading_acquired=True)
        self.assertEqual(skipped.exposure_factor, (2.0, 1.0, 1.0))
        self.assertEqual(analysed.exposure_factor, (2.0, 1.0, 1.0))


class Passes(unittest.TestCase):
    """Which passes a scan runs: a metering pass for auto_exp, then the real one."""

    def run_scan(self, s, refuse_skip=0, level=0.5):
        """_run_scan over a stubbed _scan_pass, recording each pass.

        The stub acquires a reference when asked not to reuse, or when refusing a
        skip -- the first `refuse_skip` passes that ask to skip are refused.
        """
        passes = []
        refusals = [refuse_skip]

        def fake_pass(started, emit, autofocus=False, reuse=None):
            if reuse is None:
                reuse = s.params['reuse_calibration'].value
            acquire = not reuse or s._shading is None
            if not acquire and refusals[0]:
                refusals[0] -= 1
                acquire = True
            if acquire:
                s._shading = object()
            passes.append({'reuse': reuse, 'acquired': acquire,
                           'exp_time': s.params['exp_time_r'].value,
                           'exposure': s._applied_factors})
            rgb = numpy.full((4, 4, 3), int(level * 65535), dtype=numpy.uint16)
            return s._result(rgb, started=started, shading_acquired=acquire)

        s._scan_pass = fake_pass
        return s._run_scan(0.0), passes

    def proscan_4000(self, overrides=None):
        return scanner(PROSCAN_4000, 0x002C, overrides=overrides, replies={
            transport.SCSI_READ_GAIN_OFFSET: gain_offset_reply()})

    def test_an_exposure_scans_once_on_every_model(self):
        tenth = scanner(PROSCAN_10T, 0x0047, overrides={'exposure': 2.0}, replies={
            transport.SCSI_READ_GAIN_OFFSET: gain_offset_reply(exposure=(4100,) * 3)})
        for s in (tenth, self.proscan_4000({'exposure': 2.0})):
            with self.subTest(model=hex(s.info.inquiry.model)):
                result, passes = self.run_scan(s)
                self.assertEqual([p['acquired'] for p in passes], [True])
                self.assertEqual(result.exposure_factor, (2.0, 2.0, 2.0))

    def test_a_reused_reference_skips_and_is_sent_the_devices_settings(self):
        s = self.proscan_4000({'exposure': 2.0, 'reuse_calibration': True})
        s._shading = object()
        result, passes = self.run_scan(s)
        self.assertEqual([(p['acquired'], p['exp_time']) for p in passes], [(False, 10286)])
        self.assertEqual(result.exposure_factor, (2.0, 2.0, 2.0))

    def test_a_refused_skip_still_honours_the_exposure(self):
        """The scanner insisting on its analysis costs nothing: exp_rel holds."""
        s = self.proscan_4000({'exposure': 2.0, 'reuse_calibration': True})
        s._shading = object()
        result, passes = self.run_scan(s, refuse_skip=1)
        self.assertEqual([p['acquired'] for p in passes], [True])
        self.assertEqual(result.exposure_factor, (2.0, 2.0, 2.0))

    def test_auto_exp_meters_into_the_generic_control(self):
        s = self.proscan_4000({'auto_exp': True})
        result, passes = self.run_scan(s, level=0.25)
        meter, final = passes
        self.assertEqual(meter['exposure'], (1.0, 1.0, 1.0))
        for f in result.exposure_factor:
            self.assertAlmostEqual(f, 0.85 / 0.25, places=2)
        self.assertEqual([s.params[n].value for n in METERED_OPTIONS], [1.0] * 3,
                         'the metered values belong to that scan alone')

    def test_the_master_exposure_compensates_auto_exp(self):
        s = self.proscan_4000({'auto_exp': True, 'exposure': 0.5})
        result, _ = self.run_scan(s, level=0.25)
        for f in result.exposure_factor:
            self.assertAlmostEqual(f, 0.5 * 0.85 / 0.25, places=2)

    def test_auto_exp_cannot_go_below_1_0(self):
        s = scanner(PROSCAN_10T, 0x0047, overrides={'auto_exp': True}, replies={
            transport.SCSI_READ_GAIN_OFFSET: gain_offset_reply(exposure=(4100,) * 3)})
        with self.assertLogs('pieusb.scanner', level='WARNING'):
            result, _ = self.run_scan(s, level=0.95)
        self.assertEqual(result.exposure_factor, (1.0, 1.0, 1.0))


class BaseMetering(unittest.TestCase):
    """Metering on the film base: auto_exp where the preview shows a gap, and
    meter_base(), which looks forward for one. Images are recorded RPS 10M scans
    (tests/reference/base_scans.npz)."""

    def setUp(self):
        with numpy.load(REFERENCE_DIR / 'base_scans.npz') as scans:
            self.gap = scans['rps10m_thin_full_gap']
            self.no_gap = scans['rps10m_thin_no_gap']
        self.base = film_base(self.gap, find_bands(self.gap, 300 / 25.4)).level

    def rps10m(self, overrides=None):
        dev = FakeStrip(8, STATE_FILMSTRIP, frame=1, replies={
            transport.SCSI_READ_GAIN_OFFSET: gain_offset_reply(exposure=(4100,) * 3)})
        s = scanner(PROSCAN_4000, 0x0048, overrides=overrides, device=dev)
        s._transport_initialized = True
        return s

    def passes(self, s, *images):
        """Stub _scan_pass to return `images` in turn; returns the list of the
        exposure factors each pass ran at."""
        queue, ran = list(images), []

        def fake_pass(started, emit, autofocus=False, reuse=None):
            ran.append(s._applied_factors)
            return s._result(queue.pop(0), started=started)

        s._scan_pass = fake_pass
        return ran

    def slides(self, s):
        return [c['out'] for c in s.dev.of(transport.SCSI_SLIDE)]

    def factors(self, s):
        return [s.params[n].value for n in METERED_OPTIONS]

    def assertMeteredOn(self, factors, levels):
        for got, level in zip(factors, levels):
            self.assertAlmostEqual(got, 0.85 / level, delta=0.01 * got)

    def test_auto_exp_meters_the_base_where_a_gap_shows(self):
        """Frame 4 is thin: its picture alone tops out at 0.62 of the base in
        red, so only the gap gives the strip's exposure."""
        s = self.rps10m({'auto_exp': True})
        ran = self.passes(s, self.gap, self.gap)
        s._run_scan(0.0)
        self.assertMeteredOn(ran[1], self.base)
        self.assertEqual(s.film_base.level, self.base)

    def test_auto_exp_meters_the_percentile_without_a_gap(self):
        s = self.rps10m({'auto_exp': True})
        ran = self.passes(s, self.no_gap, self.no_gap)
        s._run_scan(0.0)
        p99 = [numpy.percentile(self.no_gap[:, :, c], 99) / 65535 for c in range(3)]
        self.assertMeteredOn(ran[1], p99)
        self.assertIsNone(s.film_base)
        self.assertEqual(self.slides(s), [], 'auto_exp never moves the film')

    def test_meter_base_uses_a_gap_in_the_gate_without_moving(self):
        s = self.rps10m()
        self.passes(s, self.gap)
        self.assertEqual(s.meter_base().level, self.base)
        self.assertEqual(self.slides(s), [])
        self.assertMeteredOn(self.factors(s), self.base)
        self.assertTrue(all(s.params[n].assigned for n in METERED_OPTIONS),
                        'the lock holds for the scans that follow')

    def test_meter_base_looks_70_steps_forward_and_comes_back(self):
        s = self.rps10m()
        self.passes(s, self.no_gap, self.gap)
        s.meter_base()
        self.assertEqual(self.slides(s), [bytes.fromhex('00460000'),
                                          bytes.fromhex('01460000')])
        self.assertMeteredOn(self.factors(s), self.base)

    def test_meter_base_without_a_gap_raises_with_the_film_returned(self):
        s = self.rps10m()
        self.passes(s, self.no_gap, self.no_gap)
        with self.assertRaises(PieusbError):
            s.meter_base()
        self.assertEqual(self.slides(s), [bytes.fromhex('00460000'),
                                          bytes.fromhex('01460000')])
        self.assertEqual(self.factors(s), [1.0] * 3, 'nothing set')

    def test_meter_base_needs_a_film_transport(self):
        s = scanner(PROSCAN_10T, 0x0047)
        with self.assertRaises(PieusbError):
            s.meter_base()
        self.assertEqual(s.dev.sent, [])


# ---------------------------------------------------------------------------
# 2. Golden masters -- today's verified behaviour, frozen
# ---------------------------------------------------------------------------

# (name, option overrides, skip_shading_analysis), covering every branch in
# set_options(): both quality bits, both sample sizes, a crop, and the
# per-channel exposure that balances a colour negative on this unit.
GOLDEN_CASES = [
    ('10t_defaults', {}, False),
    ('10t_rgbi_sharpen', {'mode': 'rgbi', 'sharpen': True}, False),
    ('10t_skip_shading', {'mode': 'rgbi', 'fast_infrared': True}, True),
    ('10t_crop_8bit', {
        'color_depth': 8, 'resolution': 1200,
        'tl_x': 1000, 'tl_y': 500, 'br_x': 9000, 'br_y': 6000,
        'gain_r': 40, 'gain_g': 22, 'gain_b': 32, 'gain_i': 57,
        'offset_r': 12, 'offset_g': 16, 'offset_b': 16, 'offset_i': 16,
        'exp_time_r': 3706, 'exp_time_g': 3685, 'exp_time_b': 3707,
        'exp_time_i': 4100, 'light': 4,
    }, False),
    ('10t_negative_exposure', {
        'mode': 'rgbi',
        'exp_rel_r': 247, 'exp_rel_g': 563, 'exp_rel_b': 1086,
    }, False),
]


class GoldenMasters(unittest.TestCase):
    '''What set_options() emits for a 10T, frozen.'''

    def _check(self, name, transcript):
        path = GOLDEN_DIR / (name + '.txt')
        if REGENERATE:
            path.parent.mkdir(exist_ok=True)
            path.write_text(transcript)
            self.skipTest('regenerated %s' % path.name)
        self.assertTrue(path.exists(),
                        'missing golden %s; run with --regenerate-golden' % path)
        self.assertEqual(
            transcript, path.read_text(),
            '\n%s has changed.\n'
            'This is what a verified ProScan 10T receives. If the change was\n'
            'intended, re-run with --regenerate-golden and review the diff.\n'
            % path.name)

    def test_option_defaults(self):
        '''The default option table: a changed default changes the wire.'''
        options = generate_options(PROSCAN_10T)
        lines = ['%-20s %r' % (p.opt.name, p.value) for p in options.table]
        self._check('10t_option_defaults', '\n'.join(lines) + '\n')

    def test_set_options_transcripts(self):
        for name, overrides, skip in GOLDEN_CASES:
            with self.subTest(case=name):
                device = send(PROSCAN_10T, overrides, skip_shading_analysis=skip)
                self._check(name, device.transcript())


# ---------------------------------------------------------------------------
# 3. Conformance against the ProScan 4000 captures
# ---------------------------------------------------------------------------

CAPTURE = ('ProScan 4000/with_bulk/'
           'ProScan4000_Slide_64BitRGBI_300DPI_FULL_MF-Max_Gain-1-1-1_Exp-1-1_Scan'
           '.pcapng.gz')


class ProScan4000Captures(unittest.TestCase):
    '''What the vendor sends a ProScan 4000, against what we would send it.'''

    @classmethod
    def setUpClass(cls):
        cls.txns = load_capture(CAPTURE)
        if cls.txns is None:
            raise unittest.SkipTest('capture corpus not present')

    def vendor(self, opcode, sub=None, index=0):
        hits = [t for t in self.txns
                if t.opcode == opcode and (sub is None or t.sub == sub)]
        return hits[index]

    def test_mode_select_is_reproduced_exactly(self):
        '''16-bit RGBI at 300 dpi with fast infrared and a skipped shading pass.'''
        want = self.vendor(transport.SCSI_MODE_SELECT).out
        device = send(PROSCAN_4000,
                      {'mode': 'rgbi', 'color_depth': 16, 'resolution': 300,
                       'fast_infrared': True},
                      skip_shading_analysis=True)
        self.assertEqual(device.one(transport.SCSI_MODE_SELECT)['out'], want)

    def test_scan_frame_is_reproduced_exactly(self):
        want = self.vendor(transport.SCSI_WRITE, transport.SCSI_SCAN_FRAME).out
        _, x0, y0, x1, y1 = struct.unpack_from('<HHHHH', want, 4)
        device = send(PROSCAN_4000,
                      {'tl_x': x0, 'tl_y': y0, 'br_x': x1, 'br_y': y1})
        got = device.one(transport.SCSI_WRITE, transport.SCSI_SCAN_FRAME)
        self.assertEqual(got['out'], want)

    def test_our_seeding_reproduces_the_vendors_gain_offset_write(self):
        """The vendor reads GET GAIN OFFSET and writes it back; so do we.

        Straight from the capture: its 0xD7 reply through our field offsets must
        give the exp_time its 0xDC carried.
        """
        reply = bytes(self.vendor(transport.SCSI_READ_GAIN_OFFSET).data)
        wrote = self.vendor(transport.SCSI_WRITE_GAIN_OFFSET).out

        self.assertEqual(struct.unpack_from('<3H', reply, 60),
                         struct.unpack_from('<3H', wrote, 0))
        self.assertEqual(tuple(reply[72:75]), tuple(wrote[12:15]), 'gain')
        self.assertEqual(tuple(reply[66:69]), tuple(wrote[6:9]), 'offset')
        self.assertEqual(reply[75], wrote[15], 'light')

    def test_the_devices_own_exposure_is_a_settable_value(self):
        """A ProScan 4000 advertises maximum_exposure 2500 and calibrates to 10286.

        Four times the advertised maximum is 10000, so a ceiling derived from
        INQUIRY would refuse the scanner its own exposure.
        """
        self.assertEqual(PROSCAN_4000.maximum_exposure, 2500)
        options = generate_options(PROSCAN_4000)
        for value in (PROSCAN_4000.minimum_exposure, 2937, 10286, 0xFFFF):
            with self.subTest(value=value):
                self.assertTrue(options['exp_time_r'].opt.validate(value))
        for value in (PROSCAN_4000.minimum_exposure - 1, 0x10000):
            with self.subTest(value=value):
                self.assertFalse(options['exp_time_r'].opt.validate(value))

    def test_gain_offset_is_reproduced_exactly_at_23_bytes(self):
        '''The scanner's own settings, in the 23-byte form.'''
        want = self.vendor(transport.SCSI_WRITE_GAIN_OFFSET).out
        self.assertEqual(len(want), 23)
        exp_r, exp_g, exp_b = struct.unpack_from('<3H', want, 0)
        device = send(PROSCAN_4000, {
            'exp_time_r': exp_r, 'exp_time_g': exp_g, 'exp_time_b': exp_b,
            'offset_r': want[6], 'offset_g': want[7], 'offset_b': want[8],
            'gain_r': want[12], 'gain_g': want[13], 'gain_b': want[14],
            'light': want[15],
            'exp_time_i': struct.unpack_from('<H', want, 18)[0],
            'offset_i': want[20], 'gain_i': want[22],
        }, reject={(transport.SCSI_WRITE_GAIN_OFFSET, 29): CheckCondition(0x05, 0x1A, 0x00)})
        attempts = device.of(transport.SCSI_WRITE_GAIN_OFFSET)
        self.assertEqual([a['cdb_length'] for a in attempts], [29, 23],
                         'the long form is tried first, then the short one')
        self.assertEqual(attempts[-1]['out'], want)

    def test_a_device_that_accepts_29_is_never_sent_23(self):
        '''A device that takes the long form is sent nothing else.'''
        device = send(PROSCAN_10T)
        self.assertEqual(
            [c['cdb_length'] for c in device.of(transport.SCSI_WRITE_GAIN_OFFSET)],
            [29])

    def test_the_accepted_length_is_probed_once_per_device(self):
        '''Probing costs a rejected command and a REQUEST SENSE every pass.'''
        options = generate_options(PROSCAN_4000)
        options.validate()
        device = FakeDevice(reject={(transport.SCSI_WRITE_GAIN_OFFSET, 29):
                                    CheckCondition(0x05, 0x1A, 0x00)})
        for _ in range(3):
            set_options(device, options)
        self.assertEqual(
            [c['cdb_length'] for c in device.of(transport.SCSI_WRITE_GAIN_OFFSET)],
            [29, 23, 23, 23])

    def test_transport_state_decodes_what_the_vendor_read_back(self):
        """The first READ STATE after a SLIDE INIT echoes the focus it set.

        Only the first: the focus returns to a parked 0x03 once the pass is over,
        so later replies report where the motor is, not what was asked for.
        """
        sent = None
        checked = 0
        for t in self.txns:
            if t.opcode == transport.SCSI_SLIDE and t.out and t.out[0] == transport.SLIDE_INIT:
                sent = t.out[1]
            elif t.opcode == transport.SCSI_READ_STATE and t.data and sent is not None:
                state = scanner(PROSCAN_4000, 0x002C,
                                bytes(t.data)[:8]).transport_state()
                self.assertEqual(state.focus, sent, bytes(t.data).hex(' '))
                self.assertEqual(state.focus_max, 80, 'slide carrier')
                sent = None
                checked += 1
        self.assertGreater(checked, 0)

    def test_a_filmstrip_carrier_reports_a_shorter_focus_range(self):
        """The upper bound is a property of the carrier, not the model."""
        txns = load_capture('ProScan 4000/with_bulk/ProScan4000_EjectTop.pcapng.gz')
        if txns is None:
            self.skipTest('capture not present')
        replies = [bytes(t.data)[:8] for t in txns
                   if t.opcode == transport.SCSI_READ_STATE and t.data]
        self.assertTrue(replies)
        for reply in replies:
            state = scanner(PROSCAN_4000, 0x002C, reply).transport_state()
            self.assertEqual(state.focus_max, 30, reply.hex(' '))

    def test_the_offset_conversion_reproduces_the_captured_payloads(self):
        """The `Frame1_Offset±5` captures, against offset_frame(±5).

        The filename gives the millimetres the operator asked for, so this checks
        the 1/240 inch conversion and the sign convention end to end. A positive
        offset moves with SLIDE_OFFSET_FORWARD first and undoes it afterwards; a
        negative one starts with the opposite action.
        """
        log = logging.getLogger('pieusb.scanner')
        level = log.level
        log.setLevel(logging.ERROR)
        try:
            for mm, first in ((5, transport.SLIDE_OFFSET_FORWARD),
                              (-5, transport.SLIDE_OFFSET_BACK)):
                name = ('ProScan 4000/Film feeding/ProScan4000_Frame1_Offset%s'
                        '.pcapng.gz' % ('5' if mm > 0 else '-5'))
                txns = load_capture(name)
                if txns is None:
                    self.skipTest('capture not present')
                vendor = [t.out for t in txns if t.opcode == transport.SCSI_SLIDE
                          and t.out[0] in (transport.SLIDE_OFFSET_FORWARD,
                                           transport.SLIDE_OFFSET_BACK)]
                with self.subTest(mm=mm):
                    self.assertEqual(vendor[0][0], first, 'which action came first')
                    s = scanner(PROSCAN_4000, 0x002C, STATE_FILMSTRIP)
                    s._transport_initialized = True
                    s.offset_frame(mm)
                    self.assertEqual(s.dev.of(transport.SCSI_SLIDE)[0]['out'],
                                     vendor[0])
        finally:
            log.setLevel(level)

    def test_we_do_not_copy_the_vendors_swapped_green_length(self):
        '''The vendor's green EXPOSURE entry carries its length byte-swapped:
        `13 00 00 04 ...` where red and blue send `13 00 04 00 ...`. Red and blue
        must match the vendor, green must match SANE.
        '''
        vendor = [t.out for t in self.txns if t.sub == transport.SCSI_EXPOSURE]
        device = send(PROSCAN_4000)
        ours = [c['out'] for c in device.of(transport.SCSI_WRITE,
                                            transport.SCSI_EXPOSURE)]
        self.assertEqual(ours[0], vendor[0], 'red')
        self.assertEqual(ours[2], vendor[2], 'blue')
        self.assertEqual(vendor[1][2:4], b'\x00\x04', 'the vendor quirk itself')
        self.assertEqual(ours[1][2:4], b'\x04\x00', 'we send the length correctly')
        self.assertEqual(ours[1], ref(sub=transport.SCSI_EXPOSURE, index=1).out,
                         'and identically to SANE on the 10T')


# The controlled matrix: one baseline plus five captures that each move a single
# VueScan slider. Everything else about the six runs is the same.
EXPOSURE_MATRIX = 'ProScan 4000/Exposure-Gain'


class ProScan4000ExposureMatrix(unittest.TestCase):
    """Where a ProScan 4000's exposure actually lives, from the vendor's own runs.

    Six VueScan captures, each moving one slider. `captures/tools/capdiff.py`
    reduces them to the two assertions below.
    """

    @classmethod
    def setUpClass(cls):
        directory = CAPTURE_DIR / EXPOSURE_MATRIX
        if not directory.is_dir():
            raise unittest.SkipTest('capture corpus not present')
        cls.runs = {}
        for path in sorted(directory.glob('*.pcapng.gz')):
            txns = load_capture(f'{EXPOSURE_MATRIX}/{path.name}')
            # The slider settings are in the filename: ..._ExpRGB2_ExpI1_Gain1-1-1
            key = path.name.split('MF-Mid_')[1].split('.pcapng')[0]
            cls.runs[key] = txns
        if len(cls.runs) < 2:
            raise unittest.SkipTest('exposure matrix incomplete')

    def only(self, txns, opcode, sub=None):
        hits = [t for t in txns
                if t.opcode == opcode and (sub is None or t.sub == sub)]
        self.assertEqual(len(hits), 1, f'expected one 0x{opcode:02X} per run')
        return hits[0]

    def test_the_rgb_exposure_slider_moves_only_the_gain_offset_exposure(self):
        """VueScan multiplies the device's own exp_time; nothing else moves."""
        base = self.only(self.runs['ExpRGB1_ExpI1_Gain1-1-1'],
                         transport.SCSI_WRITE_GAIN_OFFSET).out
        doubled = self.only(self.runs['ExpRGB2_ExpI1_Gain1-1-1'],
                            transport.SCSI_WRITE_GAIN_OFFSET).out
        self.assertEqual(struct.unpack_from('<3H', doubled, 0),
                         tuple(2 * v for v in struct.unpack_from('<3H', base, 0)),
                         'exposure x2 doubles exp_time_r/g/b')
        self.assertEqual(base[6:], doubled[6:],
                         'offset, gain, light and the infrared fields are untouched')

    def test_the_relative_exposure_command_never_moves(self):
        """SCSI_EXPOSURE stays at 100 on every run, so it is not the control here.

        The library drives exposure through it on every model anyway: measured
        on a ProScan 10T and 4000, it takes effect whether or not a pass runs the
        shading analysis, so no calibration pass is needed for it.
        """
        for name, txns in self.runs.items():
            with self.subTest(run=name):
                writes = [t for t in txns if t.opcode == transport.SCSI_WRITE
                          and t.sub == transport.SCSI_EXPOSURE]
                self.assertEqual(len(writes), 3, 'one write per colour')
                for w in writes:
                    self.assertEqual(struct.unpack_from('<H', w.out, 6)[0], 100)

    def test_every_pass_carrying_an_exposure_skips_the_shading_analysis(self):
        """The gate. MODE SELECT byte 9 bit 0x08 is set on all six.

        A pass that lets the device run its shading analysis has the device
        re-derive its own exposure, overriding what the 0xDC write asked for --
        so the exposure and the skip bit are one mechanism, not two. This is what
        scanner._scan_pass's `acquire` decides, and why a scan that never skips
        cannot honour exp_time_* (PLAN-proscan4000.md 7d).
        """
        for name, txns in self.runs.items():
            with self.subTest(run=name):
                quality = self.only(txns, transport.SCSI_MODE_SELECT).out[9]
                self.assertTrue(quality & 0x08,
                                f'quality byte 0x{quality:02x} has skipShadingAnalysis clear')


if __name__ == '__main__':
    unittest.main(verbosity=2)
