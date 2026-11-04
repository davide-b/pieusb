'''
hwsuite.py's stage selection and sequencing, against a simulated scanner. No USB.

The suite runs unattended on someone else's scanner, so which stages it runs on
which model, and what it does at the end of the strip, after a failed move and
with a short strip, are checked here rather than discovered there.
'''

import dataclasses
import json
import logging
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))

from fixtures import PROSCAN_10T, PROSCAN_4000
from test_wire_format import scanner

import numpy

import exercise
import hwsuite
from hwsuite import Risk
from pieusb import transport
import pieusb.scanner
from pieusb.exceptions import CheckCondition, TransportError
from pieusb.scanner import autofocus_pick
from pieusb.option import DEFAULT_EXPOSURE_TIME, DEFAULT_RELATIVE_EXPOSURE
from pieusb.scanner import TransportState
from pieusb.transport import SLIDE_INIT, SLIDE_NEXT, SLIDE_PREV
from pieusb.types import AutofocusResult, ScanResult

MOTORS_ALL = frozenset({'film_transport', 'focus'})
SHARPEST_FOCUS = 13


class Strip:
    '''A strip of `frames` frames whose counter stops at the last one.

    Stands in for the device under Scanner's transport calls (`_slide` and
    `transport_state`), so advance() and its end-of-strip detection are the
    library's own.
    '''

    def __init__(self, frames, fail_on=None, leaves_on_overshoot=False):
        self.frames = frames
        self.frame = 0
        self.fail_on = fail_on
        self.leaves_on_overshoot = leaves_on_overshoot
        self.flags = 0x0d
        self.raw = bytes(8)
        self.moves = []

    def state(self):
        return TransportState(frame=self.frame, focus=0, focus_max=30, flags=self.flags,
                              raw=self.raw)

    def advance(self, n):
        self.moves.append(('next', n))
        if self.fail_on == 'next':
            raise TransportError('simulated')
        self.frame = min(self.frame + n, self.frames - 1)

    def slide(self, action, value, byte3=0):
        if action == SLIDE_INIT:
            return
        if action == SLIDE_NEXT:
            self.advance(value)
            return
        self.moves.append(('prev', value, byte3))
        assert action == SLIDE_PREV
        if self.leaves_on_overshoot and value > self.frame:
            self.flags &= ~0x01
        self.frame = max(self.frame - value, 0)

    def offset(self, mm):
        self.moves.append(('offset', mm))


class Film(Strip):
    '''A strip with pictures on it, scanned through a window one frame wide.

    Frame i covers film millimetres [i * PITCH, i * PITCH + FRAME), and the base
    shows between frames. `position` is the film millimetre at the window's
    right edge, the next frame lying towards -x in the image as on a ProScan
    4000. NEXT and PREV move one pitch; an offset of m moves the content m x
    `scale` towards +x. Pictures are a fixed texture of the film coordinate, so
    a feature stays put on the film whatever the window shows.
    '''
    PITCH, FRAME, GAP_LEVEL, CLEAR_LEVEL = 37.8, 35.8, 0.18, 0.8
    GRID_MM = 0.05

    def __init__(self, frames, *, scale=1.09, start_off_mm=2.0):
        super().__init__(frames)
        self.scale = scale
        # Off centre by `start_off_mm`, content to the -x side.
        self.position = -(36.5 - self.FRAME) / 2 + start_off_mm
        rng = numpy.random.default_rng(3)
        n = int((frames + 2) * self.PITCH / self.GRID_MM)
        self.levels = 0.02 + 0.1 * rng.random(n)
        self.texture = 0.6 + 0.8 * rng.random((64, n))

    def slide(self, action, value, byte3=0):
        before = self.frame
        super().slide(action, value, byte3)
        self.position += (self.frame - before) * self.PITCH

    def offset(self, mm):
        super().offset(mm)
        self.position += mm * self.scale

    def render(self, width_px, px_per_mm):
        x = self.position + (width_px - 1 - numpy.arange(width_px)) / px_per_mm
        index = numpy.clip(((x + self.PITCH) / self.GRID_MM).astype(int), 0, self.levels.size - 1)
        lum = self.levels[index] * self.texture[:, index]
        within = numpy.mod(x, self.PITCH)
        lum[:, within >= self.FRAME] = self.GAP_LEVEL
        lum[:, (x < 0) | (x >= self.frames * self.PITCH)] = self.CLEAR_LEVEL
        return (numpy.repeat(lum[:, :, None], 3, axis=2) * 65535).astype(numpy.uint16)

    def scan_pass(self, s, args, prefix):
        p = s.params
        res = p['resolution'].value
        width = round((p['br_x'].value - p['tl_x'].value) / p.inq.max_resolution_x * res)
        rgb = self.render(width, res / 25.4)
        exposure = tuple(p[f'exp_time_{c}'].value for c in 'rgbi')
        gain = tuple(p[f'gain_{c}'].value for c in 'rgbi')
        return 0, ScanResult(rgb=rgb, width=width, height=rgb.shape[0], exposure=exposure,
                             gain=gain, resolution=res, **library_reports(p))


METERED = (2.0, 1.5, 1.25)


def library_reports(p):
    '''What a pass reports about itself: a shading analysis unless it reused a
    reference, and the METERED exposure when it metered.'''
    return {'shading_acquired': not p['reuse_calibration'].value,
            'exposure_factor': METERED if p['auto_exp'].value else None}


def fake_pass_for(honours):
    '''A scan whose level follows the exposure fields the model honours, and
    whose detail is crispest at SHARPEST_FOCUS.

    'both' is what the ProScan 10T and 4000 do: exp_rel on every pass, exp_time
    on a pass that skips the shading analysis. 'exp_time' honours exp_time alone,
    on a skipping pass: a model the library's exposure would not work on.
    '''
    def fake_pass(s, args, prefix):
        p = s.params
        exposure = tuple(p[f'exp_time_{c}'].value for c in 'rgbi')
        gain = tuple(p[f'gain_{c}'].value for c in 'rgbi')
        skipping = p['reuse_calibration'].value
        level = 1000.0
        if skipping:
            level *= exposure[0] / DEFAULT_EXPOSURE_TIME
        if honours == 'both':
            level *= p['exp_rel_r'].value / DEFAULT_RELATIVE_EXPOSURE
        try:
            blur = abs(p['focus'].value - SHARPEST_FOCUS)
        except KeyError:
            blur = 0
        stripes = numpy.where(numpy.arange(32) % 2, 1.0 + 0.5 / (1 + blur), 1.0)
        rgb = numpy.repeat((level * stripes)[numpy.newaxis, :, numpy.newaxis],
                           3, axis=2).repeat(8, axis=0).astype(numpy.uint16)
        return 0, ScanResult(rgb=rgb, width=32, height=8, exposure=exposure, gain=gain,
                             resolution=p['resolution'].value, **library_reports(p))
    return fake_pass


def fake_autofocus(s, peak=SHARPEST_FOCUS, refuse_16bit=False):
    '''Scanner.autofocus() over a 30-position sweep peaking at `peak`. At 16 bits
    it returns uint16 lines with less noise, or is refused.'''
    def autofocus():
        wide = pieusb.scanner.AUTOFOCUS_COLOR_DEPTH == 16
        if wide and refuse_16bit:
            raise CheckCondition(0x05, 0x24, 0x00)
        positions = list(range(1, 31))
        noise = numpy.random.default_rng(0).normal(0, 0.002 if wide else 0.01, 30)
        scores = 1.0 - 0.02 * numpy.abs(numpy.arange(1, 31) - peak) + noise
        best, depth, smoothed = autofocus_pick(scores, positions)
        s.params['focus'].value = best
        s.params['focus'].assigned = True
        return AutofocusResult(focus=best, peaked=True, positions=tuple(positions),
                               scores=tuple(scores), smoothed=tuple(smoothed), depth=depth,
                               lines=numpy.zeros((30, 4, 3),
                                                 numpy.uint16 if wide else numpy.uint8))
    return autofocus


class SuiteRun(unittest.TestCase):

    def setUp(self):
        # Transport calls log at INFO and WARNING; they would drown the run.
        log = logging.getLogger('pieusb.scanner')
        level = log.level
        log.setLevel(logging.ERROR)
        self.addCleanup(log.setLevel, level)

    def run_suite(self, strip=None, inquiry=PROSCAN_4000, model=0x002C,
                  profile='characterize', motors=MOTORS_ALL, honours='both',
                  only=None, autofocus=None, enabled=frozenset(), on_reopen=None,
                  **settings):
        strip = strip or Strip(8)

        def wired():
            s = scanner(inquiry, model)
            if s.info.capabilities.film_transport:
                s.transport_state = strip.state
            s._slide = strip.slide
            s.offset_frame = strip.offset
            s.autofocus = (autofocus or fake_autofocus)(s)
            return s

        def reopen(old):
            self.reopened.append(old)
            if on_reopen:
                on_reopen(strip)
            return wired()

        self.reopened = []
        self.slept = []
        out = Path(self.enterContext(tempfile.TemporaryDirectory()))
        self.out = out
        suite = hwsuite.Suite(wired(), out, reopen=reopen, sleep=self.slept.append, **settings)
        scan_pass = strip.scan_pass if isinstance(strip, Film) else fake_pass_for(honours)
        with mock.patch.object(exercise, 'run_one_pass', scan_pass), \
                mock.patch.object(exercise, 'report_device_settings', lambda s: None), \
                mock.patch('builtins.print'):
            status = suite.run(hwsuite.select_stages(profile, motors, only, enabled))
        summary = json.loads((out / 'summary.json').read_text())
        return status, {st['stage']: st for st in summary['stages']}, suite.scanner, summary


class Selection(unittest.TestCase):

    def runs(self, plan):
        return [stage.name for stage, reason in plan if reason is None]

    def test_characterize_runs_everything_a_motorised_model_has_but_opt_ins(self):
        self.assertEqual(self.runs(hwsuite.select_stages('characterize', MOTORS_ALL)),
                         [n for n in hwsuite.STAGE_NAMES if n != 'drift'])

    def test_an_opt_in_stage_runs_when_its_flag_is_given(self):
        plan = hwsuite.select_stages('regression', MOTORS_ALL, enabled=frozenset({'drift'}))
        self.assertIn('drift', self.runs(plan))
        reason = dict(hwsuite.select_stages('regression', MOTORS_ALL))
        self.assertIn('--soak-minutes', next(r for st, r in reason.items() if st.name == 'drift'))

    def test_regression_runs_no_motor(self):
        stages = self.runs(hwsuite.select_stages('regression', MOTORS_ALL))
        self.assertEqual(stages, ['identity', 'first_frame', 'default_scan', 'exposure',
                                  'calibration'])

    def test_a_motorless_model_runs_no_motor_stage(self):
        plan = hwsuite.select_stages('characterize', hwsuite.motors_for(0x0047, frozenset()))
        self.assertEqual(self.runs(plan), ['identity', 'first_frame', 'default_scan',
                                           'exposure', 'calibration'])
        reasons = {s.name: r for s, r in plan if r}
        self.assertIn('no focus motor', reasons['autofocus'])

    def test_allow_cannot_give_the_10t_a_motor(self):
        with self.assertRaises(SystemExit):
            hwsuite.motors_for(0x0047, frozenset({'film_transport'}))

    def test_an_unknown_model_gets_only_what_allow_names(self):
        self.assertEqual(hwsuite.motors_for(0x00FF, frozenset()), frozenset())
        plan = hwsuite.select_stages('characterize',
                                     hwsuite.motors_for(0x00FF, frozenset({'focus'})))
        self.assertIn('autofocus', self.runs(plan))
        self.assertNotIn('walk', self.runs(plan))

    def test_known_models_take_their_motors_from_the_library(self):
        for model in (0x0048, 0x002C):
            self.assertEqual(hwsuite.motors_for(model, frozenset()), MOTORS_ALL)

    def test_every_motor_stage_needs_its_motor(self):
        for stage in hwsuite.STAGES:
            if stage.risk >= Risk.FOCUS:
                plan = dict(hwsuite.select_stages('characterize', frozenset()))
                self.assertIsNotNone(plan[stage], stage.name)


class ExposureVerdict(unittest.TestCase):

    def test_the_10t_and_4000_reading_supports_the_library(self):
        """As measured: both controls double on a skipping pass, exp_rel on an
        analysing one too."""
        ratios = {'exp_time_x2': [1.97, 1.92, 2.15], 'exp_rel_x2': [1.97, 1.92, 2.15],
                  'exp_rel_x2_analysed': [1.98, 1.95, 2.0]}
        self.assertEqual(hwsuite.exposure_verdict(ratios),
                         (True, ['exp_rel_x2_analysed', 'exp_rel_x2', 'exp_time_x2']))

    def test_exp_rel_ignored_on_an_analysing_pass_does_not(self):
        ratios = {'exp_time_x2': [2.07, 2.05, 2.01], 'exp_rel_x2': [2.0, 2.0, 2.0],
                  'exp_rel_x2_analysed': [1.0, 1.01, 0.99]}
        self.assertEqual(hwsuite.exposure_verdict(ratios), (False, ['exp_rel_x2', 'exp_time_x2']))

    def test_exp_rel_short_of_doubling_does_not(self):
        """The RPS 10M's first-round x1.44, if it holds."""
        ratios = {'exp_time_x2': [2.07, 2.05, 2.01], 'exp_rel_x2': [1.45, 1.44, 1.43],
                  'exp_rel_x2_analysed': [1.45, 1.44, 1.43]}
        self.assertEqual(hwsuite.exposure_verdict(ratios), (False, ['exp_time_x2']))

    def test_level_ratio_ignores_a_black_level_shift(self):
        rng = numpy.random.default_rng(0)
        base = rng.integers(1000, 20000, (64, 64, 3)).astype(numpy.uint16)
        base = numpy.repeat(numpy.repeat(base[::8, ::8], 8, 0), 8, 1)
        other = (2 * base.astype(numpy.int64) - 348).astype(numpy.uint16)
        self.assertEqual(hwsuite.level_ratio(base, other), [2.0, 2.0, 2.0])

    def test_level_ratio_ignores_pixels_that_could_clip(self):
        base = numpy.array([[[1000] * 3, [60000] * 3]], numpy.uint16)
        other = numpy.array([[[2000] * 3, [65535] * 3]], numpy.uint16)
        self.assertEqual(hwsuite.level_ratio(base, other), [2.0, 2.0, 2.0])

    def test_focus_sweep_spans_the_range(self):
        self.assertEqual(hwsuite.focus_sweep(1, 30), [1, 5, 9, 13, 18, 22, 26, 30])
        self.assertEqual(hwsuite.focus_sweep(1, 5), [1, 2, 3, 4, 5])


class Characterize(SuiteRun):

    def test_a_long_strip_runs_every_stage(self):
        status, stages, s, summary = self.run_suite()
        self.assertEqual(status, 0)
        self.assertEqual({k: v['status'] for k, v in stages.items()},
                         {**{k: 'ok' for k in hwsuite.STAGE_NAMES}, 'walk': 'partial',
                          'framing': 'partial', 'drift': 'skipped'})
        self.assertEqual(summary['suggested_capabilities'],
                         {'film_transport': True, 'focus': True})

    def test_the_exposure_stage_names_the_live_control(self):
        _, stages, _, _ = self.run_suite()
        exposure = stages['exposure']
        self.assertTrue(exposure['matches_library'])
        self.assertEqual(exposure['ratios']['exp_time_x2'], [2.0, 2.0, 2.0])
        self.assertEqual(exposure['ratios']['base_again'], [1.0, 1.0, 1.0])
        self.assertEqual(exposure['response'], {'exp_time_x2': 'doubles',
                                                'exp_rel_x2': 'doubles',
                                                'gain_plus8': 'no effect',
                                                'exp_rel_x2_analysed': 'doubles'})

    def test_the_exposure_stage_leaves_no_option_assigned(self):
        _, _, s, _ = self.run_suite()
        assigned = {p.opt.name for p in s.params.table if p.assigned} - {
            'resolution', 'tl_x', 'tl_y', 'br_x', 'br_y', 'focus'}
        self.assertEqual(assigned, set())

    def test_later_stages_scan_at_the_sweeps_sharpest(self):
        _, stages, s, _ = self.run_suite()
        self.assertEqual(stages['autofocus']['sweep_best'], SHARPEST_FOCUS)
        walk_scans = [e for e in stages['walk']['events'] if e['op'] == 'scan']
        self.assertTrue(all(e['focus'] == SHARPEST_FOCUS for e in walk_scans))

    def test_the_pick_is_checked_at_the_highest_resolution(self):
        _, stages, _, summary = self.run_suite(only={'identity', 'autofocus'})
        af = stages['autofocus']
        self.assertEqual(af['check_dpi'], PROSCAN_4000.max_resolution_x)
        self.assertEqual(sorted(map(int, af['check_sharpness'])), [9, 11, 13, 15, 17])
        self.assertEqual((af['check_best'], af['pick_ok']), (SHARPEST_FOCUS, True))
        self.assertTrue(summary['findings']['autofocus_pick_ok'])

    def test_a_soft_pick_fails_the_check_and_is_not_used(self):
        _, stages, s, summary = self.run_suite(
            only={'identity', 'autofocus'},
            autofocus=lambda s: fake_autofocus(s, peak=SHARPEST_FOCUS + 6))
        af = stages['autofocus']
        self.assertIn(str(SHARPEST_FOCUS), af['check_sharpness'], 'the coarse best is checked too')
        self.assertFalse(af['pick_ok'])
        self.assertFalse(summary['findings']['autofocus_pick_ok'])
        self.assertEqual(af['focus_used_from_here'], SHARPEST_FOCUS)
        self.assertEqual(s.params['focus'].value, SHARPEST_FOCUS)

    def test_the_16bit_pass_is_compared_with_the_8bit_one(self):
        _, stages, _, _ = self.run_suite(only={'identity', 'autofocus'})
        result = stages['autofocus']['autofocus_16bit']
        self.assertEqual((result['8']['dtype'], result['16']['dtype']), ('uint8', 'uint16'))
        self.assertLess(result['16']['noise_rel'], result['8']['noise_rel'])
        self.assertEqual(pieusb.scanner.AUTOFOCUS_COLOR_DEPTH, 8)

    def test_a_refused_16bit_pass_is_recorded_not_fatal(self):
        with self.assertLogs('exercise', level='ERROR'):
            status, stages, _, _ = self.run_suite(
                only={'identity', 'autofocus'},
                autofocus=lambda s: fake_autofocus(s, refuse_16bit=True))
        self.assertEqual(status, 0)
        self.assertEqual(stages['autofocus']['status'], 'ok')
        self.assertIn('CheckCondition', stages['autofocus']['autofocus_16bit']['16']['error'])
        self.assertEqual(pieusb.scanner.AUTOFOCUS_COLOR_DEPTH, 8)

    def test_the_walk_scans_every_frame_three_times(self):
        _, stages, _, _ = self.run_suite()
        scans = [e['name'] for e in stages['walk']['events'] if e['op'] == 'scan']
        self.assertEqual(scans, ['home', 'plus', 'minus'] * 8 + ['after_end_of_strip'])

    def test_the_end_of_strip_is_the_librarys_and_is_checked(self):
        _, stages, _, summary = self.run_suite()
        last = [e for e in stages['walk']['events'] if e['op'] == 'next'][-1]
        self.assertEqual((last['moved'], last['end_of_strip']), (0, True))
        ends = summary['findings']['end_of_strip']
        self.assertEqual({e['stage'] for e in ends}, {'walk', 'framing', 'roll'})
        self.assertTrue(all(e['frame'] == 7 and e['film_stayed'] for e in ends))
        self.assertTrue(all(e['state_after'] is not None for e in ends))

    def test_the_centring_offset_is_cyberviews_70_steps(self):
        _, stages, _, _ = self.run_suite()
        mm = stages['identity']['derived']['centring_offset_mm']
        self.assertEqual(int(mm * hwsuite.OFFSET_STEPS_PER_MM), 70)

    def test_every_offset_is_returned(self):
        strip = Strip(8)
        self.run_suite(strip)
        self.assertAlmostEqual(sum(m[1] for m in strip.moves if m[0] == 'offset'), 0)

    def test_the_rewind_reaches_frame_0_before_the_walk(self):
        _, stages, _, _ = self.run_suite()
        walk_start = next(e for e in stages['walk']['events'] if 'frame' in e)
        self.assertEqual(walk_start['frame'], 0)

    def test_a_short_strip_is_partial_not_fatal(self):
        status, stages, _, _ = self.run_suite(Strip(2))
        self.assertEqual(status, 0)
        self.assertEqual(stages['redetect']['status'], 'partial')
        self.assertEqual(stages['recovery']['status'], 'ok')

    def test_redetect_undoes_its_shift_at_the_end_of_the_strip(self):
        strip = Strip(2)
        self.run_suite(strip, motors=frozenset({'film_transport'}))
        self.assertAlmostEqual(sum(m[1] for m in strip.moves if m[0] == 'offset'), 0)

    def test_redetect_returns_to_where_it_started(self):
        _, stages, _, _ = self.run_suite()
        scans = [e['name'] for e in stages['redetect']['events'] if e['op'] == 'scan']
        self.assertEqual(scans, ['home', 'after_offset+3', 'restored',
                                 'after_offset-3', 'restored'])
        start = next(e for e in stages['redetect']['events'] if 'frame' in e)
        self.assertEqual(stages['rewind']['events'][0]['frame'], start['frame'])

    def test_a_failed_move_stops_the_suite(self):
        status, stages, _, _ = self.run_suite(Strip(8, fail_on='next'))
        self.assertEqual(status, 1)
        self.assertEqual(stages['stepping']['status'], 'halted')
        self.assertNotIn('redetect', stages, 'nothing moves after a failed command')

    def test_rewinds_use_byte3_0_when_only_that_moved_the_counter(self):
        strip = Strip(8)
        real = strip.slide

        def only_byte3_0(action, value, byte3=0):
            if byte3 == 1:
                strip.moves.append(('prev', value, byte3))
                return
            real(action, value, byte3)

        strip.slide = only_byte3_0
        _, stages, _, _ = self.run_suite(strip)
        self.assertEqual(stages['stepping']['prev_byte3_used_from_here'], 0)
        rewinds = [m for m in strip.moves if m[0] == 'prev'][2:]
        self.assertTrue(rewinds and all(m[2] == 0 for m in rewinds))


class SlippingCounter(Film):
    '''A strip whose counter stops at the last frame while NEXT still moves the film.'''

    def slide(self, action, value, byte3=0):
        if action == SLIDE_NEXT and self.frame == self.frames - 1:
            self.moves.append(('next', value))
            self.position += value * self.PITCH
            return
        super().slide(action, value, byte3)


class Framing(SuiteRun):

    def test_film_moving_past_a_reported_end_stops_the_suite(self):
        status, stages, _, summary = self.run_suite(
            SlippingCounter(6), only={'identity', 'framing'})
        self.assertEqual(status, 1)
        self.assertEqual(stages['framing']['status'], 'halted')
        [end] = summary['findings']['end_of_strip']
        self.assertFalse(end['film_stayed'])
        self.assertGreater(end['difference'], hwsuite.END_OF_STRIP_MOVED)

    def framing(self, film, *stages):
        _, stages, _, _ = self.run_suite(film, only={'identity', *stages, 'framing'})
        return stages

    def test_every_frame_is_centred_and_the_strip_end_stops_the_loop(self):
        film = Film(6)
        framing = self.framing(film)['framing']
        self.assertEqual(framing['status'], 'partial')
        self.assertEqual(len(framing['frames']), 6)
        first, *rest = framing['frames']
        self.assertEqual(first['result'], 'corrected', 'frame 0 starts 2 mm off')
        # Without stepsize the first move uses the model's measured step (1.10).
        self.assertLess(abs(first['residual_mm']), 0.1 * abs(first['shift_mm']) + 0.2)
        for f in rest:
            self.assertIn(f['result'], ('corrected', 'centred'))
            self.assertLess(abs(f.get('residual_mm', f['shift_mm'])), 0.25)

    def test_the_first_correction_teaches_the_offset_scale(self):
        """Roughly: a 2 mm correction measured to a pixel or two."""
        film = Film(4, scale=1.09)
        framing = self.framing(film)['framing']
        self.assertAlmostEqual(framing['offset_scale'], 1.09, delta=0.12)

    def test_a_reversed_offset_is_learnt_too(self):
        film = Film(4, scale=-1.0)
        framing = self.framing(film)['framing']
        self.assertAlmostEqual(framing['offset_scale'], -1.0, delta=0.05)
        later = [f for f in framing['frames'][1:] if f['result'] == 'corrected']
        self.assertTrue(all(abs(f['residual_mm']) < 0.25 for f in later))

    def test_a_reversed_model_step_is_used_from_the_first_move(self):
        """An RPS 10M's offsets move the image towards -x; its measured step
        steers the first correction the right way."""
        film = Film(4, scale=-0.93)
        _, stages, _, _ = self.run_suite(film, model=0x0048, only={'identity', 'framing'})
        first = stages['framing']['frames'][0]
        self.assertEqual(first['result'], 'corrected')
        self.assertLess(abs(first['residual_mm']), 0.25)

    def test_stepsize_measures_the_scale_before_framing_uses_it(self):
        film = Film(4, scale=1.09)
        stages = self.framing(film, 'stepsize')
        self.assertAlmostEqual(stages['stepsize']['image_mm_per_commanded_mm'], 1.09, delta=0.03)
        returns = [m['return_missed_mm'] for m in stages['stepsize']['moves']]
        self.assertTrue(all(abs(r) < 0.1 for r in returns))
        self.assertLess(stages['framing']['residual_mm']['max'], 0.25)

    def test_the_frame_width_is_learnt_when_both_gaps_show(self):
        film = Film(4, start_off_mm=0.0)
        with mock.patch.object(hwsuite, 'PREVIEW_DPI', 600):
            framing = self.framing(film)['framing']
        self.assertAlmostEqual(framing['frame_mm'], Film.FRAME, delta=0.2)


class Load(SuiteRun):

    def test_the_first_read_and_what_the_gate_shows_are_recorded(self):
        film = Film(4)
        film.raw = bytes([0x50, 0, 0, 0, 0x1e, 0, 0x0d, 0x20])
        _, stages, _, summary = self.run_suite(film, only={'identity', 'first_frame'})
        load = summary['findings']['load']
        self.assertEqual((load['counter'], load['power_up_read'], load['content']),
                         (0, True, 'picture'))
        self.assertTrue(load['filmstrip_carrier'])

    def test_film_past_its_end_reads_as_clear(self):
        film = Film(1)
        film.position += 60
        _, _, _, summary = self.run_suite(film, only={'identity', 'first_frame'})
        self.assertEqual(summary['findings']['load']['content'], 'clear')


class Calibration(SuiteRun):

    def test_a_reference_that_serves_every_resolution(self):
        _, stages, _, summary = self.run_suite(only={'identity', 'calibration'})
        cal = stages['calibration']
        self.assertEqual((cal['reused'], cal['matches']), (True, True))
        self.assertEqual(cal['dpi'], PROSCAN_4000.max_resolution_x)
        self.assertTrue(summary['findings']['shading_shared_across_resolution'])

    def test_a_reference_the_scanner_will_not_reuse_at_another_resolution(self):
        inner = fake_pass_for('both')

        def recalibrates(s, args, prefix):
            status, r = inner(s, args, prefix)
            return status, dataclasses.replace(r, shading_acquired=True)

        with mock.patch(f'{__name__}.fake_pass_for', lambda honours: recalibrates):
            _, stages, _, summary = self.run_suite(only={'identity', 'calibration'})
        self.assertFalse(stages['calibration']['reused'])
        self.assertFalse(summary['findings']['shading_shared_across_resolution'])


class Drift(SuiteRun):

    def test_runs_only_when_asked_and_samples_each_interval(self):
        _, stages, _, _ = self.run_suite(only={'identity', 'drift'})
        self.assertEqual(stages['drift']['status'], 'skipped')
        _, stages, _, summary = self.run_suite(only={'identity', 'drift'},
                                               enabled=frozenset({'drift'}), soak_minutes=3)
        drift = stages['drift']
        self.assertEqual(len(drift['samples']), 3)
        self.assertEqual(len(self.slept), 3)
        scans = [e for e in drift['events'] if e['op'] == 'scan']
        self.assertEqual([e['shading_acquired'] for e in scans], [True, False, False, False, True])
        self.assertEqual(summary['findings']['drift']['worst_reused'], 0.0)


class Reopen(SuiteRun):

    def test_a_counter_and_film_that_survive(self):
        film = Film(4)
        _, stages, s, summary = self.run_suite(film, only={'identity', 'reopen'})
        result = summary['findings']['reopen']
        self.assertEqual((result['counter_before'], result['counter_after']), (1, 1))
        self.assertTrue(result['counter_kept'] and result['film_stayed'])
        self.assertEqual(len(self.reopened), 1)
        self.assertIsNot(s, self.reopened[0], 'later stages use the new Scanner')

    def test_options_set_before_the_reopen_carry_over(self):
        _, _, s, _ = self.run_suite(Strip(4), only={'identity', 'autofocus', 'reopen'})
        self.assertEqual(s.params['focus'].value, SHARPEST_FOCUS)
        self.assertTrue(s.params['focus'].assigned)

    def test_a_counter_reset_and_a_moved_film_are_reported(self):
        def reset(film):
            film.frame = 0
            film.position += 3.0

        _, _, _, summary = self.run_suite(Film(4), only={'identity', 'reopen'},
                                          on_reopen=reset)
        result = summary['findings']['reopen']
        self.assertEqual((result['counter_kept'], result['counter_after']), (False, 0))
        self.assertFalse(result['film_stayed'])
        self.assertAlmostEqual(abs(result['film_shift_mm']), 3.0, delta=0.3)


class PrevMany(SuiteRun):

    ONLY = {'identity', 'rewind', 'walk', 'recovery', 'prev_many'}

    def test_one_rewind_past_the_start_lands_on_the_first_frame(self):
        film = Film(4)
        _, stages, _, summary = self.run_suite(film, only=self.ONLY)
        result = summary['findings']['prev_past_first']
        self.assertEqual((result['from'], result['asked'], result['op']), (2, 3, 'rewind'))
        self.assertEqual(result['counter_after'], 0)
        self.assertTrue(result['at_first_frame'])
        self.assertIn(('prev', 3, 1), film.moves, "sent as the library's rewind()")

    def test_a_strip_that_leaves_stops_the_suite(self):
        status, stages, _, summary = self.run_suite(
            Strip(4, leaves_on_overshoot=True), only=self.ONLY | {'framing'})
        self.assertEqual(status, 1)
        self.assertEqual(stages['prev_many']['status'], 'halted')
        self.assertNotIn('framing', stages)
        self.assertFalse(summary['findings']['prev_past_first']['medium_present'])


class Roll(SuiteRun):

    def test_every_frame_is_placed_and_scanned_once(self):
        film = Film(5)
        status, stages, _, summary = self.run_suite(film, only={'identity', 'roll'},
                                                    roll_dpi=1200)
        self.assertEqual((status, stages['roll']['status']), (0, 'ok'))
        result = summary['findings']['roll']
        self.assertEqual(result['frames'], 5)
        self.assertEqual(result['unplaced'], [])
        later = [f for f in stages['roll']['frames'][1:] if 'residual_mm' in f]
        self.assertTrue(all(abs(f['residual_mm']) < 0.25 for f in later))
        frames = [e for e in stages['roll']['events']
                  if e['op'] == 'scan' and e['name'] == 'frame']
        self.assertEqual(len(frames), 5)
        self.assertTrue(all(e['resolution'] == 1200 for e in frames))
        self.assertTrue(all(e['file'].endswith('_thumb.npy') for e in frames))
        self.assertTrue((self.out / frames[0]['file']).exists())

    def test_focus_exposure_and_shading_are_taken_once(self):
        _, stages, _, summary = self.run_suite(Film(4), only={'identity', 'roll'},
                                               roll_dpi=600)
        result = summary['findings']['roll']
        self.assertEqual(result['exposure_lock'], list(METERED))
        self.assertEqual(result['focus']['focus'], SHARPEST_FOCUS)
        self.assertEqual(result['shading_analyses'], 0, 'every pass may reuse a reference')
        meters = [e for e in stages['roll']['events'] if e['op'] == 'scan' and e['name'] == 'meter']
        self.assertEqual(len(meters), 1)

    def test_back_at_the_first_frame_it_is_where_it_was(self):
        _, _, _, summary = self.run_suite(Film(4), only={'identity', 'roll'}, roll_dpi=600)
        result = summary['findings']['roll']
        self.assertLess(abs(result['return_shift_mm']), hwsuite.SAME_PLACE_MM)
        self.assertTrue(all(abs(v - 1) <= hwsuite.SHADING_MATCH for v in result['return_level']))


class Regression(SuiteRun):

    def test_the_10t_gets_scans_and_no_slide(self):
        strip = Strip(8)
        status, stages, s, summary = self.run_suite(
            strip, PROSCAN_10T, 0x0047, profile='regression', motors=frozenset())
        self.assertEqual(status, 0)
        ran = [k for k, v in stages.items() if v['status'] == 'ok']
        self.assertEqual(ran, ['identity', 'default_scan', 'exposure', 'calibration'])
        self.assertEqual(stages['first_frame']['note'], 'no film transport on this model')
        self.assertEqual(strip.moves, [])
        self.assertEqual(s.dev.of(transport.SCSI_SLIDE), [])
        self.assertTrue(stages['exposure']['matches_library'])

    def test_a_model_where_exp_rel_does_nothing_is_reported(self):
        """The library's exposure would not work there."""
        _, stages, _, summary = self.run_suite(profile='regression', honours='exp_time')
        self.assertFalse(stages['exposure']['matches_library'])
        self.assertEqual(stages['exposure']['works'], ['exp_time_x2'])
        self.assertFalse(summary['findings']['exp_rel_works'])


if __name__ == '__main__':
    unittest.main()
