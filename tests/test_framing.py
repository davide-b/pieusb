#!/usr/bin/env python3
'''Gap finding on filmstrip scans, without hardware.

    python tests/test_framing.py

`reference/framing_scans.npz` holds 300 dpi scans recorded on a ProScan 4000 and
an RPS 10M, every fourth row kept and the low 6 bits dropped, which leaves what
find_bands sees unchanged. Expected band positions were checked by eye against
the full scans.
'''

import sys
import unittest
from pathlib import Path

import numpy

sys.path.insert(0, str(Path(__file__).resolve().parent))

from support import REFERENCE_DIR

from pieusb.framing import FRAME_MM, BandKind, Polarity, find_bands, locate_frame

PX_PER_MM = 300 / 25.4
TOLERANCE_PX = 2


def scan(name):
    with numpy.load(REFERENCE_DIR / 'framing_scans.npz') as scans:
        return scans[name]


def summary(bands):
    return [(b.kind, b.start, b.stop) for b in bands]


class RecordedScans(unittest.TestCase):

    def assertBands(self, name, want):
        got = find_bands(scan(name), PX_PER_MM)
        self.assertEqual([b.kind for b in got], [w[0] for w in want], summary(got))
        for band, (_, start, stop) in zip(got, want):
            self.assertLessEqual(abs(band.start - start), TOLERANCE_PX, summary(got))
            self.assertLessEqual(abs(band.stop - stop), TOLERANCE_PX, summary(got))
        return got

    def test_a_gap_at_the_right_edge(self):
        """The ProScan 4000's own frame position after a NEXT that followed an offset."""
        gap, = self.assertBands('ps4000_gap_right', [(BandKind.GAP, 400, 425)])
        self.assertEqual(gap.polarity, Polarity.BRIGHT)
        self.assertGreater(gap.confidence, 0.9)

    def test_a_gap_at_the_left_edge(self):
        self.assertBands('ps4000_gap_left', [(BandKind.GAP, 59, 77)])

    def test_a_gap_against_deep_shadows_is_found_with_less_confidence(self):
        """Shadows are thin on a negative, nearly the base's level, so fewer rows
        step at the edge."""
        gap, = self.assertBands('ps4000_gap_shadows', [(BandKind.GAP, 392, 408)])
        self.assertLess(gap.confidence, 0.6)

    def test_a_flat_patch_of_scene_is_not_a_gap(self):
        self.assertBands('ps4000_flat_patch', [(BandKind.GAP, 395, 412)])

    def test_a_gap_cut_by_the_image_edge(self):
        gap, = self.assertBands('ps4000_gap_clipped', [(BandKind.GAP, 417, 428)])
        self.assertTrue(gap.clipped_stop)
        self.assertIsNone(gap.edge_stop)

    def test_the_rps_10m_sliver_at_the_left_edge(self):
        """0.6 mm of gap, all an RPS 10M shows at its own frame position."""
        for name, stop in (('rps10m_sliver', 7), ('rps10m_sliver_next', 9)):
            with self.subTest(name):
                gap, = self.assertBands(name, [(BandKind.GAP, 0, stop)])
                self.assertTrue(gap.clipped_start)

    def test_clear_film_past_the_strip_end_is_split_from_the_base(self):
        self.assertBands('ps4000_strip_end', [(BandKind.CLEAR, 0, 52), (BandKind.GAP, 57, 70)])
        self.assertBands('ps4000_strip_start', [(BandKind.GAP, 413, 420),
                                                (BandKind.CLEAR, 422, 428)])

    def test_an_unexposed_frame_is_blank(self):
        self.assertBands('ps4000_blank_frame', [(BandKind.BLANK, 0, 428)])

    def test_a_leader_is_clear_then_blank(self):
        """Past the strip's end: no film, a dense black leader, then fogged base."""
        bands = self.assertBands('ps4000_leader', [(BandKind.CLEAR, 0, 76),
                                                   (BandKind.BLANK, 79, 199),
                                                   (BandKind.BLANK, 207, 428)])
        self.assertEqual(bands[1].polarity, Polarity.DARK)

    def test_a_centred_frame_shows_its_gaps_as_slivers_of_the_known_base(self):
        """A ProScan 4000 frame after one centring offset: a column or two of base at
        an edge counts once the strip's base level is known, and only then."""
        self.assertEqual(find_bands(scan('ps4000_centred_sliver'), PX_PER_MM), [])
        gap, = find_bands(scan('ps4000_centred_sliver'), PX_PER_MM, base_level=0.178)
        self.assertEqual((gap.start, gap.stop, gap.clipped_stop), (427, 428, True))
        both = find_bands(scan('ps4000_centred_both'), PX_PER_MM, base_level=0.178)
        self.assertEqual([(b.start, b.stop) for b in both], [(0, 2), (425, 428)])

    def test_a_sliver_of_another_level_is_not_base(self):
        self.assertEqual(find_bands(scan('ps4000_centred_sliver'), PX_PER_MM,
                                    base_level=0.08), [])

    def test_a_frame_filling_the_window_shows_no_band(self):
        self.assertEqual(find_bands(scan('ps4000_centred_no_gap'), PX_PER_MM,
                                    base_level=0.178), [])

    def test_a_polarity_that_does_not_match_drops_the_gap(self):
        self.assertEqual(find_bands(scan('ps4000_gap_right'), PX_PER_MM, Polarity.DARK), [])


def synthetic(gap_level, picture=(0.05, 0.6), width=428, height=287, gap=(200, 220)):
    '''A textured picture whose columns span `picture` in level, with one
    uniform gap band, as 16-bit RGB.'''
    rng = numpy.random.default_rng(1)
    lo, hi = picture
    columns = rng.permutation(numpy.linspace(0, 1, width))
    lum = lo + (hi - lo) * (0.7 * columns + 0.3 * rng.random((height, width)))
    lum[:, gap[0]:gap[1]] = gap_level * (1 + 0.01 * rng.standard_normal((height, gap[1] - gap[0])))
    return (numpy.repeat(lum[:, :, None], 3, axis=2) * 65535).clip(0, 65535).astype(numpy.uint16)


class Synthetic(unittest.TestCase):

    def test_a_dark_gap_on_a_slide(self):
        gap, = find_bands(synthetic(0.003), PX_PER_MM)
        self.assertEqual((gap.kind, gap.polarity, gap.start, gap.stop),
                         (BandKind.GAP, Polarity.DARK, 200, 220))

    def test_a_neutral_bright_gap_is_clear_film(self):
        gap, = find_bands(synthetic(0.8), PX_PER_MM)
        self.assertEqual(gap.kind, BandKind.CLEAR)

    def test_a_band_that_stands_out_neither_way_is_dropped(self):
        self.assertEqual(find_bands(synthetic(0.3), PX_PER_MM), [])

    def test_a_band_too_narrow_is_noise(self):
        self.assertEqual(find_bands(synthetic(0.003, gap=(200, 202)), PX_PER_MM), [])

    def test_a_band_wider_than_a_gap_is_blank(self):
        band, = find_bands(synthetic(0.003, gap=(100, 200)), PX_PER_MM)
        self.assertEqual(band.kind, BandKind.BLANK)


def two_gaps(width=860, gaps=((0, 5), (851, 860)), height=200):
    '''A picture between two bright gaps, the frame filling the window.'''
    rng = numpy.random.default_rng(2)
    columns = rng.permutation(numpy.linspace(0.02, 0.12, width))
    lum = columns * (0.7 + 0.6 * rng.random((height, width)))
    for a, b in gaps:
        lum[:, a:b] = 0.18 * (1 + 0.01 * rng.standard_normal((height, b - a)))
    return (numpy.repeat(lum[:, :, None], 3, axis=2) * 65535).astype(numpy.uint16)


class Placement(unittest.TestCase):

    def place(self, rgb, px_per_mm=PX_PER_MM, **kwargs):
        return locate_frame(find_bands(rgb, px_per_mm), rgb.shape[1], px_per_mm, **kwargs)

    def test_a_gap_at_the_right_places_the_frame_one_width_to_its_left(self):
        frame = self.place(scan('ps4000_gap_right'))
        self.assertEqual((frame.measured_start, frame.measured_stop), (False, True))
        self.assertAlmostEqual(frame.stop, 400, delta=TOLERANCE_PX)
        self.assertAlmostEqual(frame.start, 400 - FRAME_MM * PX_PER_MM, delta=TOLERANCE_PX)
        self.assertAlmostEqual(frame.shift_mm, 2.25, delta=0.2)
        self.assertGreater(frame.overhang_mm[0], 2)

    def test_a_gap_at_the_left_measures_the_start(self):
        frame = self.place(scan('ps4000_gap_left'))
        self.assertEqual((frame.measured_start, frame.measured_stop), (True, False))
        self.assertAlmostEqual(frame.start, 77, delta=TOLERANCE_PX)
        self.assertAlmostEqual(frame.shift_mm, -6.4, delta=0.2)

    def test_the_rps_10m_frame_sits_just_right_of_centre(self):
        frame = self.place(scan('rps10m_sliver'))
        self.assertTrue(frame.measured_start)
        self.assertAlmostEqual(frame.shift_mm, -0.35, delta=0.2)

    def test_the_strip_ends_bound_the_frame(self):
        self.assertAlmostEqual(self.place(scan('ps4000_strip_start')).stop, 413, delta=TOLERANCE_PX)
        self.assertAlmostEqual(self.place(scan('ps4000_strip_end')).start, 70, delta=TOLERANCE_PX)

    def test_no_frame_on_a_blank_frame_or_a_leader(self):
        self.assertIsNone(self.place(scan('ps4000_blank_frame')))
        self.assertIsNone(self.place(scan('ps4000_leader')))

    def test_both_gaps_in_view_measure_the_width(self):
        """The ProScan 4000's window at 600 dpi: a gap at each edge, 35.8 mm apart."""
        frame = self.place(two_gaps(), px_per_mm=600 / 25.4)
        self.assertEqual((frame.measured_start, frame.measured_stop), (True, True))
        self.assertAlmostEqual(frame.width_mm, 846 / (600 / 25.4), places=3)
        self.assertAlmostEqual(frame.shift_mm, (430 - 428) / (600 / 25.4), places=3)

    def test_without_a_gap_the_frame_is_assumed_centred(self):
        frame = self.place(two_gaps(gaps=()), px_per_mm=600 / 25.4)
        self.assertFalse(frame.measured)
        self.assertEqual((frame.confidence, frame.shift_mm), (0.0, 0.0))

    def test_a_centred_frame_measures_its_width_from_two_slivers(self):
        frame = locate_frame(find_bands(scan('ps4000_centred_both'), PX_PER_MM, base_level=0.178),
                             428, PX_PER_MM)
        self.assertEqual((frame.measured_start, frame.measured_stop), (True, True))
        self.assertAlmostEqual(frame.width_mm, 35.8, delta=0.1)
        self.assertLess(abs(frame.shift_mm), 0.1)

    def test_the_frame_width_places_the_unseen_edge(self):
        frame = self.place(scan('ps4000_gap_right'), frame_mm=35.6)
        self.assertAlmostEqual(frame.start, 400 - 35.6 * PX_PER_MM, delta=TOLERANCE_PX)


if __name__ == '__main__':
    unittest.main()
