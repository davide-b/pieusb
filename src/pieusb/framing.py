'''
Finding where frames are on a filmstrip, from a scan of it.

Pure numpy over a finished RGB image: no device access, no state. The film runs
along the image's x axis (ProScan 4000, RPS 10M), so everything here works on
columns.

What separates two frames is a band a millimetre or two wide that is uniform
from top to bottom, because nothing was exposed there: unexposed base, brighter
than the picture on a negative and darker on a slide. Past the end of the
strip, or where the film was never exposed (a leader, a blank frame), the
uniform region is wider than any gap. Measured on a ProScan 4000 at 300 dpi:
gap columns have an interquartile spread down their length of 0.06-0.09 in
log2 brightness, picture columns 0.17 and up.

The scan window of these scanners is barely one frame wide, so a gap is often
cut off by the image edge -- on an RPS 10M at its own frame position only a
0.5 mm sliver shows. A cut band still marks the frame boundary: its edge that
faces the picture.
'''

from dataclasses import dataclass
from enum import StrEnum

import numpy

# Rows this close to the top or bottom are left out: film holders and the
# strip's own edges sit there.
ROW_MARGIN = 0.08
# A column is uniform if its interquartile spread in log2 brightness is below
# this, or below UNIFORM_RELATIVE x the image's median column spread.
UNIFORM_SPREAD = 0.15
UNIFORM_RELATIVE = 0.25
# Darker than this, as a fraction of full scale, log noise swamps the spread;
# those columns are judged on their linear spread against the image's range.
DARK_LEVEL = 1 / 64
DARK_SPREAD = 0.02
# Uniform runs separated by no more than this many columns are one band, and a
# band splits where its level steps by more than LEVEL_STEP in log2 between
# neighbouring columns: base running into clear film past the strip's end.
BRIDGE_PX = 2
LEVEL_STEP = 0.5
# A band narrower than this is noise; one cut off by the image edge needs only
# BAND_MIN_CLIPPED_PX, since it may show as a sliver.
BAND_MIN_MM = 0.3
BAND_MIN_CLIPPED_PX = 3
# Wider than this, a uniform band is not a gap between frames.
GAP_MAX_MM = 3.0
# A gap is the film's least or most dense region: at or above this quantile of
# the picture's column levels (negatives), or at or below 1 minus it (slides).
POLARITY_QUANTILE = 0.9
# Clear film -- no film at all -- is neutral and bright.
CLEAR_LEVEL = 0.5
NEUTRAL_RATIO = 1.3
# A row counts towards an edge where the picture beside the band differs from
# the band by more than this many of the band's own spreads.
EDGE_CONTRAST = 4.0
EDGE_SAMPLE_PX = 3
# A GAP needs at least one edge this strong; a flat patch of scene has none.
EDGE_MIN = 0.2


class Polarity(StrEnum):
    '''What unexposed film looks like in the scan.'''
    BRIGHT = 'bright'   # a negative: unexposed base is the thinnest film
    DARK = 'dark'       # a slide: unexposed film is opaque


class BandKind(StrEnum):
    GAP = 'gap'         # between two frames
    BLANK = 'blank'     # wider than a gap: a leader, an unexposed frame
    CLEAR = 'clear'     # no film


@dataclass(frozen=True)
class Band:
    '''A uniform vertical band, in image columns.

    `start` is its first column and `stop` one past its last. `clipped_start`
    and `clipped_stop` say it runs into the image edge on that side, so its
    extent there is unknown. `level` is its median brightness as a fraction of
    full scale. `edge_start` and `edge_stop` are the fraction of rows across
    which the picture beside it steps away from it: near 1 for a gap against a
    picture, lower where a scene's own shadows or highlights match the base; None
    on a clipped side or where nothing lies beyond. `confidence` combines them
    into how sure a GAP is to be one, from 0 to 1.
    '''
    start: int
    stop: int
    kind: BandKind
    polarity: Polarity | None
    level: float
    clipped_start: bool
    clipped_stop: bool
    edge_start: float | None
    edge_stop: float | None
    confidence: float

    @property
    def width(self) -> int:
        return self.stop - self.start


@dataclass(frozen=True)
class _Columns:
    level: numpy.ndarray        # median linear brightness, fraction of full scale
    spread: numpy.ndarray       # IQR of log2 brightness down the column
    lin_spread: numpy.ndarray   # IQR of linear brightness, fraction of full scale
    rows: numpy.ndarray         # the brightness used, rows trimmed: (h, w)
    rgb: numpy.ndarray          # per-column median RGB, fraction of full scale: (w, 3)


def _columns(rgb: numpy.ndarray) -> _Columns:
    full = float(numpy.iinfo(rgb.dtype).max) if rgb.dtype.kind in 'ui' else 1.0
    h = rgb.shape[0]
    margin = int(h * ROW_MARGIN)
    body = rgb[margin:h - margin] if h - 2 * margin >= 4 else rgb
    planes = body[:, :, :3].astype(numpy.float64) / full
    lum = planes.mean(axis=2)
    log = numpy.log2(numpy.maximum(lum, 1 / 4096))
    q75, q25 = numpy.percentile(log, [75, 25], axis=0)
    l75, l25 = numpy.percentile(lum, [75, 25], axis=0)
    return _Columns(level=numpy.median(lum, axis=0), spread=q75 - q25,
                    lin_spread=l75 - l25, rows=lum,
                    rgb=numpy.median(planes, axis=0))


def _uniform(cols: _Columns) -> numpy.ndarray:
    limit = max(UNIFORM_SPREAD, UNIFORM_RELATIVE * float(numpy.median(cols.spread)))
    image_range = float(numpy.percentile(cols.level, 99) - numpy.percentile(cols.level, 1))
    dark = cols.level < DARK_LEVEL
    uniform = cols.spread < limit
    uniform[dark] = cols.lin_spread[dark] < DARK_SPREAD * max(image_range, 1 / 256)
    return uniform


def _runs(mask: numpy.ndarray) -> list[tuple[int, int]]:
    '''[start, stop) of each run of True, joining runs BRIDGE_PX or less apart.'''
    edges = numpy.flatnonzero(numpy.diff(numpy.concatenate(([0], mask.astype(int), [0]))))
    runs = list(zip(edges[::2].tolist(), edges[1::2].tolist()))
    joined: list[tuple[int, int]] = []
    for start, stop in runs:
        if joined and start - joined[-1][1] <= BRIDGE_PX:
            joined[-1] = (joined[-1][0], stop)
        else:
            joined.append((start, stop))
    return joined


def _split(cols: _Columns, start: int, stop: int) -> list[tuple[int, int]]:
    '''[start, stop) pieces of a band, cut where its level steps.'''
    log = numpy.log2(numpy.maximum(cols.level[start:stop], 1 / 4096))
    cuts = numpy.flatnonzero(numpy.abs(numpy.diff(log)) > LEVEL_STEP) + 1
    bounds = [0, *cuts.tolist(), stop - start]
    return [(start + a, start + b) for a, b in zip(bounds, bounds[1:]) if b > a]


def _edge(cols: _Columns, start: int, stop: int, side: str) -> float | None:
    '''Fraction of rows where the picture just outside the band steps away from it.'''
    w = cols.rows.shape[1]
    if side == 'start':
        outside = cols.rows[:, max(0, start - EDGE_SAMPLE_PX):start]
    else:
        outside = cols.rows[:, stop:min(w, stop + EDGE_SAMPLE_PX)]
    if outside.shape[1] == 0:
        return None
    band = cols.rows[:, start:stop]
    centre = numpy.median(band)
    spread = max(float(numpy.percentile(band, 75) - numpy.percentile(band, 25)),
                 1 / 4096)
    step = numpy.abs(outside.mean(axis=1) - centre)
    return float(numpy.mean(step > EDGE_CONTRAST * spread))


def _polarity(cols: _Columns, picture: numpy.ndarray, start: int,
              stop: int) -> Polarity | None:
    '''Which way the band stands out from the picture, if at all. `picture`
    masks the columns that are not uniform.'''
    outside = cols.level[picture]
    if outside.size < 4:
        return None
    level = float(numpy.median(cols.level[start:stop]))
    if level >= numpy.quantile(outside, POLARITY_QUANTILE):
        return Polarity.BRIGHT
    if level <= numpy.quantile(outside, 1 - POLARITY_QUANTILE):
        return Polarity.DARK
    return None


def find_bands(rgb: numpy.ndarray, px_per_mm: float,
               polarity: Polarity | None = None) -> list[Band]:
    '''Every uniform vertical band in a filmstrip scan, left to right.

    `rgb` is (height, width, 3+) with the film along x, integer or float in 0..1.
    `polarity` says which way a gap stands out when the film type is known; None
    accepts either. A narrow uniform band that stands out neither way, such as a
    flat patch of scene, is not returned.
    '''
    cols = _columns(rgb)
    w = cols.level.size
    bands = []
    uniform = _uniform(cols)
    pieces = [p for run in _runs(uniform) for p in _split(cols, *run)]
    for start, stop in pieces:
        clipped_start, clipped_stop = start == 0, stop == w
        width = stop - start
        if width < (BAND_MIN_CLIPPED_PX if clipped_start or clipped_stop
                    else BAND_MIN_MM * px_per_mm):
            continue
        level = float(numpy.median(cols.level[start:stop]))
        colour = numpy.median(cols.rgb[start:stop], axis=0)
        neutral = colour.max() <= NEUTRAL_RATIO * max(colour.min(), 1e-6)
        edge_start = None if clipped_start else _edge(cols, start, stop, 'start')
        edge_stop = None if clipped_stop else _edge(cols, start, stop, 'stop')
        if neutral and level >= CLEAR_LEVEL:
            kind, standout = BandKind.CLEAR, Polarity.BRIGHT
        elif width > GAP_MAX_MM * px_per_mm:
            kind, standout = BandKind.BLANK, _polarity(cols, ~uniform, start, stop)
        else:
            kind, standout = BandKind.GAP, _polarity(cols, ~uniform, start, stop)
            if standout is None or (polarity is not None and standout is not polarity):
                continue
        edges = [e for e in (edge_start, edge_stop) if e is not None]
        if kind is BandKind.GAP and max(edges, default=0.0) < EDGE_MIN:
            continue
        confidence = float(numpy.mean(edges)) if edges else 0.0
        bands.append(Band(start=start, stop=stop, kind=kind, polarity=standout,
                          level=level, clipped_start=clipped_start,
                          clipped_stop=clipped_stop, edge_start=edge_start,
                          edge_stop=edge_stop, confidence=confidence))
    return bands


# The picture width of a 35 mm frame along the film. Cameras vary by a few
# tenths: 35.6 mm measured on a ProScan 4000 strip, from both gaps at 600 dpi.
FRAME_MM = 36.0
# A stretch of picture narrower than this fraction of a frame is a sliver of
# the neighbouring frame, not the frame being placed.
FRAME_MIN_FRACTION = 0.5


@dataclass(frozen=True)
class FramePosition:
    '''Where a frame lies in a scan, in image columns.

    `start` and `stop` may lie outside the image: an edge that was not seen is
    placed one frame width from the edge that was. `measured_start` and
    `measured_stop` say which edges a band showed; with neither, the frame is
    assumed centred on what is visible and `confidence` is 0. `confidence` is
    otherwise the step strength of the measured edges, from Band.

    `shift_mm` is how far the image content must move towards +x to centre the
    frame in the window. `overhang_mm` is how much of the frame lies past each
    edge of the image now, (left, right), 0 where it is inside.
    '''
    start: float
    stop: float
    measured_start: bool
    measured_stop: bool
    confidence: float
    shift_mm: float
    overhang_mm: tuple[float, float]
    width_mm: float

    @property
    def measured(self) -> bool:
        return self.measured_start or self.measured_stop


def locate_frame(bands: list[Band], width_px: int, px_per_mm: float,
                 frame_mm: float = FRAME_MM) -> FramePosition | None:
    '''The frame a scan mostly shows, from find_bands' result.

    The frame is the widest stretch of picture between bands. Each side of it
    that a band bounds is a measured edge: a gap, clear film or a blank stretch
    all end the picture. A side that runs into the image edge was not seen and
    is placed `frame_mm` from the other. Returns None when no stretch of picture
    is half a frame wide: a blank frame, or film past the strip's end.
    '''
    frame_px = frame_mm * px_per_mm
    picture = []
    left: Band | None = None
    x = 0
    for band in sorted(bands, key=lambda b: b.start):
        if band.start > x:
            picture.append((x, band.start, left, band))
        x = max(x, band.stop)
        left = band
    if x < width_px:
        picture.append((x, width_px, left, None))
    picture = [p for p in picture if p[1] - p[0] >= FRAME_MIN_FRACTION * frame_px]
    if not picture:
        return None
    start, stop, left, right = max(picture, key=lambda p: p[1] - p[0])

    edges = []
    measured_start = left is not None and start > 0
    measured_stop = right is not None and stop < width_px
    if measured_start:
        edges.append(left.edge_stop or 0.0)
    if measured_stop:
        edges.append(right.edge_start or 0.0)
    if measured_start and not measured_stop:
        stop = start + frame_px
    elif measured_stop and not measured_start:
        start = stop - frame_px
    elif not measured_start and not measured_stop:
        centre = (start + stop) / 2
        start, stop = centre - frame_px / 2, centre + frame_px / 2

    shift_px = width_px / 2 - (start + stop) / 2
    return FramePosition(
        start=float(start), stop=float(stop),
        measured_start=measured_start, measured_stop=measured_stop,
        confidence=float(numpy.mean(edges)) if edges else 0.0,
        shift_mm=shift_px / px_per_mm,
        overhang_mm=(max(0.0, -start) / px_per_mm,
                     max(0.0, stop - width_px) / px_per_mm),
        width_mm=(stop - start) / px_per_mm,
    )
