'''
Auto-exposure: turn a preview scan into new gain and exposure-time settings.

A direct port of the SANE backend's "calibration from preview" path, which is
spread over three places in the C:

    sanei_pieusb_analyze_preview()                    (pieusb_specific.c:2368)
    sanei_pieusb_set_gain_offset(..., "from preview") (pieusb_specific.c:1912)
    updateGain2()                                     (pieusb_specific.c:2528)

The idea: scan once at preview resolution, find how much headroom is left below
the CCD's saturation level, and amplify by that factor for the real scan so the
scan depth is actually used. The headroom is measured at the 99th percentile
rather than the maximum, so a handful of specular pixels cannot veto the whole
adjustment.

Everything here is pure -- numpy in, numbers out, no device access -- which is
what makes it testable without hardware. Scanner._meter_from_preview() drives it.
'''

import logging
import math

import numpy

log = logging.getLogger(__name__)

# 256 bins, matching sanei_ir's HISTOGRAM_SIZE (sanei_ir.h:28-29). The bound
# indices this module returns are bin numbers in that space, and the dg formula
# divides by HISTOGRAM_SIZE, so the two must agree.
HISTOGRAM_SIZE = 256

# The denominator SANE compares saturationLevel against: the level is a 16-bit
# quantity, so full scale is 65536, not 65535 (pieusb_specific.c:2511).
SATURATION_FULL_SCALE = 65536

# Ceiling on the amplification a single preview may ask for. Without it a nearly
# black preview -- an empty holder, a lamp that has not warmed -- would demand an
# unbounded boost. 3.0 is the C's literal (pieusb_specific.c:1917).
MAX_GAIN_INCREASE = 3.00

# Actual amplification at gain settings 0, 5, 10, ... 60, from the firmware
# disassembly (pieusb_specific.c:193). Settings in between are interpolated, so
# the usable range is 0..63 => 1.000 .. ~4.63x.
GAINS = (
    1.000, 1.075, 1.154, 1.251, 1.362, 1.491, 1.653,
    1.858, 2.115, 2.458, 2.935, 3.638, 4.627,
)


def _lround(x: float) -> int:
    '''C's lround(): half away from zero. Python's round() is half-to-even and
    would disagree on exactly the .5 cases the interpolations keep producing.'''
    return int(math.floor(x + 0.5)) if x >= 0 else -int(math.floor(-x + 0.5))


def get_gain(setting: int) -> float:
    '''Actual amplification at gain `setting` (0-63). Ports getGain()
    (pieusb_specific.c:2420).

    The >= 60 branch extrapolates from (setting - 55), which does not line up
    with get_gain_setting()'s inverse branch (that one starts at 60). The
    inconsistency is in the C and is kept: it only affects the top four settings,
    and diverging here would silently shift every gain the backend has ever
    chosen.
    '''
    if setting <= 0:
        return GAINS[0]
    if setting >= 60:
        return (setting - 55) * (GAINS[12] - GAINS[11]) / 5 + GAINS[11]
    k = setting // 5
    return (setting - 5 * k) * (GAINS[k + 1] - GAINS[k]) / 5 + GAINS[k]


def get_gain_setting(gain: float) -> int:
    '''Lowest gain setting reaching `gain`. Ports getGainSetting()
    (pieusb_specific.c:2440). Inverse of get_gain(), modulo the >= 60 skew.'''
    if gain < 1.0:
        return 0
    if gain >= GAINS[12]:
        return min(63, 60 + _lround((gain - GAINS[11]) / (GAINS[12] - GAINS[11]) * 5))
    setting = 0
    for k in range(12):
        if GAINS[k] <= gain < GAINS[k + 1]:
            setting = 5 * k + _lround((gain - GAINS[k]) / (GAINS[k + 1] - GAINS[k]) * 5)
    return setting


def norm_histogram(plane: numpy.ndarray) -> numpy.ndarray:
    '''256-bin histogram of one colour plane, normalised to sum 1.

    Ports sanei_ir_create_norm_histo() (sanei_ir.c:58), which hard-codes a
    >> (16 - HISTOGRAM_SHIFT) = >> 8 because its buffer is always 16-bit.

    Divergence, deliberate: an 8-bit plane is binned directly instead of shifted.
    The C would shift 8-bit samples (which it stores widened into a 16-bit
    buffer) down into bin 0 and derive an upper bound of 0 from it, which is a
    bug, not a behaviour worth reproducing.
    '''
    if plane.dtype.itemsize > 1:
        bins = numpy.right_shift(plane, 8 * (plane.dtype.itemsize - 1))
    else:
        bins = plane
    counts = numpy.bincount(bins.ravel(), minlength=HISTOGRAM_SIZE)
    return counts[:HISTOGRAM_SIZE] / plane.size


def percentile_bounds(plane: numpy.ndarray) -> tuple[int, int]:
    '''The 1% and 99% bin indices of `plane`. Ports the loop in
    sanei_pieusb_analyze_preview() (pieusb_specific.c:2394-2405).

    Both bounds are the LAST bin whose inclusive cumulative share is still under
    the threshold -- not the first bin at or above it. That is an off-by-one
    against the usual reading of "99th percentile", but it is what the C does and
    what the gain formula was tuned against.
    '''
    histo = norm_histogram(plane)
    level = 0.0
    lower = upper = 0
    for n in range(HISTOGRAM_SIZE):
        level += float(histo[n])
        if level < 0.01:
            lower = n
        if level < 0.99:
            upper = n
    return lower, upper


def gain_increase(
    upper_bounds: tuple[int, ...],
    saturation_levels: tuple[int, ...],
    cap: float = MAX_GAIN_INCREASE,
) -> float:
    '''How much brighter the real scan should be than the preview.

    Ports the SCAN_CALIBRATION_PREVIEW branch (pieusb_specific.c:1912-1940):
    per channel, the ratio between where the CCD saturates and where the
    preview's 99th percentile landed; the smallest such ratio wins, capped.

    ONE factor for all channels, not one per channel -- that is the whole point
    of the C's updateGain2 replacing the per-channel updateGain it left commented
    out above it. Equalising the channels individually amplifies the negative's
    unexposed orange border up to near-white and makes it noisy
    (pieusb_specific.c:2464-2472); a common factor moves the whole exposure up
    and leaves the channel ratios, i.e. the colour, alone.

    A channel whose upper bound is 0 is skipped: the C would divide by zero
    there. If every channel is 0 the preview carries no signal at all and 1.0
    (leave the settings alone) is returned.
    '''
    dg = cap
    used = False
    for k, upper in enumerate(upper_bounds):
        if upper <= 0:
            log.warning(f"[autoexp] channel {k} preview 99% bound is 0; skipping it")
            continue
        dgi = (saturation_levels[k] / SATURATION_FULL_SCALE) / (upper / HISTOGRAM_SIZE)
        used = True
        if dgi < dg:
            dg = dgi
    if not used:
        log.warning("[autoexp] no usable preview channel; leaving gain and exposure as they are")
        return 1.0
    return dg


def update_gain(gain_setting: int, exposure_time: int, dg: float) -> tuple[int, int]:
    '''Split an amplification of `dg` between gain and exposure time.

    Ports updateGain2() (pieusb_specific.c:2528). Gain takes sqrt(dg) and the
    exposure time takes whatever the (coarsely quantised) gain could not deliver:

        new_gain     = get_gain_setting(g * sqrt(dg))
        new_exposure = lround(g / get_gain(new_gain) * dg * exposure_time)

    so the product of the two changes is exactly dg regardless of where the gain
    setting rounded to. Returns (gain setting 0-63, exposure time in Timer 1
    counts); neither is clamped, matching the C -- the caller checks the ranges.

    `exposure_time` is the ABSOLUTE integration time, i.e. the exp_time_* options
    carried by SET GAIN OFFSET. The scanner's separate relative-exposure
    percentage (exp_rel_*) plays no part in any of this and is assumed to be at
    its 100% default -- the saturation levels gain_increase() meters against were
    measured by the firmware with it there.
    '''
    g = get_gain(gain_setting)
    new_setting = get_gain_setting(g * math.sqrt(dg))
    new_gain = get_gain(new_setting)
    new_exposure = _lround(g / new_gain * dg * exposure_time)
    log.debug(f"[autoexp] gain {gain_setting} ({g:.3f}) x{dg:.3f} -> setting {new_setting} "
              f"({new_gain:.3f}), exposure {exposure_time} -> {new_exposure}")
    return new_setting, new_exposure
