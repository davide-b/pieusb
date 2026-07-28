'''
Host-side post-processing of raw scanner data.

The scanner returns RAW, uncorrected pixels. Flat-field (shading) correction is
done entirely on the host, in two steps, exactly as the C backend does it:

    1. pieusb_calculate_shading()      (pieusb_specific.c:1681)
    2. sanei_pieusb_correct_shading()  (pieusb_specific.c:1207)

Everything here is pure numpy over bytes already read from the device -- no
device access, no state -- which also makes it the one part of the package that
is testable without hardware.

Infrared dust/scratch removal (sanei_pieusb_post) is a separate, later concern
and is deliberately not done here.
'''

import logging

import numpy

log = logging.getLogger(__name__)

# First tag byte of a shading line -> channel index. The scanner emits the tag
# byte duplicated ('RR', 'GG', ...); the C backend keys off byte 0 only, so we
# do the same (0x52='R', 0x47='G', 0x42='B', 0x49='I').
SHADING_TAG_TO_CHANNEL = {0x52: 0, 0x47: 1, 0x42: 2, 0x49: 3}


def calculate_shading(
    shading_raw: bytes, shading_ppl: int
) -> tuple[dict[int, numpy.ndarray], dict[int, float]] | tuple[None, None]:
    '''Average the shading reference lines into a per-column reference.

    Replicates pieusb_calculate_shading() (pieusb_specific.c:1681-1753).
    Shading data is ALWAYS 16-bit little-endian, with the same two-byte channel
    tag per line as the image data. For each channel its lines are averaged into
    a per-column reference `shading_ref[c]` (length shading_ppl), and
    `shading_mean[c]` is the mean of that reference across all columns.

    Returns (shading_ref, shading_mean), both keyed by channel 0..3, or
    (None, None) if no usable shading lines were found.
    '''
    line_stride = 2 + shading_ppl * 2
    acc = {c: numpy.zeros(shading_ppl, dtype=numpy.float64) for c in range(4)}
    cnt = {c: 0 for c in range(4)}

    n_lines = len(shading_raw) // line_stride
    for k in range(n_lines):
        off = k * line_stride
        channel = SHADING_TAG_TO_CHANNEL.get(shading_raw[off])
        if channel is None:
            continue
        samples = numpy.frombuffer(
            shading_raw, dtype='<u2', count=shading_ppl, offset=off + 2
        ).astype(numpy.float64)
        acc[channel] += samples
        cnt[channel] += 1

    if not any(cnt.values()):
        return None, None

    shading_ref = {}
    shading_mean = {}
    for c in range(4):
        # C divides by shading_height (nLines); cnt[c] == nLines per channel, so
        # this is numerically identical while tolerating channel-count skew.
        ref = acc[c] / cnt[c] if cnt[c] else numpy.ones(shading_ppl)
        shading_ref[c] = ref
        shading_mean[c] = float(ref.mean())
    return shading_ref, shading_mean


def build_width_to_loc(ccd_mask: bytes, width: int) -> list[int]:
    '''Map output column j -> shading-reference column, via the CCD mask.

    Mirrors sanei_pieusb_correct_shading() (pieusb_specific.c:1216-1230): a mask
    byte of 0x00 marks a USED pixel (0x70 = unused). The j-th used pixel's
    position in the mask is the shading-reference index for output column j.
    This is the CCD mask's whole purpose: aligning the `width` image columns to
    the (wider) shading reference, which still contains unused CCD pixels.
    '''
    locs = [i for i, b in enumerate(ccd_mask) if b == 0]
    if len(locs) < width:
        log.warning(
            f"CCD mask marks {len(locs)} used pixels but the image is {width} "
            f"wide; shading columns may be misaligned"
        )
    return locs


def apply_shading_correction(
    image_planes: numpy.ndarray,
    shading_ref: dict[int, numpy.ndarray],
    shading_mean: dict[int, float],
    width_to_loc: list[int],
) -> numpy.ndarray:
    '''Apply flat-field correction in place, per sanei_pieusb_correct_shading()
    (pieusb_specific.c:1232-1244):

        val = shading_mean[c] / shading_ref[c][width_to_loc[j]] * val_org

    `image_planes` is (n_planes, height, width); its dtype decides the clamp.

    Two deliberate divergences from the C:

    - Over-range results. The C backend lets them wrap (an `& 0xFF` on buffer
      output for 8-bit scans); we CLAMP to the dtype max instead, which avoids
      that wraparound artifact.
    - Rounding. The C uses lround(), i.e. half away from zero -- not the
      truncation the PoC's `.astype()` did, which biases every pixel down by up
      to one LSB. floor(x + 0.5) reproduces lround for the non-negative values
      involved here; numpy.rint would not, as it rounds halves to even.
    '''
    n_planes, _height, width = image_planes.shape
    dtype = image_planes.dtype
    maxval = numpy.iinfo(dtype).max
    loc = numpy.asarray(width_to_loc[:width], dtype=numpy.intp)
    if loc.size < width:
        log.warning(
            f"shading correction: only {loc.size}/{width} columns mapped; "
            f"leaving the remainder uncorrected"
        )
    for c in range(n_planes):
        ref_cols = shading_ref[c][loc]                       # length <= width
        gain = numpy.where(ref_cols > 0, shading_mean[c] / ref_cols, 1.0)
        corrected = image_planes[c, :, :loc.size].astype(numpy.float64) * gain
        numpy.floor(corrected + 0.5, out=corrected)
        numpy.clip(corrected, 0, maxval, out=corrected)
        image_planes[c, :, :loc.size] = corrected.astype(dtype)
    return image_planes
