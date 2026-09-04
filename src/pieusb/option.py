from collections.abc import Callable
from dataclasses import dataclass
from typing import Generic, TypeVar
from enum import Enum

import logging
import struct

from pieusb.inquiry import (
    InquiryResponse,
    Filter
)
from pieusb.types import Capabilities
from pieusb.transport import (
    UASDevice,
    SCSI_WRITE,
    SCSI_MODE_SELECT,
    SCSI_HIGHLIGHT_SHADOW,
    SCSI_EXPOSURE,
    SCSI_SCAN_FRAME,
    SCSI_WRITE_GAIN_OFFSET
)
from pieusb.exceptions import CheckCondition, ParamError

log = logging.getLogger(__name__)

# MODE SELECT payload constants, from pieusb_specific.h:55-75.

# 'passes' -- which filters the scanner acquires in a single pass
SCAN_ONE_PASS_RGBI = 0x90
SCAN_ONE_PASS_COLOR = 0x80
SCAN_FILTER_GREEN = 0x04

# 'colorFormat' -- how the scanner lays the returned data out. INDEX means every
# line is prefixed by a two-byte 'RR'/'GG'/'BB'/'II' channel tag; PIXEL means
# interleaved RGB pixels, of which only the first is valid in a single-filter scan.
SCAN_COLOR_FORMAT_INDEX = 0x04
SCAN_COLOR_FORMAT_PIXEL = 0x01

# mode name -> (passes, colorFormat), per sanei_pieusb_set_mode_from_options()
# (pieusb_specific.c:1808-1838). Gray is the green filter alone: the C backend
# notes it was "unable to get R & B & I to work" as single-filter scans.
MODE_SETTINGS = {
    'gray': (SCAN_FILTER_GREEN, SCAN_COLOR_FORMAT_PIXEL),
    'rgb': (SCAN_ONE_PASS_COLOR, SCAN_COLOR_FORMAT_INDEX),
    'rgbi': (SCAN_ONE_PASS_RGBI, SCAN_COLOR_FORMAT_INDEX),
}

# How many planes a mode produces, i.e. how many tagged line blocks the scanner
# sends per image.
MODE_PLANES = {'gray': 1, 'rgb': 3, 'rgbi': 4}

# Refused by validate(): 'gray' uses SCAN_COLOR_FORMAT_PIXEL, whose lines carry no
# channel tag, so Scanner's tag-based deinterleave cannot decode it. The data comes
# back as RGB pixel triples of which only the first is valid
# (pieusb_scancmd.h:165-173).
UNSUPPORTED_MODES = ('gray',)

# Sample sizes the deinterleave in Scanner._scan_pass understands. INQUIRY may
# advertise 1/4/10/12 too; only these map onto a numpy dtype.
SUPPORTED_COLOR_DEPTHS = (8, 16)

# 'colorDepth' bitmask, pieusb_specific.h:64-70
COLOR_DEPTHS = {1: 0x01, 4: 0x02, 8: 0x04, 10: 0x08, 12: 0x10, 16: 0x20}

# 'lineThreshold', 0xFF being 100%. Only meaningful for the unsupported
# lineart/halftone modes. 128 is what the capture in pieusb_scancmd.c:757 sends.
LINE_THRESHOLD = 128

# The only relative-exposure value SANE ever sends (pieusb.c:878-882), and the
# floor the device clamps to: 50 measures the same as 100.
DEFAULT_RELATIVE_EXPOSURE = 100

# Width of the scanner's Timer 1 count. The relative exposure is applied to the
# device's own absolute exposure time before it lands in that register:
#
#     timer = exposure_time * exp_rel / 100      (mod TIMER_FULL_SCALE)
#
# so exp_rel_* is only usable while the product fits. Past that the register wraps
# and the exposure drops, non-monotonically, which is audible as a wrong line rate.
# Measured on a ProScan 10T, whose exposure_time is 4100 -- giving a ceiling of
# 65535 * 100 // 4100 = 1598 -- against a nominal of exp_rel/100:
#
#   exp_rel   1086   1500  |  1598  1599  |  2200  3300  5000  7500  10000
#   predicted x10.9  x15.0 | x15.98 x1.00 | x6.02 x1.03 x2.05 x11.1  x4.09
#   measured  x10.5  x14.7 |   clip x1.00 | x5.79 x1.06 x2.01 x10.7  x3.94
#
# Every value matches to within 4%, and the 1598/1599 step is exactly where the
# product crosses 65535. It lands on x1.00 rather than near zero because a wrapped
# result below 100 meets the same floor that makes exp_rel=50 behave as 100.
TIMER_FULL_SCALE = 65536


def max_relative_exposure(exposure_time: int) -> int:
    '''Largest exp_rel_* that does not overflow the device's Timer 1 count.'''
    if exposure_time <= 0:
        return DEFAULT_RELATIVE_EXPOSURE
    return max(
        DEFAULT_RELATIVE_EXPOSURE,
        (TIMER_FULL_SCALE - 1) * DEFAULT_RELATIVE_EXPOSURE // exposure_time,
    )


# The exposure time a ProScan 10T reports and uses whatever is written to it, and
# the ceiling that follows from it. Scanner reads the real value per scan; this is
# for OptionsTable.validate(), which has no device reading to work from.
OBSERVED_EXPOSURE_TIME = 4100
MAX_RELATIVE_EXPOSURE = max_relative_exposure(OBSERVED_EXPOSURE_TIME)

# exp_time_*, in Timer 1 counts. 2937 is SANE's DEFAULT_EXPOSURE
# (pieusb_specific.h:105) and the floor both ProScan 4000 drivers clamp to.
DEFAULT_EXPOSURE_TIME = 2937
EXPOSURE_TIME_MAX = 0xFFFF

# SCSI_SLIDE's focus byte. 1 is the near end of the range on every model that has
# a focus motor; the far end is holder-dependent and only the device knows it, so
# Scanner clamps against READ STATE rather than validating here.
DEFAULT_FOCUS = 1
FOCUS_MIN = 1
FOCUS_MAX_FIELD = 0xFF

# Light level used when the device reports none of its own. SANE's DEFAULT_LIGHT
# (pieusb_specific.h:107) and the value the firmware settles at once warm.
DEFAULT_LIGHT = 4

# 'extraEntries', byte 16 of SET GAIN OFFSET. Sizes the device's BADF table
# (pieusb.c:1027-1033). 0 is what the C sends alongside the device's own gain and
# exposure: SCAN_CALIBRATION_AUTO leaves the field untouched because
# get_gain_offset() does not decode it (pieusb_scancmd.c:1042-1051). The
# DEFAULT_ADDITIONAL_ENTRIES of 1 belongs to the DEFAULT and OPTIONS modes only.
ADDITIONAL_ENTRIES = 0

# 'doubleTimes', byte 17. Undocumented beyond the name; no driver sends anything
# but SANE's DEFAULT_DOUBLE_TIMES (pieusb_specific.h:109).
DOUBLE_TIMES = 0

# SET GAIN OFFSET parameter list lengths, longest first. The list's last field is
# gain_i at byte 22, so 23 bytes carries all of them; SANE sends 29 by writing out
# its whole padded struct (pieusb_scancmd.c:1082), and the ProScan 4000 rejects
# that with ILLEGAL REQUEST / parameter list length error while its own driver
# sends 23 (captures/ProScan4000.pcapng, transaction 72). Tried in this order so a
# scanner that takes the padded form keeps getting it.
GAIN_OFFSET_LENGTHS = (29, 23)

class Unit(Enum):
    MM = 0
    PIXEL = 1
    BITS = 2
    MICROSECONDS = 3
    NONE = 4
    PERCENT = 5
    # The unit the scanner's exposure times use. Not convertible to a duration
    # without the Timer 1 clock rate, which the device does not report.
    TIMER_COUNTS = 6

T = TypeVar("T")

@dataclass(frozen=True)
class Option(Generic[T]):
    name: str
    type: type[T]
    unit: Unit
    validate: Callable[[T], bool]
    default: T

class Parameter:
    def __init__(self, opt: Option):
        self.opt = opt
        assert opt.validate(opt.default)
        self.value = opt.default

class OptionsTable:
    def __init__(self, params: list[Parameter], inq: InquiryResponse) -> None:
        self.table = params
        # Kept so validate() can re-check the frame against the reported bed.
        self.inq = inq

    def __getitem__(self, key) -> Parameter:
        try:
            out = next(filter(lambda o: o.opt.name == key, self.table))
            return out
        except StopIteration:
            raise KeyError(f"No parameter named {key} exists")

    def validate(self) -> None:
        '''Validate the table as a whole, before anything reaches the device.

        Re-runs every per-option validator, then applies the cross-option checks a
        single-option validator cannot express: the frame corners against each
        other and against the bed, and the mode/quality-bit combinations.

        Raises ParamError, naming the offending option(s). Combinations that are
        merely ineffective rather than contradictory are logged as warnings.
        '''
        for par in self.table:
            if type(par.value) is not par.opt.type:
                raise ParamError(
                    f"Option '{par.opt.name}' holds a {type(par.value).__name__} "
                    f"({par.value!r}), expected {par.opt.type.__name__}"
                )
            if not par.opt.validate(par.value):
                raise ParamError(f"Option '{par.opt.name}' has an invalid value ({par.value!r})")

        tl_x, tl_y = self['tl_x'].value, self['tl_y'].value
        br_x, br_y = self['br_x'].value, self['br_y'].value

        if tl_x >= br_x:
            raise ParamError(f"Parameter 'tl_x' ({tl_x}) must be smaller than parameter 'br_x' ({br_x})")

        if tl_y >= br_y:
            raise ParamError(f"Parameter 'tl_y' ({tl_y}) must be smaller than parameter 'br_y' ({br_y})")

        # The frame against the bed the device reports (INQUIRY offsets 40/42, in
        # native-resolution units). X is the long axis and Y the short one;
        # swapping them drives the head off its rail, so this is checked again
        # here, the last gate before SET SCAN FRAME.
        if tl_x < 0 or br_x > self.inq.max_scan_w:
            raise ParamError(
                f"Requested X range [{tl_x}, {br_x}] exceeds the scanner's reported bed "
                f"[0, {self.inq.max_scan_w}] -- check that the X/Y axes are not swapped"
            )
        if tl_y < 0 or br_y > self.inq.max_scan_h:
            raise ParamError(
                f"Requested Y range [{tl_y}, {br_y}] exceeds the scanner's reported bed "
                f"[0, {self.inq.max_scan_h}] -- check that the X/Y axes are not swapped"
            )

        mode = self['mode'].value
        if mode in UNSUPPORTED_MODES:
            raise ParamError(
                f"Mode '{mode}' is not implemented yet: it returns untagged pixel data "
                f"which the deinterleave cannot decode. Use "
                f"{' or '.join(repr(m) for m in MODE_SETTINGS if m not in UNSUPPORTED_MODES)}."
            )

        depth = self['color_depth'].value
        if depth not in SUPPORTED_COLOR_DEPTHS:
            raise ParamError(
                f"Colour depth {depth} is advertised by the scanner but not implemented here; "
                f"supported: {', '.join(str(d) for d in SUPPORTED_COLOR_DEPTHS)}"
            )

        # "Only effective with fastInfrared off" (pieusb_scancmd.h:180), so asking
        # for both is a contradiction; the scanner would silently drop one.
        if self['sharpen'].value and self['fast_infrared'].value:
            raise ParamError(
                "Options 'sharpen' and 'fast_infrared' are mutually exclusive: sharpening is "
                "only effective with fast infrared off (pieusb_scancmd.h:180)"
            )

        # Ineffective, not contradictory: warn and carry on.
        if self['sharpen'].value and mode != 'rgb' and mode != 'rgbi':
            log.warning(f"option 'sharpen' has no effect in '{mode}' mode (one-pass colour only)")
        if self['fast_infrared'].value and mode != 'rgbi':
            log.warning(f"option 'fast_infrared' has no effect in '{mode}' mode (no infrared plane)")

        # exp_rel_* is the exposure control and is meant to be moved. Warned about
        # rather than refused: the real ceiling depends on the exposure time the
        # device reports, which only Scanner gets to read.
        high = [
            f'{n}={self[n].value}' for n in ('exp_rel_r', 'exp_rel_g', 'exp_rel_b')
            if self[n].value > MAX_RELATIVE_EXPOSURE
        ]
        if high:
            log.warning(
                f"relative exposure above {MAX_RELATIVE_EXPOSURE}% ({', '.join(high)}); "
                f"exposure_time * exp_rel / 100 overflows the scanner's 16-bit Timer 1 "
                f"there, and the scan comes out DARKER than asked for, not brighter"
            )
        low = [
            f'{n}={self[n].value}' for n in ('exp_rel_r', 'exp_rel_g', 'exp_rel_b')
            if self[n].value < DEFAULT_RELATIVE_EXPOSURE
        ]
        if low:
            log.warning(
                f"relative exposure below {DEFAULT_RELATIVE_EXPOSURE}% ({', '.join(low)}); "
                f"the device clamps there, so this scales nothing"
            )

def generate_options(inq: InquiryResponse,
                     capabilities: Capabilities | None = None) -> OptionsTable:
    out: list[Parameter] = []
    capabilities = capabilities or Capabilities()

    modes = []
    if Filter.GREEN in inq.filters:
        modes.append('gray')
    if set({Filter.RED, Filter.GREEN, Filter.BLUE}) <= set(inq.filters):
        modes.append('rgb')
    if set({Filter.RED, Filter.GREEN, Filter.BLUE, Filter.INFRARED}) <= set(inq.filters):
        modes.append('rgbi')
    out.append(Parameter(Option(
        name='mode',
        type=str,
        unit=Unit.NONE,
        validate=lambda v: v in modes,
        default='rgb'
    )))

    # Bit depth of the scan
    out.append(Parameter(Option(
        name='color_depth',
        type=int,
        unit=Unit.BITS,
        validate=lambda v: v in inq.color_depths,
        default=16
    )))

    # Scan resolution
    out.append(Parameter(Option(
        name='resolution',
        type=int,
        unit=Unit.NONE,
        validate=lambda v: v <= min(inq.max_resolution_x, inq.max_resolution_y),
        default=300
    )))

    # Increase sharpness by giving the CCD more time to discharge between lines.
    # Only effective with 'fast_infrared' off and a one-pass colour mode
    # (pieusb_scancmd.h:180).
    out.append(Parameter(Option(
        name='sharpen',
        type=bool,
        unit=Unit.NONE,
        validate=lambda v: type(v) is bool,
        default=False
    )))

    # Reuse the shading (flat-field) reference an earlier scan on this Scanner
    # acquired instead of acquiring a fresh one. BEST EFFORT, and neither failure
    # mode can produce an uncorrected image:
    #
    #  - The scanner may refuse, answering START SCAN with MUST_CALIBRATE.
    #  - With nothing cached to reuse, Scanner acquires anyway.
    #
    # Only ever saves a calibration pass. See Scanner._scan_pass.
    out.append(Parameter(Option(
        name='reuse_calibration',
        type=bool,
        unit=Unit.NONE,
        validate=lambda v: type(v) is bool,
        default=False
    )))

    # Acquire the infrared plane in a faster, lower-quality pass. Requires a
    # scanner with an infrared filter, and disables 'sharpen' when on.
    out.append(Parameter(Option(
        name='fast_infrared',
        type=bool,
        unit=Unit.NONE,
        validate=lambda v: not v or Filter.INFRARED in inq.filters,
        default=False
    )))

    # Meter a preview pass and set exp_rel_* per channel from it, instead of
    # scanning with whatever they hold. Costs one extra pass at the device's preview
    # resolution; see Scanner._meter_exposure.
    #
    # Nothing else is derived, because nothing else has an effect: exp_rel_* is the
    # only exposure control this hardware honours (see the exposure section below).
    # In particular this does NOT adopt the device's own gain/exposure calibration
    # the way the C's SCAN_CALIBRATION_AUTO does -- those values are inert here.
    out.append(Parameter(Option(
        name='auto_exp',
        type=bool,
        unit=Unit.NONE,
        validate=lambda v: type(v) is bool,
        default=False
    )))

    # X coordinate of the top-left corner
    out.append(Parameter(Option(
        name='tl_x',
        type=int,
        unit=Unit.PIXEL,
        validate=lambda v: v >= 0,
        default=0
    )))

    # Y coordinate of the top-left corner
    out.append(Parameter(Option(
        name='tl_y',
        type=int,
        unit=Unit.PIXEL,
        validate=lambda v: v >= 0,
        default=0
    )))

    # X coordinate of the bottom-right corner
    out.append(Parameter(Option(
        name='br_x',
        type=int,
        unit=Unit.PIXEL,
        validate=lambda v: v <= inq.max_scan_w,
        default=inq.max_scan_w
    )))

    # Y coordinate of the bottom-right corner
    out.append(Parameter(Option(
        name='br_y',
        type=int,
        unit=Unit.PIXEL,
        validate=lambda v: v <= inq.max_scan_h,
        default=inq.max_scan_h
    )))

    # --- Exposure -------------------------------------------------------------
    #
    # Two independent controls, sent by different commands:
    #
    #   exp_time_*  ABSOLUTE integration time in Timer 1 counts, one per filter
    #               including infrared, carried by SET GAIN OFFSET.
    #   exp_rel_*   RELATIVE percentage, R/G/B only, sent by SCSI_EXPOSURE.
    #
    # MEASURED ON A PIE ProScan 10T (rev 1.70): exp_rel_* is the only one that does
    # anything. exp_time_* has no effect on the image at any value -- 500, 1500,
    # 2500, 2937, 6000 and 10000 all produce the same exposure to within 0.2% --
    # and neither does gain_* or light. Whether that generalises to other models is
    # unknown, so all of them are still sent, and exp_time_*'s documented meaning
    # is recorded below for the ones where it may work.
    #
    # The firmware optimises exp_time_* during warm-up until R and B reach >=90%
    # of full scale and G >=80% (pieusb_scancmd.h:188-197), then reports what it
    # settled on through GET GAIN OFFSET. A ProScan 10T reports a flat 4100 and
    # ignores what it is sent; a ProScan 4000 reports 10286/7345/4405 and honours
    # it.
    #
    # INQUIRY's maximum_exposure is not a bound. Both models advertise 2500, the
    # 4000's own calibration exceeds even four times that, and its drivers write
    # up to 65535. The 16-bit wire field is the only real limit.
    for filt in ('r', 'g', 'b', 'i'):
        out.append(Parameter(Option(
            name=f'exp_time_{filt}',
            type=int,
            unit=Unit.TIMER_COUNTS,
            validate=lambda v: inq.minimum_exposure <= v <= EXPOSURE_TIME_MAX,
            default=DEFAULT_EXPOSURE_TIME
        )))

    # RELATIVE percentage, a 16-bit field per filter sent by SCSI_EXPOSURE
    # (pieusb_scancmd.c:521-544). Infrared has no entry: the C struct holds three
    # colours.
    #
    # THIS IS THE EXPOSURE CONTROL, at least on a ProScan 10T, and it is per
    # channel. Measured against a colour negative at 300 dpi, 16-bit:
    #
    #   exp_rel   50    100    200    250    400    800   1086   1500   1598
    #   measured   x1     x1  x1.97  x2.46  x3.93  x8.03  x10.5  x14.7  clips
    #
    # Linear to within 4% and clamped at the bottom -- 50 behaves as 100, so it
    # scales up only. The top is set by Timer 1 overflow, see
    # max_relative_exposure(). It is the per-line integration period, so the line
    # rate halves as it doubles and scan time scales with the largest of the three
    # channels.
    #
    # Setting it per channel is how a colour negative gets exposed: the orange mask
    # attenuates blue about 4.4x more than red, and 247/563/1086 put all three
    # channels within 1% of each other at ~88% of full scale with nothing clipped.
    # Driving any channel to saturation produces vertical banding rather than flat
    # white, because the shading correction's per-column gain makes the clipping
    # point column-dependent (see postprocess.apply_shading_correction).
    #
    # SANE hard-codes all three to 100 and never varies them (pieusb.c:878-882,
    # 927), which is why its calibration could never expose this film. The bound
    # below is the width of the wire field; behaviour above 1086 is untested.
    for filt in ('r', 'g', 'b'):
        out.append(Parameter(Option(
            name=f'exp_rel_{filt}',
            type=int,
            unit=Unit.PERCENT,
            validate=lambda v: 0 <= v <= 0xFFFF,
            default=DEFAULT_RELATIVE_EXPOSURE
        )))

    # SANE gain default 19
    # Gain for red channel
    out.append(Parameter(Option(
        name='gain_r',
        type=int,
        unit=Unit.NONE,
        validate=lambda v: v >= 0 and v < 64, # From firmware disassembly
        default=19
    )))
    
    # Gain for green channel
    out.append(Parameter(Option(
        name='gain_g',
        type=int,
        unit=Unit.NONE,
        validate=lambda v: v >= 0 and v < 64, # From firmware disassembly
        default=19
    )))
    
    # Gain for blue channel
    out.append(Parameter(Option(
        name='gain_b',
        type=int,
        unit=Unit.NONE,
        validate=lambda v: v >= 0 and v < 64, # From firmware disassembly
        default=19
    )))
    
    # Gain for infrared channel
    out.append(Parameter(Option(
        name='gain_i',
        type=int,
        unit=Unit.NONE,
        validate=lambda v: v >= 0 and v < 64, # From firmware disassembly
        default=19
    )))

    # SANE offset default 0
    # Offset for the red channel
    out.append(Parameter(Option(
        name='offset_r',
        type=int,
        unit=Unit.NONE,
        validate=lambda v: v >= 0 and v < 255, # From firmware disassembly
        default=0
    )))

    # Offset for the green channel
    out.append(Parameter(Option(
        name='offset_g',
        type=int,
        unit=Unit.NONE,
        validate=lambda v: v >= 0 and v < 255, # From firmware disassembly
        default=0
    )))
    
    # Offset for the blue channel
    out.append(Parameter(Option(
        name='offset_b',
        type=int,
        unit=Unit.NONE,
        validate=lambda v: v >= 0 and v < 255, # From firmware disassembly
        default=0
    )))
    
    # Offset for the infrared channel
    out.append(Parameter(Option(
        name='offset_i',
        type=int,
        unit=Unit.NONE,
        validate=lambda v: v >= 0 and v < 255, # From firmware disassembly
        default=0
    )))

    # Focus position for SCSI_SLIDE action 0x10, on models whose slide command
    # carries one. Absent otherwise, so it cannot be set on a scanner that would
    # read the byte as something else.
    if capabilities.focus:
        out.append(Parameter(Option(
            name='focus',
            type=int,
            unit=Unit.NONE,
            validate=lambda v: FOCUS_MIN <= v <= FOCUS_MAX_FIELD,
            default=DEFAULT_FOCUS
        )))

    # Lamp level, byte 15 of SET GAIN OFFSET.
    #
    #   "Current light level. The stability of the light source is tested during
    #    warming up. The check starts with a light value 7 or 6, and decrements
    #    it when the light warms up. At a light value of 4, the scanner produces
    #    stable scans (i.e. successive 'white' scan values don't differ more than
    #    0x200)." (pieusb_scancmd.h:208-213)
    #
    # So the operating band is 4..7, with 4 the warmed-up value; SANE types it as a
    # duration in microseconds (pieusb_specific.c:916). The range below is the width
    # of the wire field, not a documented limit.
    #
    # No measurable effect on a ProScan 10T: 4, 5, 6 and 7 produce the same image to
    # within 0.1%. Sent anyway, since 0 is outside the band and no other model has
    # been tested. Use exp_rel_* to change exposure.
    out.append(Parameter(Option(
        name='light',
        type=int,
        unit=Unit.MICROSECONDS,
        validate=lambda v: 0 <= v <= 0xFF,
        default=DEFAULT_LIGHT
    )))

    return OptionsTable(out, inq)

def write_gain_offset(dev: UASDevice, payload: bytes) -> int:
    """SET GAIN OFFSET at a length this device accepts, returning that length.

    Every candidate in GAIN_OFFSET_LENGTHS is a prefix of the longest one, so a
    shorter list drops trailing padding rather than a field. A rejection that is
    not about the length propagates from the first attempt.
    """
    for length in GAIN_OFFSET_LENGTHS:
        try:
            dev.command(SCSI_WRITE_GAIN_OFFSET, out_data=payload[:length],
                        cdb_length=length)
        except CheckCondition as e:
            if not e.parameter_list_length_error or length == GAIN_OFFSET_LENGTHS[-1]:
                raise
            log.info(f"SET GAIN OFFSET: the scanner refused a {length}-byte "
                     f"parameter list ({e}); trying a shorter one")
            continue
        if length != GAIN_OFFSET_LENGTHS[0]:
            log.info(f"SET GAIN OFFSET: this scanner takes a {length}-byte "
                     f"parameter list")
        return length

def set_options(dev: UASDevice, options: OptionsTable, *,
                skip_shading_analysis: bool = False) -> None:
    """Send every option to the device, in the order the scan sequence needs.

    `skip_shading_analysis` is a parameter rather than an option because whether a
    pass can skip acquiring a shading reference depends on what the Scanner has
    cached, not on 'reuse_calibration' alone.
    """
    # Highlight and shadow, hard-coded to 100 in SANE (pieusb.c:884-887).
    for filt, value in ((0x02, 100), (0x04, 100), (0x08, 100)):
        payload = struct.pack("<HHHH", SCSI_HIGHLIGHT_SHADOW, 4, filt, value)
        dev.command(SCSI_WRITE, out_data=payload, cdb_length=8)

    # RELATIVE exposure, a percentage -- not exp_time_*, which is an absolute
    # integration time and goes out with SET GAIN OFFSET below. Three filters only.
    for filt, name in ((0x02, 'exp_rel_r'), (0x04, 'exp_rel_g'), (0x08, 'exp_rel_b')):
        payload = struct.pack("<HHHH", SCSI_EXPOSURE, 4, filt, options[name].value)
        dev.command(SCSI_WRITE, out_data=payload, cdb_length=8)

    index = 128 # Frame index; the scanner accepts this one.
    x0 = options["tl_x"].value
    y0 = options["tl_y"].value
    x1 = options["br_x"].value
    y1 = options["br_y"].value
    payload = struct.pack("<HHHHHHH", SCSI_SCAN_FRAME, 10, index, x0, y0, x1, y1)
    dev.command(SCSI_WRITE, out_data=payload, cdb_length=14)

    # ABSOLUTE exposure time, gain, offset and lamp level, all one command. Byte
    # for byte sanei_pieusb_cmd_set_gain_offset() (pieusb_scancmd.c:1102-1116);
    # bytes 9-11, 21 and 23-28 are unused.
    payload = struct.pack('<HHHBBBBBBBBBBBBHBBBBBBBBB',
        options['exp_time_r'].value,
        options['exp_time_g'].value,
        options['exp_time_b'].value,
        options['offset_r'].value,
        options['offset_g'].value,
        options['offset_b'].value,
        0,
        0,
        0,
        options['gain_r'].value,
        options['gain_g'].value,
        options['gain_b'].value,
        options['light'].value,
        ADDITIONAL_ENTRIES,
        DOUBLE_TIMES,
        options['exp_time_i'].value,
        options['offset_i'].value,
        0,
        options['gain_i'].value,
        0, 0, 0, 0, 0, 0
    )
    write_gain_offset(dev, payload)

    # MODE SELECT, a fixed 16-byte payload per sanei_pieusb_cmd_set_mode()
    # (pieusb_scancmd.c:731-800). Bytes 0, 7, 10, 11 and 15 are unused.
    passes, color_format = MODE_SETTINGS[options['mode'].value]

    # Quality bitmask, byte 9 (pieusb_scancmd.c:790-794)
    quality = 0
    if options['sharpen'].value:
        quality |= 0x02
    if skip_shading_analysis:
        # skipShadingAnalysis = !OPT_SHADING_ANALYSIS (pieusb_specific.c:1857).
        quality |= 0x08
    if options['fast_infrared'].value:
        quality |= 0x80

    payload = bytearray(16)
    payload[1] = 15 # Size of the data that follows
    struct.pack_into('<H', payload, 2, options['resolution'].value)
    payload[4] = passes
    payload[5] = COLOR_DEPTHS[options['color_depth'].value]
    payload[6] = color_format
    payload[8] = 0x01 # byteOrder: Intel, only bit 0 is used
    payload[9] = quality
    payload[12] = 0x00 # halftonePattern; the C backend only ever sends 0
    payload[13] = LINE_THRESHOLD
    payload[14] = 0x10 # Unexplained; sent unconditionally by the C backend
                       # (pieusb_scancmd.c:797) and present in the capture there
    dev.command(SCSI_MODE_SELECT, out_data=bytes(payload), cdb_length=16)
