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
from pieusb.transport import (
    UASDevice,
    SCSI_WRITE,
    SCSI_MODE_SELECT,
    SCSI_HIGHLIGHT_SHADOW,
    SCSI_EXPOSURE,
    SCSI_SCAN_FRAME,
    SCSI_WRITE_GAIN_OFFSET
)
from pieusb.exceptions import ParamError

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

# The only relative-exposure value SANE ever sends (pieusb.c:878-882).
DEFAULT_RELATIVE_EXPOSURE = 100

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
        if self['advance'].value:
            log.warning("option 'advance' is not implemented yet and will be ignored: "
                        "slide-transport commands are not sent")

        # Warns every scan rather than refusing: a stray relative exposure is
        # invisible in the result.
        moved = [
            f'{n}={self[n].value}' for n in ('exp_rel_r', 'exp_rel_g', 'exp_rel_b')
            if self[n].value != DEFAULT_RELATIVE_EXPOSURE
        ]
        if moved:
            log.warning(
                f"relative exposure moved off {DEFAULT_RELATIVE_EXPOSURE}% ({', '.join(moved)}). "
                f"No shipping driver sends anything else and the device's response is "
                f"unknown. Prefer exp_time_* to change exposure."
            )

def generate_options(inq: InquiryResponse) -> OptionsTable:
    out: list[Parameter] = []

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

    # Take gain_*, exp_time_*, offset_* and light from GET GAIN OFFSET instead of
    # from whatever those options hold. The firmware optimises them per channel
    # while warming up; this is the C's default calibration mode
    # (SCAN_CALIBRATION_AUTO, pieusb_specific.c:733, 1988) and costs no extra pass.
    #
    # The scanner only fills those fields in during its first scan, so the first
    # auto_exp scan of a session finds zeros, warns and runs with the options as
    # set. Scan twice. See Scanner._adopt_device_calibration.
    out.append(Parameter(Option(
        name='auto_exp',
        type=bool,
        unit=Unit.NONE,
        validate=lambda v: type(v) is bool,
        default=False
    )))

    # Advance slide after scan
    out.append(Parameter(Option(
        name='advance',
        type=bool,
        unit=Unit.NONE,
        validate=lambda v: not v or inq.slide_transport,
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
    # Two independent controls, sent by different commands. Easy to confuse, hence
    # the distinct names:
    #
    #   exp_time_*  ABSOLUTE integration time in Timer 1 counts, one per filter
    #               including infrared, carried by SET GAIN OFFSET.
    #   exp_rel_*   RELATIVE percentage, R/G/B only, sent by SCSI_EXPOSURE.
    #               See below -- leave it alone.
    #
    # The firmware optimises exp_time_* during warm-up until R and B reach >=90% of
    # full scale and G >=80% (pieusb_scancmd.h:188-197), then usually resets them
    # to 0x0B79 = 2937, SANE's DEFAULT_EXPOSURE (pieusb_specific.h:105).
    #
    # This is the per-line integration period, so raising it scales scan time and
    # lamp-on time one for one. Prefer gain_*, or 'light' within its 4..7 band.
    #
    # The advertised maximum is multiplied by 4 because it does not otherwise
    # contain 2937 -- the device's own default is out of its own reported range.
    # SANE applies the same factor (pieusb_specific.c:391).
    DEFAULT_EXPOSURE_TIME = 2937
    exposure_time_max = inq.maximum_exposure * 4
    for filt in ('r', 'g', 'b', 'i'):
        out.append(Parameter(Option(
            name=f'exp_time_{filt}',
            type=int,
            unit=Unit.TIMER_COUNTS,
            validate=lambda v: inq.minimum_exposure <= v <= exposure_time_max,
            default=DEFAULT_EXPOSURE_TIME
        )))

    # RELATIVE percentage on top of the absolute exposure time, a 16-bit field per
    # filter sent by SCSI_EXPOSURE (pieusb_scancmd.c:521-544). Infrared has no
    # entry: the C struct holds three colours.
    #
    # LEAVE THIS AT 100. Exposed for experiments, not for tuning:
    #
    #   - SANE hard-codes all three to 100 and never varies them (pieusb.c:878-882,
    #     927), so no other value has been exercised against this hardware.
    #   - Redundant with exp_time_*, which has a known range and meaning.
    #   - The bound below is the width of the wire field, not a documented limit;
    #     what the device does outside 100 is unknown.
    #   - The gain and exposure times auto_exp adopts were optimised by the
    #     firmware with this at 100, so moving it invalidates them.
    #
    # OptionsTable.validate() warns if you move it, rather than refusing.
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

    # Lamp level, byte 15 of SET GAIN OFFSET. Sent on every scan and it scales the
    # whole acquisition, so a wrong value starves everything downstream.
    #
    #   "Current light level. The stability of the light source is tested during
    #    warming up. The check starts with a light value 7 or 6, and decrements
    #    it when the light warms up. At a light value of 4, the scanner produces
    #    stable scans (i.e. successive 'white' scan values don't differ more than
    #    0x200)." (pieusb_scancmd.h:208-213)
    #
    # So the operating band is 4..7, with 4 the warmed-up value; SANE types it as a
    # duration in microseconds (pieusb_specific.c:916). The range below is the
    # width of the wire field, not a documented limit. auto_exp adopts the device's
    # own value over this default (Scanner._adopt_device_calibration).
    out.append(Parameter(Option(
        name='light',
        type=int,
        unit=Unit.MICROSECONDS,
        validate=lambda v: 0 <= v <= 0xFF,
        default=DEFAULT_LIGHT
    )))

    return OptionsTable(out, inq)

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
    dev.command(SCSI_WRITE_GAIN_OFFSET, out_data=payload, cdb_length=29)

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
