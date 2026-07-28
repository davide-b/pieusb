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
# sends per image. Mirrors MODE_PLANES in the PoC (poc:462).
MODE_PLANES = {'gray': 1, 'rgb': 3, 'rgbi': 4}

# Modes this library can read back but not yet decode. 'gray' maps to
# SCAN_COLOR_FORMAT_PIXEL, so its lines carry no 'RR'/'GG'/'BB'/'II' tag and the
# tag-based deinterleave in Scanner does not apply -- per pieusb_scancmd.h:165-173
# the data comes back as RGB pixel triples of which only the first is valid.
# Nothing has ever exercised that layout on hardware, so rather than guess at it
# in the worker, validate() refuses the mode outright. See TODO 12a.
UNSUPPORTED_MODES = ('gray',)

# Sample sizes the deinterleave in Scanner._run_scan understands. INQUIRY can
# advertise 1/4/10/12 as well, but only 8 and 16 map cleanly onto a numpy dtype
# and only those two have been seen on the wire.
SUPPORTED_COLOR_DEPTHS = (8, 16)

# 'colorDepth' bitmask, pieusb_specific.h:64-70
COLOR_DEPTHS = {1: 0x01, 4: 0x02, 8: 0x04, 10: 0x08, 12: 0x10, 16: 0x20}

# 'lineThreshold', where 0xFF is 100%. Only meaningful for the lineart/halftone
# modes, which aren't supported yet. The C backend derives it from OPT_THRESHOLD
# (pieusb_specific.c:1864); 128 is what the cyberview capture quoted in
# pieusb_scancmd.c:757 sends, and what the PoC uses.
LINE_THRESHOLD = 128

class Unit(Enum):
    MM = 0
    PIXEL = 1
    BITS = 2
    MICROSECONDS = 3
    NONE = 4

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
        # Kept so validate() can re-check the frame against the device's own
        # reported bed rather than trusting the per-option validators alone.
        self.inq = inq

    def __getitem__(self, key) -> Parameter:
        try:
            out = next(filter(lambda o: o.opt.name == key, self.table))
            return out
        except StopIteration:
            raise KeyError(f"No parameter named {key} exists")

    def validate(self) -> None:
        '''Validate the table as a whole, before anything reaches the device.

        Two jobs, in order:

        1. Re-run every per-option validator. Values normally arrive through
           Scanner.__setitem__, which validates -- but the attribute interface
           DESIGN.md specifies does not exist yet, and any future write path that
           misses the check would otherwise reach SET SCAN FRAME unvalidated.
           Getting the frame wrong has already cost this project one carriage
           crash, so this re-checks rather than trusts.
        2. The cross-option checks a single-option validator structurally cannot
           express -- the frame corners against each other and against the bed,
           and the mode/quality-bit combinations.

        Raises ParamError, naming the offending option(s). Combinations that are
        merely ineffective rather than contradictory are logged as warnings, the
        way sanei_pieusb_analyse_options does (pieusb_specific.c:1518-1620).
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

        # The frame against the bed the device itself reports (INQUIRY offsets
        # 40/42, native-resolution units). The PoC added the same check after a
        # real carriage crash caused by an X/Y axis mix-up (poc:690-703): X is the
        # long axis, Y the short one, and swapping them drives the head off its
        # rail. Asserted here even though br_x/br_y have their own validators,
        # because this is the last gate before SET SCAN FRAME.
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

        # 'sharpen' is documented as "only effective with fastInfrared off"
        # (pieusb_scancmd.h:180), so asking for both is a contradiction rather
        # than a preference -- the scanner would silently drop one of them.
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
        if self['auto_exp'].value:
            log.warning("option 'auto_exp' is not implemented yet and will be ignored")
        if self['advance'].value:
            log.warning("option 'advance' is not implemented yet and will be ignored: "
                        "slide-transport commands are not sent (TODO 12c)")

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
        validate=lambda v: v < min(inq.max_resolution_x, inq.max_resolution_y),
        default=300
    )))

    # Halftone

    # Increase sharpness by giving more time to the CCD to discharge between each line.
    # Only effective with 'fast_infrared' off and a one-pass colour mode
    # (pieusb_scancmd.h:180).
    out.append(Parameter(Option(
        name='sharpen',
        type=bool,
        unit=Unit.NONE,
        validate=lambda v: type(v) is bool,
        default=False
    )))

    # Collect shading (flat-field) information as part of the scan. Turning this
    # off sets the 'skip calibration' quality bit, which makes the scanner reject
    # the subsequent CCD MASK and GET PARAMETERS commands as invalid -- the whole
    # read sequence in Scanner.scan() depends on the shading pass having run.
    out.append(Parameter(Option(
        name='calibrate',
        type=bool,
        unit=Unit.NONE,
        validate=lambda v: type(v) is bool,
        default=True
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

    # Perform a preview pass to determine best exposure parameters
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

    # SANE exposure default 2937
    # Though setting the option does nothing as the value is hardcoded to 100 per channel
    # Exposure for red channel
    out.append(Parameter(Option(
        name='exp_r',
        type=int,
        unit=Unit.MICROSECONDS,
        validate=lambda v: v >= inq.minimum_exposure and v <= inq.maximum_exposure, # SANE multiplies the max by 4
        default=100
    )))
    
    # Exposure for green channel
    out.append(Parameter(Option(
        name='exp_g',
        type=int,
        unit=Unit.MICROSECONDS,
        validate=lambda v: v >= inq.minimum_exposure and v <= inq.maximum_exposure, # SANE multiplies the max by 4
        default=100
    )))
    
    # Exposure for blue channel
    out.append(Parameter(Option(
        name='exp_b',
        type=int,
        unit=Unit.MICROSECONDS,
        validate=lambda v: v >= inq.minimum_exposure and v <= inq.maximum_exposure, # SANE multiplies the max by 4
        default=100
    )))
    
    # Exposure for infrared channel
    out.append(Parameter(Option(
        name='exp_i',
        type=int,
        unit=Unit.MICROSECONDS,
        validate=lambda v: v >= inq.minimum_exposure and v <= inq.maximum_exposure, # SANE multiplies the max by 4
        default=100
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

    return OptionsTable(out, inq)

def set_options(dev: UASDevice, options: OptionsTable) -> None:
    # Hard-coded in SANE. Experiment with setting different values
    for filt, value in ((0x02, 100), (0x04, 100), (0x08, 100)):
        payload = struct.pack("<HHHH", SCSI_HIGHLIGHT_SHADOW, 4, filt, value)
        dev.command(SCSI_WRITE, out_data=payload, cdb_length=8)

    # Hard-coded to 100 in SANE. The auto exposure usually uses the gain instead
    r = options['exp_r'].value
    g = options['exp_g'].value
    b = options['exp_b'].value
    for filt, value in ((0x02, r), (0x04, g), (0x08, b)):
        payload = struct.pack("<HHHH", SCSI_EXPOSURE, 4, filt, value)
        dev.command(SCSI_WRITE, out_data=payload, cdb_length=8)

    index = 128 # Trust me bro
    x0 = options["tl_x"].value
    y0 = options["tl_y"].value
    x1 = options["br_x"].value
    y1 = options["br_y"].value
    payload = struct.pack("<HHHHHHH", SCSI_SCAN_FRAME, 10, index, x0, y0, x1, y1)
    dev.command(SCSI_WRITE, out_data=payload, cdb_length=14)

    # Set exposure, gain and offset
    payload = struct.pack('<HHHBBBBBBBBBBBBHBBBBBBBBB',
        options['exp_r'].value,
        options['exp_g'].value,
        options['exp_b'].value,
        options['offset_r'].value,
        options['offset_g'].value,
        options['offset_b'].value,
        0,
        0,
        0,
        options['gain_r'].value,
        options['gain_g'].value,
        options['gain_b'].value,
        0, # Light, maybe in SANE is 5?
        0, # Extra entried
        0, # Double times
        options['exp_i'].value,
        options['offset_i'].value,
        0,
        options['gain_i'].value,
        0, 0, 0, 0, 0, 0
    )
    dev.command(SCSI_WRITE_GAIN_OFFSET, out_data=payload, cdb_length=29)

    # Set mode -- MODE SELECT, a fixed 16-byte payload built byte by byte, per
    # sanei_pieusb_cmd_set_mode() (pieusb_scancmd.c:731-800). Bytes 0, 7, 10, 11
    # and 15 are unused and stay zero.
    passes, color_format = MODE_SETTINGS[options['mode'].value]

    # Quality bitmask, byte 9 (pieusb_scancmd.c:790-794)
    quality = 0
    if options['sharpen'].value:
        quality |= 0x02
    if not options['calibrate'].value:
        # skipShadingAnalysis. The C backend sets this bit when shading analysis
        # is *not* wanted: skipShadingAnalysis = !OPT_SHADING_ANALYSIS
        # (pieusb_specific.c:1857), so 'calibrate' inverts into it.
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
    payload[14] = 0x10 # Unexplained, but sent unconditionally by the C backend
                       # (pieusb_scancmd.c:797) and present in the cyberview capture
    dev.command(SCSI_MODE_SELECT, out_data=bytes(payload), cdb_length=16)