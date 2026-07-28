from collections.abc import Callable
from dataclasses import dataclass
from typing import Generic, TypeVar
from enum import Enum

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
    def __init__(self, params: list[Parameter]) -> None:
        self.table = params

    def __getitem__(self, key) -> Parameter:
        try:
            out = next(filter(lambda o: o.opt.name == key, self.table))
            return out
        except StopIteration:
            raise KeyError(f"No parameter named {key} exists")

    def validate(self) -> None:
        '''
        Cross check parameter values
        Raise if any of them are incompatible with eachother
        '''
        if self['tl_x'].value >= self['br_x'].value:
            raise ParamError(f"Parameter 'tl_x' ({self['tl_x'].value}px) must be smaller than parameter 'br_x' ({self['br_x'].value}px)")

        if self['tl_y'].value >= self['br_y'].value:
            raise ParamError(f"Parameter 'tl_y' ({self['tl_y'].value}px) must be smaller than parameter 'br_y' ({self['br_y'].value}px)")

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

    return OptionsTable(out)

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