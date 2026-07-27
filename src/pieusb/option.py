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
    SCSI_HIGHLIGHT_SHADOW,
    SCSI_EXPOSURE,
    SCSI_SCAN_FRAME,
    SCSI_WRITE_GAIN_OFFSET
)

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

    # Increase sharpness by giving more time to the CCD to discharge between each line
    out.append(Parameter(Option(
        name='sharpen',
        type=bool,
        unit=Unit.NONE,
        validate=lambda v: type(v) is bool,
        default=False
    )))

    # Force calibration of the CCD sensor
    out.append(Parameter(Option(
        name='calibrate',
        type=bool,
        unit=Unit.NONE,
        validate=lambda v: type(v) is bool,
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

    # Set mode
    quality = 0
    if options['sharpen'].value:
        quality |= 0x02
    if not options['calibrate'].value:
        quality |= 0x08
    struct.pack('<BBHBBBBBBBB',
        0,
        15, # Mode size, for some reason
        options['resolution'].value,
        0, # TODO: passes
        options['color_depth'].value,
    )