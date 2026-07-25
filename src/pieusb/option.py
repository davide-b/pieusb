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
    SCSI_EXPOSURE
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
    value: T

class OptionsTable:
    def __init__(self, options: list[Option]) -> None:
        self.table = options

    def __getitem__(self, key) -> Option:
        return next(filter(lambda o: o.name == key, self.table))

def generate_options(inq: InquiryResponse) -> OptionsTable:
    out = []

    modes = []
    if Filter.GREEN in inq.filters:
        modes.append('gray')
    if set(Filter.RED, Filter.GREEN, Filter.BLUE) <= set(inq.filters):
        modes.append('rbg')
    if set(Filter.RED, Filter.GREEN, Filter.BLUE, Filter.INFRARED) <= set(inq.filters):
        modes.append('rbgi')
    out.append(Option(
        name='mode',
        type=str,
        unit=Unit.NONE,
        validate=lambda v: v in modes,
        value='rgb'
    ))

    # Bit depth of the scan
    out.append(Option(
        name='color_depth',
        type=int,
        unit=Unit.BITS,
        validate=lambda v: v in inq.color_depths,
        value=16
    ))

    # Scan resolution
    out.append(Option(
        name='resolution',
        type=int,
        unit=Unit.NONE,
        validate=lambda v: v < min(inq.max_resolution_x, inq.max_resolution_y),
        value=300
    ))

    # Halftone

    # Increase sharpness by giving more time to the CCD to discharge between each line
    out.append(Option(
        name='sharpen',
        type=bool,
        unit=Unit.NONE,
        validate=lambda v: type(v) is bool,
        value=False
    ))

    # Force calibration of the CCD sensor
    out.append(Option(
        name='calibrate',
        type=bool,
        unit=Unit.NONE,
        validate=lambda v: type(v) is bool,
        value=False
    ))

    # Perform a preview pass to determine best exposure parameters
    out.append(Option(
        name='auto_exp',
        type=bool,
        unit=Unit.NONE,
        validate=lambda v: type(v) is bool,
        value=False
    ))

    # Advance slide after scan
    out.append(Option(
        name='advance',
        type=bool,
        unit=Unit.NONE,
        validate=lambda v: inq.slide_transport,
        value=False
    ))

    # X coordinate of the top-left corner
    out.append(Option(
        name='tl_x',
        type=int,
        unit=Unit.PIXEL,
        validate=lambda v: v >= 0,
        value=0
    ))

    # Y coordinate of the top-left corner
    out.append(Option(
        name='tl_y',
        type=int,
        unit=Unit.PIXEL,
        validate=lambda v: v >= 0,
        value=0
    ))

    # X coordinate of the bottom-right corner
    out.append(Option(
        name='br_x',
        type=int,
        unit=Unit.PIXEL,
        validate=lambda v: v <= inq.max_scan_w,
        value=inq.max_scan_w
    ))

    # Y coordinate of the bottom-right corner
    out.append(Option(
        name='br_y',
        type=int,
        unit=Unit.PIXEL,
        validate=lambda v: v <= inq.max_scan_h,
        value=inq.max_scan_h
    ))

    # SANE exposure default 2937
    # Though setting the option does nothing as the value is hardcoded to 100 per channel
    # Exposure for red channel
    out.append(Option(
        name='exp_r',
        type=int,
        unit=Unit.MICROSECONDS,
        validate=lambda v: v >= inq.minimum_exposure and v <= inq.maximum_exposure, # SANE multiplies the max by 4
        value=100
    ))
    
    # Exposure for green channel
    out.append(Option(
        name='exp_g',
        type=int,
        unit=Unit.MICROSECONDS,
        validate=lambda v: v >= inq.minimum_exposure and v <= inq.maximum_exposure, # SANE multiplies the max by 4
        value=100
    ))
    
    # Exposure for blue channel
    out.append(Option(
        name='exp_b',
        type=int,
        unit=Unit.MICROSECONDS,
        validate=lambda v: v >= inq.minimum_exposure and v <= inq.maximum_exposure, # SANE multiplies the max by 4
        value=100
    ))
    
    # Exposure for infrared channel
    out.append(Option(
        name='exp_i',
        type=int,
        unit=Unit.MICROSECONDS,
        validate=lambda v: v >= inq.minimum_exposure and v <= inq.maximum_exposure, # SANE multiplies the max by 4
        value=100
    ))

    # SANE gain default 19
    # Gain for red channel
    out.append(Option(
        name='gain_r',
        type=int,
        unit=Unit.NONE,
        validate=lambda v: v >= 0 and v < 64, # From firmware disassembly
        value=19
    ))
    
    # Gain for green channel
    out.append(Option(
        name='gain_g',
        type=int,
        unit=Unit.NONE,
        validate=lambda v: v >= 0 and v < 64, # From firmware disassembly
        value=19
    ))
    
    # Gain for blue channel
    out.append(Option(
        name='gain_b',
        type=int,
        unit=Unit.NONE,
        validate=lambda v: v >= 0 and v < 64, # From firmware disassembly
        value=19
    ))
    
    # Gain for infrared channel
    out.append(Option(
        name='gain_i',
        type=int,
        unit=Unit.NONE,
        validate=lambda v: v >= 0 and v < 64, # From firmware disassembly
        value=19
    ))

    # SANE offset default 0
    # Offset for the red channel
    out.append(Option(
        name='offset_r',
        type=int,
        unit=Unit.NONE,
        validate=lambda v: v >= 0 and v < 255, # From firmware disassembly
        value=0
    ))

    # Offset for the green channel
    out.append(Option(
        name='offset_g',
        type=int,
        unit=Unit.NONE,
        validate=lambda v: v >= 0 and v < 255, # From firmware disassembly
        value=0
    ))
    
    # Offset for the blue channel
    out.append(Option(
        name='offset_b',
        type=int,
        unit=Unit.NONE,
        validate=lambda v: v >= 0 and v < 255, # From firmware disassembly
        value=0
    ))
    
    # Offset for the infrared channel
    out.append(Option(
        name='offset_i',
        type=int,
        unit=Unit.NONE,
        validate=lambda v: v >= 0 and v < 255, # From firmware disassembly
        value=0
    ))

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