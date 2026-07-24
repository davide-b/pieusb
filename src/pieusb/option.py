from collections.abc import Callable
from dataclasses import dataclass
from typing import Generic, TypeVar
from enum import Enum

from pieusb.inquiry import (
    InquiryResponse,
    Filter
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

def generate_options(inq: InquiryResponse) -> list[Option]:
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
        validate=lambda v: v in modes
    ))

    # Bit depth of the scan
    out.append(Option(
        name='color_depth',
        type=int,
        unit=Unit.BITS,
        validate=lambda v: v in inq.color_depths
    ))

    # Scan resolution
    out.append(Option(
        name='resolution',
        type=int,
        unit=Unit.NONE,
        validate=lambda v: v < min(inq.max_resolution_x, inq.max_resolution_y)
    ))

    # Halftone

    # Increase sharpness by giving more time to the CCD to discharge between each line
    out.append(Option(
        name='sharpen',
        type=bool,
        unit=Unit.NONE,
        validate=lambda v: type(v) is bool
    ))

    # Force calibration of the CCD sensor
    out.append(Option(
        name='calibrate',
        type=bool,
        unit=Unit.NONE,
        validate=lambda v: type(v) is bool
    ))

    # Perform a preview pass to determine best exposure parameters
    out.append(Option(
        name='auto_exp',
        type=bool,
        unit=Unit.NONE,
        validate=lambda v: type(v) is bool
    ))

    # Advance slide after scan
    out.append(Option(
        name='advance',
        type=bool,
        unit=Unit.NONE,
        validate=lambda v: inq.slide_transport
    ))

    # X coordinate of the top-left corner
    out.append(Option(
        name='tl_x',
        type=int,
        unit=Unit.PIXEL,
        validate=lambda v: v >= 0
    ))

    # Y coordinate of the top-left corner
    out.append(Option(
        name='tl_y',
        type=int,
        unit=Unit.PIXEL,
        validate=lambda v: v >= 0
    ))

    # X coordinate of the bottom-right corner
    out.append(Option(
        name='br_x',
        type=int,
        unit=Unit.PIXEL,
        validate=lambda v: v <= inq.max_scan_w
    ))

    # Y coordinate of the bottom-right corner
    out.append(Option(
        name='br_y',
        type=int,
        unit=Unit.PIXEL,
        validate=lambda v: v <= inq.max_scan_h
    ))

    # Exposure for red channel
    out.append(Option(
        name='exp_r',
        type=int,
        unit=Unit.MICROSECONDS,
        validate=lambda v: v >= inq.minimum_exposure and v <= inq.maximum_exposure # SANE multiplies the max by 4
    ))
    
    # Exposure for green channel
    out.append(Option(
        name='exp_g',
        type=int,
        unit=Unit.MICROSECONDS,
        validate=lambda v: v >= inq.minimum_exposure and v <= inq.maximum_exposure # SANE multiplies the max by 4
    ))
    
    # Exposure for blue channel
    out.append(Option(
        name='exp_b',
        type=int,
        unit=Unit.MICROSECONDS,
        validate=lambda v: v >= inq.minimum_exposure and v <= inq.maximum_exposure # SANE multiplies the max by 4
    ))
    
    # Exposure for infrared channel
    out.append(Option(
        name='exp_i',
        type=int,
        unit=Unit.MICROSECONDS,
        validate=lambda v: v >= inq.minimum_exposure and v <= inq.maximum_exposure # SANE multiplies the max by 4
    ))

    # Gain for red channel
    out.append(Option(
        name='gain_r',
        type=int,
        unit=Unit.NONE,
        validate=lambda v: v >= 0 and v < 64 # From firmware disassembly
    ))
    
    # Gain for green channel
    out.append(Option(
        name='gain_g',
        type=int,
        unit=Unit.NONE,
        validate=lambda v: v >= 0 and v < 64 # From firmware disassembly
    ))
    
    # Gain for blue channel
    out.append(Option(
        name='gain_b',
        type=int,
        unit=Unit.NONE,
        validate=lambda v: v >= 0 and v < 64 # From firmware disassembly
    ))
    
    # Gain for infrared channel
    out.append(Option(
        name='gain_i',
        type=int,
        unit=Unit.NONE,
        validate=lambda v: v >= 0 and v < 64 # From firmware disassembly
    ))

    # Offset for the red channel
    out.append(Option(
        name='offset_r',
        type=int,
        unit=Unit.NONE,
        validate=lambda v: v >= 0 and v < 255 # From firmware disassembly
    ))

    # Offset for the green channel
    out.append(Option(
        name='offset_g',
        type=int,
        unit=Unit.NONE,
        validate=lambda v: v >= 0 and v < 255 # From firmware disassembly
    ))
    
    # Offset for the blue channel
    out.append(Option(
        name='offset_b',
        type=int,
        unit=Unit.NONE,
        validate=lambda v: v >= 0 and v < 255 # From firmware disassembly
    ))
    
    # Offset for the infrared channel
    out.append(Option(
        name='offset_i',
        type=int,
        unit=Unit.NONE,
        validate=lambda v: v >= 0 and v < 255 # From firmware disassembly
    ))
