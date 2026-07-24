from dataclasses import dataclass
from enum import StrEnum

class Filter(StrEnum):
    INFRARED = 'Infrared'
    RED = 'Red'
    GREEN = 'Green'
    BLUE = 'Blue'
    NEUTRAL = 'Neutral'

class ColorFormat(StrEnum):
    INDEXED = 'Indexed'
    LINE = 'Line'
    PIXEL = 'Pixel'

class ImageFormat(StrEnum):
    OKLINE = 'OKLine'
    BLACKONE = 'BlackOne'
    MOTOROLA = 'Motorola'
    INTEL = 'Intel'

class ScanCapabilities(StrEnum):
    PWRSAV = "PowerSave"
    EXT_CAL = "ExtCal"
    FAST_PREVIEW = "FastPreview"
    DISABLE_CAL = "DisCal"

class OptionalDevices(StrEnum):
    DEV_MPCL = "MultiPageLoad"
    DEV_TP1 = "TransModule1"
    DEV_TP = "TransModule"
    DEV_ADF = "ADF"

@dataclass(frozen=True)
class InquiryResponse:
    vendor: str
    product: str
    revision: str
    max_resolution_x: int
    max_resolution_y: int
    max_scan_w: int
    max_scan_h: int
    filters: tuple[Filter, ...]
    color_depths: tuple[int, ...]
    color_formats: tuple[ColorFormat, ...]
    image_formats: tuple[ImageFormat, ...]
    scan_capabilities: tuple[ScanCapabilities, ...]
    scan_speeds: int
    optional_devices: tuple[OptionalDevices, ...]
    enhancements: int
    gamma_bits: int
    last_filter: int
    preview_scan_resolution: int
    halftones: int
    minimum_highlight: int
    maximum_shadow: int
    calibration_equation: int
    maximum_exposure: int
    minimum_exposure: int
    x0: int
    x1: int
    y0: int
    y1: int
    model: int
    model_str: str
    production: str
    timestamp: str
    signature: str
    slide_transport: bool

# @dataclass(frozen=True)
# class Capabilities:
#     ir_channel: bool
#     supported_dpi: tuple[int, ...]
#     supported_depths: tuple[int, ...]
#     multi_sample: bool
#     adapter_frame_capacity: int | None
#     adapter_frame_control: bool
#     auto_exposure: bool
#     registered_geometry: bool
#     can_eject: bool

@dataclass(frozen=True)
class DeviceInfo:
    id: str
    vendor: str
    model: str
    inquiry: InquiryResponse