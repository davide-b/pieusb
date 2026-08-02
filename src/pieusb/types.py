from dataclasses import dataclass
from enum import StrEnum
import usb.core
import numpy

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

@dataclass(frozen=True)
class DeviceInfo:
    dev: usb.core.Device
    vendor: str
    model: str
    inquiry: InquiryResponse

class ScanPhase(StrEnum):
    # Every phase of the auto-exposure preview pass reports as METERING, so a
    # progress bar sees one 0->100% sweep for it and a second for the real scan,
    # rather than two indistinguishable SCANNING sweeps.
    METERING = 'Metering'
    CONFIGURING = 'Configuring'
    WARMING_UP = 'Warming up'
    CALIBRATING = 'Calibrating'
    SCANNING = 'Scanning'
    PROCESSING = 'Post process'

@dataclass(frozen=True)
class UpdateData:
    phase: ScanPhase
    scanned_lines: int | None = None
    total_lines: int | None = None

@dataclass(frozen=True)
class ScanResult:
    '''What a finished scan reports, delivered exactly once per scan().

    `rgb` is (height, width, 3) and `ir` is (height, width); both are None if the
    scan was cancelled or failed, so `error` is the only channel through which a
    worker-thread failure reaches the caller -- check it before using `rgb`.

    `width`/`height` come from the device's own GET PARAMETERS mid-scan, not from
    the requested frame, and can differ from what the options asked for.
    '''
    rgb: numpy.ndarray | None
    ir: numpy.ndarray | None = None
    width: int = 0
    height: int = 0
    mode: str = ''
    color_depth: int = 0
    resolution: int = 0
    shading_corrected: bool = False
    cancelled: bool = False
    error: Exception | None = None
    duration_s: float = 0.0