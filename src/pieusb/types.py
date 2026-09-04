from dataclasses import dataclass, field
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

class ExposureControl(StrEnum):
    '''Which field actually changes a scan's exposure on a given model.

    RELATIVE is exp_rel_*, sent by SCSI_EXPOSURE. ABSOLUTE is exp_time_*, carried
    by SET GAIN OFFSET. A model honours one and ignores the other.
    '''
    RELATIVE = 'Relative'
    ABSOLUTE = 'Absolute'

@dataclass(frozen=True)
class Capabilities:
    '''Per-model facts INQUIRY does not report, keyed on model number in
    inquiry.DEVICE_CAPABILITIES.'''
    # Whether the hardware has a film transport. Scanner does not drive the
    # ProScan 4000's; InquiryResponse.slide_transport is that gate.
    film_transport: bool = False
    # Whether SCSI_SLIDE action 0x10 moves a focus motor.
    focus: bool = False
    exposure_control: ExposureControl = ExposureControl.RELATIVE

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
    capabilities: Capabilities = field(default_factory=Capabilities)

class ScanPhase(StrEnum):
    # Never emitted. Kept so consumers matching on it still compile.
    METERING = 'Metering'
    CONFIGURING = 'Configuring'
    WARMING_UP = 'Warming up'
    CALIBRATING = 'Calibrating'
    SCANNING = 'Scanning'
    PROCESSING = 'Post process'

@dataclass(frozen=True)
class UpdateData:
    '''Progress within one phase of a scan, delivered to scan()'s on_update.

    `progress` runs 0.0 -> 1.0 within `phase` and says nothing about the scan as a
    whole: each phase sweeps the range again, and how many phases a scan has
    depends on the options (a reused calibration removes CALIBRATING). A phase
    whose length the device does not report in advance stays at 0.0 throughout, so
    treat a phase change as the event and `progress` as detail within it.
    '''
    phase: ScanPhase
    progress: float = 0.0

@dataclass(frozen=True)
class ScanResult:
    '''What a finished scan reports, delivered exactly once per scan().

    `rgb` is (height, width, 3) and `ir` is (height, width); both are None if the
    scan was cancelled or failed. `error` is the only channel through which a
    worker-thread failure reaches the caller, so check it before using `rgb`.

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