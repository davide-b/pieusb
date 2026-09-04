from pieusb.types import (
    Capabilities,
    ExposureControl,
    InquiryResponse,
    Filter,
    ColorFormat,
    ColorFormat,
    ImageFormat,
    ScanCapabilities,
    OptionalDevices,
    DeviceInfo
)

import struct

PIE_VENDOR_ID = 0x05E3

KNOWN_PIDS = [0x0142, 0x0143, 0x0144, 0x0145]

# Model number, INQUIRY offset 116.
MODEL_NAMES = {
    0x002c: 'ProScan 4000',
    0x003a: 'DigitDia 6000 Multiple Slide Scanner',
    0x0030: 'CrystalScan 7200',
    0x0036: 'ProScan 7200',
    0x0047: 'ProScan 10T',
    0x002e: 'CrystalScan 3600',
    0x002f: 'DigitDia 4000',
    0x0048: 'RPS 10M (aka Pacific Image PrimeFilm XAs)',
}

# Per-model knowledge, in the spirit of SANE's pieusb.conf flags. A model absent
# here gets the conservative Capabilities() defaults.
DEVICE_CAPABILITIES = {
    0x002c: Capabilities(              # ProScan 4000
        film_transport=True,
        focus=True,
        exposure_control=ExposureControl.ABSOLUTE,
    ),
    0x002f: Capabilities(film_transport=True),   # DigitDia 4000
    0x003a: Capabilities(film_transport=True),   # DigitDia 6000
}

def capabilities_for(model_nr: int) -> Capabilities:
    return DEVICE_CAPABILITIES.get(model_nr, Capabilities())

def parse_filters(byte: int) -> tuple[Filter, ...]:
    out = []

    if byte & 0x01:
        out.append(Filter.NEUTRAL)
    if byte & 0x02:
        out.append(Filter.RED)
    if byte & 0x04:
        out.append(Filter.GREEN)
    if byte & 0x08:
        out.append(Filter.BLUE)
    if byte & 0x10:
        out.append(Filter.INFRARED)

    return tuple(out)

def parse_color_depths(byte: int) -> tuple[int, ...]:
    out = []

    if byte & 0x01:
        out.append(1)
    if byte & 0x02:
        out.append(4)
    if byte & 0x04:
        out.append(8)
    if byte & 0x08:
        out.append(10)
    if byte & 0x10:
        out.append(12)
    if byte & 0x20:
        out.append(16)

    return tuple(out)

def parse_color_format(byte: int) -> tuple[ColorFormat, ...]:
    out = []

    if byte & 0x01:
        out.append(ColorFormat.PIXEL)
    if byte & 0x02:
        out.append(ColorFormat.LINE)
    if byte & 0x04:
        out.append(ColorFormat.INDEXED)

    return tuple(out)

def parse_image_format(byte: int) -> tuple[ImageFormat, ...]:
    out = []

    if byte & 0x01:
        out.append(ImageFormat.INTEL)
    if byte & 0x02:
        out.append(ImageFormat.MOTOROLA)
    if byte & 0x04:
        out.append(ImageFormat.BLACKONE)
    if byte & 0x08:
        out.append(ImageFormat.OKLINE)

    return tuple(out)

def parse_scan_capabilities(byte: int) -> tuple[ScanCapabilities, ...]:
    out = []

    if byte & 0x80:
        out.append(ScanCapabilities.PWRSAV)
    if byte & 0x40:
        out.append(ScanCapabilities.EXT_CAL)
    if byte & 0x10:
        out.append(ScanCapabilities.FAST_PREVIEW)
    if byte & 0x08:
        out.append(ScanCapabilities.DISABLE_CAL)

    return tuple(out)

def parse_optional_devices(byte: int) -> tuple[OptionalDevices, ...]:
    out = []

    if byte & 0x80:
        out.append(OptionalDevices.DEV_MPCL)
    if byte & 0x04:
        out.append(OptionalDevices.DEV_TP1)
    if byte & 0x02:
        out.append(OptionalDevices.DEV_TP)
    if byte & 0x01:
        out.append(OptionalDevices.DEV_ADF)

    return tuple(out)

def parse_inquiry(raw: bytes) -> InquiryResponse:
    def txt(b):
        return b.rstrip(b"\x00").decode("ascii", errors="replace").strip()

    model_nr = struct.unpack_from("<H", raw, 116)[0]
    if model_nr in MODEL_NAMES:
        model_str = MODEL_NAMES[model_nr]
    else:
        # model_str reaches the user via DeviceInfo.model; keep the number visible.
        model_str = f'Unknown (model 0x{model_nr:04x})'
    return InquiryResponse(
        vendor=txt(raw[8:16]),
        product=txt(raw[16:32]),
        revision=txt(raw[32:36]),
        max_resolution_x=struct.unpack_from("<H", raw, 36)[0],
        max_resolution_y=struct.unpack_from("<H", raw, 38)[0],
        max_scan_w=struct.unpack_from("<H", raw, 40)[0],
        max_scan_h=struct.unpack_from("<H", raw, 42)[0],
        filters=parse_filters(raw[44]),
        color_depths=parse_color_depths(raw[45]),
        color_formats=parse_color_format(raw[46]),
        image_formats=parse_image_format(raw[48]),
        scan_capabilities=parse_scan_capabilities(raw[49]),
        scan_speeds=raw[49] & 7,
        optional_devices=parse_optional_devices(raw[50]),
        enhancements=raw[51],
        gamma_bits=raw[52],
        last_filter=raw[53],
        preview_scan_resolution=struct.unpack_from("<H", raw, 54)[0],
        halftones=raw[100],
        minimum_highlight=raw[101],
        maximum_shadow=raw[102],
        calibration_equation=raw[103],
        maximum_exposure=struct.unpack_from("<H", raw, 104)[0],
        minimum_exposure=struct.unpack_from("<H", raw, 106)[0],
        x0=struct.unpack_from("<H", raw, 108)[0],
        x1=struct.unpack_from("<H", raw, 110)[0],
        y0=struct.unpack_from("<H", raw, 112)[0],
        y1=struct.unpack_from("<H", raw, 114)[0],
        model=model_nr,
        model_str=model_str,
        production=txt(raw[120:124]),
        timestamp=txt(raw[124:144]),
        signature=txt(raw[144:184]),
        slide_transport=model_nr == 0x3a or model_nr == 0x2f
    )