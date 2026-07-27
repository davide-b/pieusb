from pieusb.usb_utils import find_device, get_device
from pieusb.transport import UASDevice, SCSI_INQUIRY
from pieusb.scanner import Scanner

from pieusb.inquiry import KNWON_PIDS, PIE_VENDOR_ID, parse_inquiry
from pieusb.types import DeviceInfo
from pieusb.scanner import Scanner

def _get_inquiry(dev: UASDevice):# -> InquiryResponse:
    header = dev.command(SCSI_INQUIRY, in_size=5, cdb_length=5)
    additional_length = header[4]
    total_size = additional_length + 4
    raw = dev.command(SCSI_INQUIRY, in_size=total_size, cdb_length=total_size)
    return parse_inquiry(raw)

def get_devices() -> list[DeviceInfo]:
    devices: list[DeviceInfo] = []

    for pid in KNWON_PIDS:
        dev = find_device(PIE_VENDOR_ID, pid)

        if dev is not None:
            with UASDevice(dev) as d:
                r = _get_inquiry(d)
                devices.append(DeviceInfo(
                    dev=dev,
                    vendor=r.vendor,
                    model=r.model_str,
                    inquiry=r
                ))

    return devices

def open(dev: DeviceInfo) -> Scanner:
    return Scanner(dev)