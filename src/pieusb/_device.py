import logging

from pieusb.usb_utils import find_device
from pieusb.transport import UASDevice, SCSI_INQUIRY
from pieusb.scanner import Scanner

from pieusb.inquiry import KNOWN_PIDS, PIE_VENDOR_ID, InquiryResponse, parse_inquiry
from pieusb.types import DeviceInfo

log = logging.getLogger(__name__)

def _get_inquiry(dev: UASDevice) -> InquiryResponse:
    header = dev.command(SCSI_INQUIRY, in_size=5, cdb_length=5)
    additional_length = header[4]
    total_size = additional_length + 4
    raw = dev.command(SCSI_INQUIRY, in_size=total_size, cdb_length=total_size)
    return parse_inquiry(raw)

def get_devices() -> list[DeviceInfo]:
    """Every supported scanner attached, identified by INQUIRY.

    A scanner that cannot be identified is logged and skipped, so it does not
    hide the others. If none could be identified, the first error is raised
    rather than returning an empty list.
    """
    devices: list[DeviceInfo] = []
    first_error: Exception | None = None

    for pid in KNOWN_PIDS:
        dev = find_device(PIE_VENDOR_ID, pid)

        if dev is None:
            continue

        try:
            with UASDevice(dev) as d:
                r = _get_inquiry(d)
        except Exception as e:
            log.warning(f"{PIE_VENDOR_ID:04x}:{pid:04x} is attached but could not be "
                        f"identified, skipping it: {e}")
            log.debug("[get_devices] INQUIRY failed", exc_info=True)
            if first_error is None:
                first_error = e
            continue

        devices.append(DeviceInfo(
            dev=dev,
            vendor=r.vendor,
            model=r.model_str,
            inquiry=r
        ))

    if not devices and first_error is not None:
        raise first_error

    return devices

# Convenience alias; Scanner(dev) or `with Scanner(dev) as s` does the same, and
# opens the device in the constructor either way.
def open(dev: DeviceInfo) -> Scanner:
    return Scanner(dev)