import usb.core
import usb.util
from usb.backend import IBackend

import ctypes
import logging

log = logging.getLogger(__name__)

def get_backend() -> IBackend:
    """Prefer libusb_package's bundled DLL; fall back to system libusb-1.0."""
    try:
        import libusb_package
        backend = libusb_package.get_libusb1_backend()
        if backend is not None:
            log.debug('[get_backend] found libusb_package bundled DLL')
            return backend
    except ImportError:
        pass
    import usb.backend.libusb1 as libusb1_backend
    log.debug('[get_backend] falling back to libusb-1.0')
    return libusb1_backend.get_backend()

def find_device(vid: int, pid: int) -> usb.core.Device | None:
    log.debug(f"[find_device] resolving libusb backend...")
    backend = get_backend()
    log.debug(f"[find_device] backend = {backend}")

    log.debug(f"[find_device] looking for {vid:04x}:{pid:04x} ...")
    dev = usb.core.find(idVendor=vid, idProduct=pid, backend=backend)
    if dev is None:
        log.debug("[find_device] device not found")
    log.debug("[find_device] found device")

    return dev

def get_device(bus: int, address: int) -> usb.core.Device:
    backend = get_backend()
    return usb.core.find(custom_match=lambda d: d.bus == bus and d.address == address, backend=backend)