#!/usr/bin/env python3
'''USB-level probe: why can this scanner not be opened?

Separates the USB layer from the protocol. Every step is reported on its own, so
a failure says which one broke rather than surfacing as one opaque libusb error.

    python tests/usbprobe.py
    sudo python tests/usbprobe.py        # descriptors and node permissions need it

Touches no scanner protocol: the furthest it goes is SET_CONFIGURATION, which is
what `UASDevice.open()` does first.
'''

import ctypes
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src'))

import usb.core
import usb.util

from pieusb.inquiry import KNOWN_PIDS, PIE_VENDOR_ID
from pieusb.usb_utils import get_backend


class _Version(ctypes.Structure):
    _fields_ = [('major', ctypes.c_uint16), ('minor', ctypes.c_uint16),
                ('micro', ctypes.c_uint16), ('nano', ctypes.c_uint16),
                ('rc', ctypes.c_char_p), ('describe', ctypes.c_char_p)]


def libusb_version(backend):
    try:
        fn = backend.lib.libusb_get_version
        fn.restype = ctypes.POINTER(_Version)
        v = fn().contents
        return f"{v.major}.{v.minor}.{v.micro}.{v.nano}"
    except Exception as e:
        return f"unknown ({type(e).__name__}: {e})"


def node_path(dev):
    return f"/dev/bus/usb/{dev.bus:03d}/{dev.address:03d}"


def report_node(dev):
    path = node_path(dev)
    print(f"  device node        : {path}")
    try:
        st = os.stat(path)
    except OSError as e:
        print(f"    stat failed      : {e}")
        return
    import grp
    import pwd
    try:
        owner = pwd.getpwuid(st.st_uid).pw_name
        group = grp.getgrgid(st.st_gid).gr_name
    except KeyError:
        owner, group = st.st_uid, st.st_gid
    print(f"    mode             : {oct(st.st_mode & 0o777)}  {owner}:{group}")
    print(f"    writable by us   : {os.access(path, os.W_OK)}")


def report_sysfs(dev):
    '''bConfigurationValue and the driver bound to each interface, from sysfs.'''
    root = Path('/sys/bus/usb/devices')
    if not root.exists():
        return
    for entry in sorted(root.glob(f'{dev.bus}-*')):
        try:
            if int((entry / 'devnum').read_text()) != dev.address:
                continue
        except (OSError, ValueError):
            continue
        print(f"  sysfs              : {entry.name}")
        for name in ('bConfigurationValue', 'bNumConfigurations', 'speed',
                     'maxchild', 'busnum', 'devnum'):
            try:
                print(f"    {name:<17}: {(entry / name).read_text().strip()}")
            except OSError:
                pass
        for iface in sorted(entry.glob(f'{entry.name}:*')):
            driver = iface / 'driver'
            bound = os.path.basename(os.readlink(driver)) if driver.is_symlink() else '-'
            print(f"    interface {iface.name.split(':')[-1]:<7}: driver {bound}")


def probe(dev):
    print(f"\nFound {dev.idVendor:04x}:{dev.idProduct:04x} "
          f"on bus {dev.bus} address {dev.address}")
    report_node(dev)
    report_sysfs(dev)

    print("  descriptors        :", end=' ')
    try:
        print(f"bcdDevice 0x{dev.bcdDevice:04x}, "
              f"{dev.bNumConfigurations} configuration(s)")
    except Exception as e:
        print(f"unreadable: {type(e).__name__}: {e}")

    print("  active config      :", end=' ')
    active = None
    try:
        active = dev.get_active_configuration()
        print(f"bConfigurationValue {active.bConfigurationValue}, "
              f"{active.bNumInterfaces} interface(s)")
    except usb.core.USBError as e:
        print(f"none or unreadable: {e}")

    if active is not None:
        for iface in active:
            n = iface.bInterfaceNumber
            try:
                kernel = dev.is_kernel_driver_active(n)
            except Exception as e:
                kernel = f"unknown ({e})"
            print(f"    interface {n}        : class 0x{iface.bInterfaceClass:02x}, "
                  f"kernel driver active {kernel}")

    print("  set_configuration  :", end=' ')
    try:
        dev.set_configuration()
        print("ok")
    except usb.core.USBError as e:
        print(f"FAILED: {e}  (errno {e.errno}, backend_error_code "
              f"{getattr(e, 'backend_error_code', None)})")
        print("""
    This is where UASDevice.open() stops. On Linux the usual causes, in order:

      1. Another process or a VM still holds the device. USB passthrough
         (VirtualBox, VMware, usbip, qemu) keeps it claimed even when the guest
         is idle. Check with:  sudo fuser -v /dev/bus/usb/*/*
         and make sure no VM has it captured.
      2. A kernel driver is bound to interface 0 -- see the driver lines above.
         Detach it with:  sudo modprobe -r <driver>
      3. libusb too old to map EBUSY, reporting it as this generic error.
""")
        return 1

    print("  claim_interface    :", end=' ')
    try:
        cfg = dev.get_active_configuration()
        number = cfg[(0, 0)].bInterfaceNumber
        usb.util.claim_interface(dev, number)
        print(f"ok (interface {number})")
        usb.util.release_interface(dev, number)
    except Exception as e:
        print(f"FAILED: {type(e).__name__}: {e}")
        return 1
    finally:
        usb.util.dispose_resources(dev)

    print("\n  The USB layer is fine. A failure past this point is protocol.")
    return 0


def main():
    backend = get_backend()
    print(f"backend            : {backend}")
    print(f"libusb             : {libusb_version(backend)}")
    print(f"euid               : {os.geteuid()}")

    found = [d for pid in KNOWN_PIDS
             for d in usb.core.find(find_all=True, idVendor=PIE_VENDOR_ID,
                                    idProduct=pid, backend=backend)]
    if not found:
        print(f"\nNo {PIE_VENDOR_ID:04x} device with a known product id "
              f"({', '.join(f'{p:04x}' for p in KNOWN_PIDS)}) is attached.")
        print("Every 05e3 device the host can see:")
        for d in usb.core.find(find_all=True, idVendor=PIE_VENDOR_ID,
                               backend=backend) or []:
            print(f"  {d.idVendor:04x}:{d.idProduct:04x} "
                  f"bus {d.bus} address {d.address}")
        return 1

    return max(probe(d) for d in found)


if __name__ == '__main__':
    sys.exit(main())
