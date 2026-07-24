from pieusb.transport import UASDevice
import usb.core

from pieusb.option import Option

class Scanner:
    def __init__(self, dev: usb.core.Device) -> None:
        self.dev = UASDevice(dev=dev)
        self.dev.open()

    def __enter__(self) -> "Scanner":
        return self

    def __exit__(self, exc_type, exc, tb):
        self.dev.close()