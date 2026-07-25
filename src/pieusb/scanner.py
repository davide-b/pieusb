from pieusb.transport import UASDevice
import usb.core

from pieusb.option import Option, generate_options
from pieusb.types import DeviceInfo

class Scanner:
    def __init__(self, info: DeviceInfo) -> None:
        self.dev = UASDevice(info.dev)
        self.options = generate_options(info.inquiry)
        self.dev.open()

    def __enter__(self) -> "Scanner":
        return self

    def __exit__(self, exc_type, exc, tb):
        self.dev.close()

    def __setitem__(self, key, value):
        opt = self.options[key]
        if type(value) is not opt.type:
            raise TypeError(f"Invalid type provided to option '{opt.name}'")
        if not opt.validate(value):
            raise ValueError(f"Invalid value provided to option '{opt.name}'")
        opt.value = value
    