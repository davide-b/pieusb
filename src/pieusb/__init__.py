"""pieusb -- direct-USB acquisition library for USB-connected Reflecta film scanners.
"""

try:
    from importlib.metadata import version as _pkg_version

    __version__ = _pkg_version("pieusb")
except Exception:  # pragma: no cover - only when the package isn't installed
    __version__ = "0.1.0"

from pieusb._device import get_devices

__all__ = [
    "__version__",
    # module-level (python-sane-shaped)
    "get_devices",
]