"""pieusb -- direct-USB acquisition library for USB-connected Reflecta film scanners.
"""

try:
    from importlib.metadata import version as _pkg_version

    __version__ = _pkg_version("pieusb")
except Exception:  # pragma: no cover - only when the package isn't installed
    # The real version is derived from the git tag at build time; there is no
    # meaningful number to fall back on when running from an uninstalled tree.
    __version__ = "0.0.0+unknown"

from pieusb._device import get_devices

__all__ = [
    "__version__",
    # module-level (python-sane-shaped)
    "get_devices",
]