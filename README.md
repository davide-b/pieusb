# pieusb

Python package to enable interfacing with USB-connected Reflecta (a.k.a. PacificImage) film and dia scanners.

## Usage example

```python
import pieusb
from pieusb.types import ScanPhase

for info in pieusb.available_devices():
    print(info.vendor, info.model)

scanner = pieusb.Scanner(pieusb.available_devices()[0])
scanner.mode = "rgbi"
scanner.resolution = 5000
scanner.color_depth = 16
scanner.tl_x, scanner.tl_y = 0, 0
scanner.br_x, scanner.br_y = 14200, 9600

def on_progress(data):
    if data.phase == ScanPhase.SCANNING:
        print(f"{data.scanned_lines}/{data.total_lines}")

def on_complete(result):
    if result.error:
        raise result.error
    numpy.save("scan.npy", result.image)

scanner.scan(progress=on_progress, scan_complete=on_complete)   # returns immediately
scanner.wait()                                                  # optional
scanner.close()
```

# WARNING

No guarantees are made as far as hardware safety goes.

It may be possible to set certain parameters, such as scan width or height, such that the physical limits of the scanner are exceeded. This may damage your hardware.

Proceed at your own risk.