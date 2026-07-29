# pieusb

Python package to enable interfacing with USB-connected Reflecta (a.k.a. PacificImage) film and dia scanners.

## Prerequisites

### Windows

You need to swap whatever driver Windows auto-assigned to your scanner to WinUSB

### Linux

You need to add a udev rule

```shell
sudo tee /etc/udev/rules.d/60-pieusb.rules >/dev/null <<'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="05e3", ATTR{idProduct}=="0142", TAG+="uaccess"
SUBSYSTEM=="usb", ATTR{idVendor}=="05e3", ATTR{idProduct}=="0144", TAG+="uaccess"
SUBSYSTEM=="usb", ATTR{idVendor}=="05e3", ATTR{idProduct}=="0145", TAG+="uaccess"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
```
If you only care about operating one scanner, you can skip the lines with irrelevant `ATTR{idProduct}`. Here is a summary of which scanners have which

| Model | `ATTR{idProduct}` |
|---|---|
| DigitDia 6000 Multiple Slide Scanner | 0142 |
| CrystalScan 7200 | 0145 |
| ProScan 7200 | 0145 |
| ProScan 10T | 0145 |
| CrystalScan 3600 | 0145 |
| DigitDia 4000 | 0142 |
| RPS 10M (aka Pacific Image PrimeFilm XAs) | 0144 |

### macOS

It *should* just work™

## Install

```shell
pip install pieusb
```

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
