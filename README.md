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
import numpy
import pieusb
from pieusb.scanner import Scanner

for info in pieusb.get_devices():
    print(info.vendor, info.model)

def on_update(data):
    # progress runs 0.0 -> 1.0 within each phase, not across the scan
    print(f"{data.phase}: {data.progress * 100:.1f}%")

def on_complete(result):
    if result.error:
        raise result.error
    numpy.save("scan.npy", result.rgb)

with Scanner(pieusb.get_devices()[0]) as scanner:
    scanner.mode = "rgbi"
    scanner.resolution = 5000
    scanner.color_depth = 16
    scanner.tl_x, scanner.tl_y = 0, 0
    scanner.br_x, scanner.br_y = 14200, 9600

    scanner.scan(on_update, on_complete)   # returns immediately
    scanner.wait()                         # optional
```

### Scanning more than one frame

Keep one `Scanner` open for the whole batch rather than opening one per frame.
It is faster, and the reason is `reuse_calibration`:

```python
with Scanner(pieusb.get_devices()[0]) as scanner:
    scanner.resolution = 5000
    scanner.reuse_calibration = True

    for _ in range(50):
        scanner.scan(on_update, on_complete)
        scanner.wait()
```

Shading (flat-field) correction is applied on the host from a reference the
scanner produces during a calibration pass, so the reference has to be read at
least once before it can be reused. The first scan above therefore calibrates
whatever the option says; the rest skip it and correct from the cached reference,
which is what saves the time. The scanner keeps the last word: when it decides
its calibration has gone stale -- after a power cycle, or on its own drift
checks -- it refuses the skip and that scan calibrates again.

Every scan is shading-corrected either way. `reuse_calibration` only ever
determines whether a pass pays for its own calibration, never whether the image
gets corrected. Since the cache lives on the `Scanner`, a fresh one starts cold:
a loop that opens a `Scanner` per frame is correct but calibrates every time.

## Status of verification

### Verified on hardware

- ProScan 10T
- RPS 10M (aka Pacific Image PrimeFilm XAs) - **Single frame scan only**

# WARNING

No guarantees are made as far as hardware safety goes.

It may be possible to set certain parameters, such as scan width or height, such that the physical limits of the scanner are exceeded. This may damage your hardware.

Proceed at your own risk.
