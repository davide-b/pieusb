# pieusb — API & module design

Design notes for shaping `pieusb` into a [python-sane](https://github.com/python-pillow/Sane)-lookalike
Python library for USB-connected Reflecta (a.k.a. PacificImage) film/dia scanners.

The goal is that everyday usage reads as close to `python-sane` as the hardware allows.
Conveniently, python-sane's *usage pattern* and the SANE C backend's *internal layering*
(`backend/pieusb*.c`, which the `pieusb_proscan10t_poc.py` PoC already follows) point at the
same structure — so mirroring one reinforces the other.

## Target usage

```python
import pieusb

pieusb.init()                              # -> version tuple
for name, vendor, model, kind in pieusb.get_devices():
    print(name, vendor, model)

dev = pieusb.open("pieusb:1:5")            # -> Scanner
dev.mode = "rgbi"                          # options as attributes
dev.resolution = 300
dev.depth = 8
dev.tl_x, dev.tl_y, dev.br_x, dev.br_y = 0.0, 0.0, 36.0, 24.0

fmt, last, (ppl, lines), depth, bpl = dev.get_parameters()
dev.start()
arr = dev.arr_snap()                       # -> numpy (H, W, planes)
dev.close()

# or, context-managed + one-shot:
with pieusb.open(name) as dev:
    dev.mode, dev.resolution = "color", 600
    img = dev.scan()                       # start() + snap() -> PIL.Image
```

## Proposed file structure

Split along the C backend's proven module boundaries, which map cleanly onto the layers the PoC
already has. (The current `_device.py` is a grab-bag — enumeration + INQUIRY parsing + model
tables + `open()` — and `open()` isn't even exported.)

| File | Role | C-backend analog | Comes from today |
|------|------|------------------|------------------|
| `__init__.py` | Public API: `init()`, `get_devices()`, `open()`, `exit()`; re-export `Scanner`, exceptions, enums | `pieusb.c` (SANE entry points) | partial |
| `scanner.py` | **`Scanner`** — the `SaneDev` analog: options-as-attributes, `get_parameters/start/snap/scan/arr_snap/multi_scan/cancel/close`, context mgr, the scan state machine | `pieusb_specific.c` | `scanning.py` (grows) |
| `options.py` | `Option` class + option table driving `Scanner`'s attribute magic | (SANE opt descriptors) | new |
| `commands.py` | Typed SCSI command wrappers: `inquiry`, `set_mode`, `set_scan_frame`, `set_exposure`, `set_highlight_shadow`, `get/set_gain_offset`, `get_shading_parms`, `start_scan`, `get_ccd_mask`, `get_scan_parameters`, `get_scanned_lines` | `pieusb_scancmd.c` | PoC free-funcs + `scanning.py` |
| `transport.py` | **`UASDevice`** — SCSI-over-USB transaction layer | `pieusb_usb.c` | `uas.py` |
| `protocol.py` | Opcode / port / status constants | (headers) | consts in `uas.py` |
| `usb.py` | libusb backend + `find_device`/`get_device`/enumeration | `pieusb_usb.c` | `usb_utils.py` |
| `inquiry.py` | `InquiryResponse` parsing + model/VID-PID tables | `pieusb_specific.c` | parsing in `_device.py` |
| `postprocess.py` | Line-tag deinterleave, shading correction, (future) IR dust removal | `pieusb_buffer.c` + `sanei_pieusb_post` | PoC `scan()` tail |
| `types.py` | Enums + dataclasses | — | keep |
| `exceptions.py` | `PieusbError` base, `CheckCondition`, `NotReady`, `Timeout` | — | `CheckCondition` in `uas.py` |

**Minimum viable split** (if the full list is too many files up front):
`__init__.py` (API) · `scanner.py` (Scanner + options) · `commands.py` · `transport.py` +
`protocol.py` · `usb.py` · `inquiry.py` · `postprocess.py` · `types.py`.

## The `Scanner` class (the `SaneDev` analog)

Named **`Scanner`** rather than copying `SaneDev` — cleaner, and the *methods* are what carry the
familiarity.

### Options-as-attributes

Exactly like python-sane's `__getattr__`/`__setattr__` over an option dict:

```python
OPTIONS = [
    Option("resolution", int,   unit="dpi", constraint=(...)),  # SET MODE dpi
    Option("mode",       str,   constraint=["color", "rgbi"]),  # passes/planes
    Option("depth",      int,   constraint=[8, 16]),            # SET MODE bit depth
    Option("tl_x", float, unit="mm"), Option("tl_y", float, unit="mm"),
    Option("br_x", float, unit="mm"), Option("br_y", float, unit="mm"),  # SET SCAN FRAME
    Option("exposure",   tuple),                                # SET EXPOSURE r,g,b
    Option("highlight",  tuple), Option("shadow", tuple),
    Option("infrared",   bool),                                 # IR plane on/off
    Option("correct_shading", bool),
    Option("preview",    bool),
]
```

- Keep the virtual read-only attrs: `optlist`, `area` -> `((tl_x, tl_y), (br_x, br_y))`,
  `scanner_model`.
- `get_options()` returns the `Option` tuples; `dev["mode"]` returns the `Option` object with
  `.constraint`, `.is_settable()`.
- `InquiryResponse` is the natural source for the constraints (max res, supported depths, filters,
  bed size).

### Methods

Map onto the PoC's `scan()`, split at the point where pixel data starts:

- `start()` — front half of PoC `scan()`: `wait_ready` -> exposure/highlight/shadow -> shading
  parms -> SET SCAN FRAME -> gain/offset -> SET MODE -> START SCAN (+ warming-up/must-calibrate
  loop) -> read shading ref + CCD mask -> GET PARAMETERS. Leaves the device positioned to stream
  pixels.
- `snap()` — read all lines, deinterleave by channel tag, shading-correct, return **PIL.Image**.
- `arr_snap()` — same, return **numpy** `(H, W, planes)`. This is the PoC's current return and the
  natural primary for this hardware.
- `scan()` = `start()` + `snap()`; `arr_scan()` = `start()` + `arr_snap()`.
- `get_parameters()` -> `(format, last_frame, (ppl, lines), depth, bytes_per_line)`.
- `cancel()` -> `stop_scan()`; `close()`; `__enter__`/`__exit__`.

## Two forced divergences from python-sane

1. **RGBI doesn't fit a PIL mode.** python-sane's `snap()` returns `'RGB'`/`'L'`. For 4-plane rgbi,
   `arr_snap()` (numpy) is the honest primary. Proposed: `snap()` returns RGB (dropping the IR
   plane) for `mode="color"` and raises a clear error for `rgbi`, steering users to `arr_snap()`.
   Alternatively keep the IR plane accessible as `dev.infrared_plane` after a scan.
2. **`get_parameters()` timing.** In python-sane it's valid before `start()`. On this hardware,
   authoritative width/lines only come from GET PARAMETERS *mid-scan* (after shading + CCD read).
   Proposed: return a computed estimate before `start()`, cache the real values after — matching
   the "some backends return different values before vs after start()" caveat already in
   python-sane's own docs.

## Small fixes to fold in during the move

- Export `open` from `__init__` (currently missing).
- `get_devices()` should yield python-sane's 4-tuple shape `(name, vendor, model, type)`;
  `DeviceInfo` can still back it, but `open()` should accept the `name` string it hands out.
- `MODEL_NAMES[...]` in `parse_inquiry` will `KeyError` on an unknown model — use
  `.get(code, f"Unknown (0x{code:04x})")`.
- Typo: `KNWON_PIDS` -> `KNOWN_PIDS`.

## Suggested implementation order

1. **Pure reorganization** — file split + renames, behavior unchanged; verify imports still work.
2. **API layer** — add the `Scanner`/`Option` python-sane-shaped layer on top and port the PoC's
   `scan()` into `start()` / `arr_snap()`.

## References

- python-sane API (module `sane`, v2.9.2): module fns `init/get_devices/open/exit`; `SaneDev`
  with `get_parameters/get_options/start/snap/scan/arr_snap/arr_scan/multi_scan/cancel/close/`
  `fileno`, options-as-attributes, `Option` fields `index,name,title,desc,type,unit,size,cap,`
  `constraint` + `is_active()/is_settable()`, context-manager support, exceptions via
  `sane._sane.error`.
- SANE C backend: `~/backends/backend/pieusb*.c` (`pieusb.c`, `pieusb_specific.c`,
  `pieusb_scancmd.c`, `pieusb_buffer.c`, `pieusb_usb.c`).
- PoC: `~/sw/pieusb_ref/pieusb_proscan10t_poc.py` — linear, single-device reference implementation
  of the full open -> INQUIRY -> set params -> scan -> numpy flow.
