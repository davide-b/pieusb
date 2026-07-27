# pieusb — API & module design

Design notes for `pieusb`, a Python library for USB-connected Reflecta (a.k.a. PacificImage)
film/dia scanners.

Earlier revisions of this document aimed at a [python-sane](https://github.com/python-pillow/Sane)
lookalike. That goal has been dropped. python-sane's surface carries a lot of shape that exists
for SANE's sake rather than the user's — `init()`/`exit()` lifecycle calls, device name strings
that have to be parsed back into devices, a `start()`/`snap()` split that this hardware does not
actually stream across, `get_parameters()` returning a five-tuple, PIL conversions that cannot
represent a 4-plane RGBI scan. **The API is instead reduced to the smallest thing that does the
job**: enumerate, configure, scan.

The internal *layering* still follows the SANE C backend (`backend/pieusb*.c`), which remains the
authoritative reference for everything on the wire. Only the public surface changes.

## Target usage

```python
import pieusb

for info in pieusb.available_devices():
    print(info.vendor, info.model)

scanner = pieusb.Scanner(pieusb.available_devices()[0])
scanner.mode = "rgbi"
scanner.resolution = 5000
scanner.color_depth = 16
scanner.tl_x, scanner.tl_y = 0, 0
scanner.br_x, scanner.br_y = 14200, 9600

def on_progress(scanned_lines, total_lines, chunk, plane):
    print(f"{scanned_lines}/{total_lines}")

def on_complete(result):
    if result.error:
        raise result.error
    numpy.save("scan.npy", result.image)

scanner.scan(progress=on_progress, scan_complete=on_complete)   # returns immediately
scanner.wait()                                                  # optional
scanner.close()
```

A scan takes minutes at high resolution, so `scan()` is **non-blocking**: it validates the
options, starts a worker thread and returns. Callers who want the blocking behaviour call
`wait()`.

## Public API

The entire exported surface:

| Name | Role |
|------|------|
| `available_devices() -> list[DeviceInfo]` | Enumerate connected scanners and INQUIRY each one |
| `Scanner(info: DeviceInfo)` | Open a device; options as attributes; `scan()`/`cancel()`/`wait()`/`close()` |
| `DeviceInfo`, `InquiryResponse` | What enumeration hands back |
| `ScanResult`, `ScanPhase` | What a finished scan reports |
| `PieusbError` and subclasses | Errors — see [Exceptions](#exceptions) |

Deliberately **not** exposed: `init()`, `exit()`, `open()`, device-name strings, `start()`/`snap()`,
`get_parameters()`, `multi_scan()`, `fileno()`, PIL conversion.

### `available_devices()`

Named after pyserial's `comports()` idiom: one call that both finds devices and describes them.
It returns `DeviceInfo` objects, each holding the live `usb.core.Device` alongside the parsed
`InquiryResponse`, so a caller can pick one by model, capability, or index and hand it straight
to `Scanner`. There is no name string and therefore nothing to parse back into a device.

This replaces `get_devices()`.

### `Scanner`

Constructed directly from a `DeviceInfo` — no `open()` indirection, since `DeviceInfo` already
carries everything needed. The constructor opens the USB interface and builds the option table
from the device's own INQUIRY response.

- **Options are attributes.** `scanner.resolution = 5000` goes through `__setattr__`, which
  type-checks and validates against the constraints derived from INQUIRY, raising `TypeError` /
  `ValueError` immediately on a bad value. Reading an option returns its current value.
  `scanner.options` exposes the underlying table for introspection (name, type, unit, default).
  Setting an option while a scan is running raises `ScanInProgress`.
- **Nothing reaches the device until `scan()`.** Options are host-side state. This mirrors the C
  backend exactly: `sane_control_option()` only writes into `scanner->val[]`, and `sane_start()`
  (`pieusb.c:865`) pushes the whole configuration to the device in one sequence.
- **`scanner.phase -> ScanPhase`** is readable at any time, so a caller can tell warm-up from
  calibration from the pixel read without waiting for a callback.
- **`scanner.scanning -> bool`**.

#### Methods

```python
def ready(self) -> bool
def wait_ready(self, timeout_s: float = 180) -> None
def scan(self, progress=None, scan_complete=None) -> None
def cancel(self) -> None
def wait(self, timeout: float | None = None) -> bool
def close(self) -> None
```

- **`ready()`** asks the device whether it can start a scan right now, via TEST UNIT READY.
  Returns a plain `True`/`False` — every NOT_READY sense maps to `False`, including warm-up. Only
  transport-level failures raise. It is a convenience predicate; `scan()` does its own check and
  does not require you to call this first.
- **`wait_ready()`** polls `ready()` until the device is ready or the timeout expires, raising
  `WarmingUp` if it was still warming up when time ran out and `DeviceNotReady` otherwise.
- **`scan()`** checks readiness, validates the whole option table, then starts a worker thread
  and returns immediately. It raises rather than starting a scan that cannot succeed:
  - `WarmingUp` — the lamp is still warming up. Retryable; the caller decides whether to
    `wait_ready()` and try again, or report it and move on.
  - `DeviceNotReady` — the device is reachable but cannot scan (including the case where the
    device itself reports a scan already active, `Pieusb_Scanner_State.scanning`).
  - `ScanInProgress` — *this* `Scanner` already has a worker running. Host-side state, distinct
    from the device-side conditions above.
  - `ValueError` / `TypeError` — invalid option combination.

  All of these are raised on the *calling* thread before the worker starts, so a scan that cannot
  begin fails at the call site rather than arriving in a callback.
- **`cancel()`** requests a stop and returns immediately. It does **not** touch the device: it
  sets a flag the worker checks between chunk reads, and the worker issues STOP SCAN and then
  fires `scan_complete` with `result.cancelled` set. Cancellation therefore takes effect at the
  next chunk boundary (≤255 lines), not instantly. Calling `cancel()` when no scan is running is
  a no-op.
- **`wait()`** blocks until the scan finishes, is cancelled or fails; returns `True` if it
  finished within the timeout.
- **`close()`** cancels any running scan, waits for the worker, then releases the interface.

`__exit__` does exactly what `close()` does. This matters: a `with` block that exits while a scan
is in flight must not release the USB interface out from under the worker thread.

`ready()` and `wait_ready()` talk to the device, so they raise `ScanInProgress` while a worker is
running — see the threading contract below.

## Exceptions

```
PieusbError
├── DeviceNotReady        device is reachable but cannot start a scan
│   └── WarmingUp         ... specifically because the lamp is warming up (retryable)
├── CheckCondition        raw SCSI CHECK CONDITION with decoded sense data
├── Timeout               a command did not complete in time
└── ScanInProgress        host-side: this Scanner already has a worker running
```

`WarmingUp` subclasses `DeviceNotReady` so that `except DeviceNotReady` catches "cannot scan"
generally, while a caller who wants to retry can catch `WarmingUp` specifically. That hierarchy
mirrors the wire: both are SCSI sense key `NOT_READY` (0x02), and warm-up is the single case
distinguished by code 0x04 / qualifier 0x01 — `sanei_pieusb_decode_sense` maps exactly that
triple to `PIEUSB_STATUS_WARMING_UP` and every other `NOT_READY` to a generic failure
(`pieusb_usb.c:387-397`).

`ScanInProgress` is deliberately *not* under `DeviceNotReady`: it describes this library's own
state, not the scanner's, and the two want different handling — one is "wait and retry", the
other is "you have a bug". It is also why the name avoids "busy", which on this hardware already
means the SCSI `STATUS_BUSY` code.

### Warm-up is handled in two different places, on purpose

- **Before the worker starts**, `scan()` refuses and raises `WarmingUp`. The caller is still on
  the stack and can decide: wait, retry, tell the user, give up.
- **After the worker starts**, at START SCAN, the worker retries with a bounded backoff rather
  than failing. There is no caller left to decide, and the C backend does the same
  (`pieusb.c:1088-1093`, retrying while the status is `WARMING_UP`). If the retries are
  exhausted, it surfaces as `result.error = WarmingUp` through `scan_complete`.

Both references also check up front: `sane_start()` reads the device state and returns
immediately if `state.warmingUp` (`pieusb.c:908-921`), and the PoC calls `wait_ready()` before it
does anything else.

### Threading contract

`scan()` is non-blocking, which makes threading part of the public contract rather than an
implementation detail:

- **Both callbacks run on the worker thread**, not the caller's. A GUI caller must marshal to its
  own event loop; a callback that blocks stalls the read loop and, on a slow enough callback, can
  stall it long enough for the device to time out.
- **All device access stays on the worker thread.** pyusb/`UASDevice` is not thread-safe, which
  is why `cancel()` is flag-based rather than issuing STOP SCAN itself.
- **Exceptions never propagate to the caller** once the worker has started. Anything raised
  inside the scan is caught, STOP SCAN is attempted, and it is delivered as `result.error`. A
  caller that only registers `progress` and never `scan_complete` will therefore *silently* miss
  failures — `wait()` returning is not evidence of success. Check `result.error`.
- One worker at a time per `Scanner`; a second `scan()` raises `ScanInProgress`.

### `progress` callback

```python
def progress(scanned_lines: int, total_lines: int, chunk: numpy.ndarray, plane: int) -> None
```

- `scanned_lines` / `total_lines` count raw scanner lines across the whole read, so
  `total_lines == planes * height`. They reach equality exactly once, at the end.
- `chunk` is the lines read since the previous call, shape **`(n_lines, pixels_per_line)`**,
  already shading-corrected, dtype matching `color_depth`.
- `plane` is which channel those lines belong to: `0=R, 1=G, 2=B, 3=I`.

**Why `chunk` is single-plane rather than an image strip.** The scanner does not return
interleaved rows. It returns the planes as sequential blocks — all R lines, then all G, and so on
— each line prefixed by a two-byte channel tag (`poc:794-818`). So the lines arriving at any
moment belong to *one* channel, and a chunk of them cannot be assembled into an
`(n_lines, width, planes)` strip; the other channels for those rows do not exist yet and will not
until much later in the scan. Delivering `(n_lines, width)` plus a `plane` index is the only
shape that is both honest about what arrived and useful for a live preview.

The 255-line read cap does not align to plane boundaries, so a single device read can straddle a
channel transition. The library splits such a read by tag and calls `progress` once per plane
run, so a chunk is always single-plane. This means the callback may fire twice for one device
read, and `n_lines` varies.

### `scan_complete` callback

```python
def scan_complete(result: ScanResult) -> None
```

Fires exactly once per `scan()`, on every outcome — success, cancellation and failure — so it is
the single place teardown belongs.

```python
@dataclass(frozen=True)
class ScanResult:
    image: numpy.ndarray | None   # None if cancelled or failed
    width: int                    # pixels per line, as reported by GET PARAMETERS
    height: int                   # lines per plane
    planes: int                   # 4 for rgbi, 3 for rgb, 1 for gray
    mode: str
    color_depth: int              # 8 or 16
    resolution: int               # dpi actually used
    shading_corrected: bool
    cancelled: bool
    error: Exception | None
    duration_s: float
```

`width`/`height` come from the device's own GET PARAMETERS mid-scan, not from the requested
frame, and can differ from what the options asked for — which is why they are reported rather
than left for the caller to infer. On a cancelled scan `image` is `None` and the partial data is
discarded; assembling a half-read image would mean returning planes with no data in them.

The parameters were collapsed into one `ScanResult` rather than a long positional signature: it
keeps the callback stable as fields are added, and it is the same frozen-dataclass style the rest
of the package uses.

### Return value

`ScanResult.image` is a numpy array of shape **`(lines, pixels_per_line, planes)`** — i.e.
`(height, width, planes)`, the standard numpy image convention. `image[y, x, c]` indexes row,
column, channel.

- Plane order is **R, G, B, I**, resolved from the per-line channel tag, not from arrival order.
- `mode="rgbi"` gives 4 planes, `"rgb"` gives 3, `"gray"` gives 1.
- dtype is `uint8` at `color_depth=8`, `uint16` at 16.
- Pixels are flat-field (shading) corrected. Infrared dust removal is a separate, later concern.

Returning raw numpy rather than PIL is the honest choice here: a 4-plane RGBI scan has no PIL
mode, and 16-bit multi-channel is awkward in PIL regardless. Callers who want a PIL image can
convert the RGB planes themselves.

## Module structure

Split along the C backend's module boundaries.

| File | Role | C-backend analog | State |
|------|------|------------------|-------|
| `__init__.py` | Public API: `available_devices()`, re-export `Scanner`, types, exceptions | `pieusb.c` | partial |
| `scanner.py` | **`Scanner`** — options-as-attributes, `scan()`/`cancel()`/`wait()`/`close()`, the worker thread and its state machine | `pieusb_specific.c` | partial |
| `options.py` | `Option`/`Parameter`/`OptionsTable`, the table generator, whole-table validation | (SANE opt descriptors) | exists as `option.py` |
| `commands.py` | Typed SCSI wrappers: `inquiry`, `set_mode`, `set_scan_frame`, `set_exposure`, `set_highlight_shadow`, `get`/`set_gain_offset`, `get_shading_parms`, `start_scan`, `get_ccd_mask`, `get_scan_parameters`, `get_scanned_lines` | `pieusb_scancmd.c` | scattered in `option.py` |
| `transport.py` | **`UASDevice`** — SCSI-over-USB transaction layer | `pieusb_usb.c` | done |
| `protocol.py` | Opcodes, ports, statuses, MODE SELECT constants | (headers) | inline in `transport.py`/`option.py` |
| `usb.py` | libusb backend + enumeration | `pieusb_usb.c` | exists as `usb_utils.py` |
| `inquiry.py` | `InquiryResponse` parsing + model/VID-PID tables | `pieusb_specific.c` | done |
| `postprocess.py` | Deinterleave by line tag, shading correction, (later) IR dust removal | `pieusb_buffer.c`, `sanei_pieusb_post` | missing |
| `types.py` | Enums + dataclasses, incl. `ScanResult`/`ScanPhase` | — | partial |
| `exceptions.py` | `PieusbError` base; `DeviceNotReady`, `WarmingUp`, `CheckCondition`, `Timeout`, `ScanInProgress` | — | `CheckCondition` in `transport.py` |

`_device.py` folds into `__init__.py` + `inquiry.py`.

## What the worker does

The command sequence is `sane_start()` (`pieusb.c:865-1140`), which is the authoritative order.
Steps 3 and 8 are reads, not writes — the configuration push is not purely one-directional, and
`pixels_per_line` from step 3 sizes both the shading read and the CCD mask later.

Step 4 runs on the calling thread inside `scan()`, before the worker starts. Everything from
step 1 onward runs on the worker. The worker checks the cancel flag before each chunked read in
steps 11 and 12.

| Phase (`ScanPhase`) | Steps |
|---|---|
| `CONFIGURING` | 1. Refuse if already scanning; READ STATE and check warming-up.<br>2. SET EXPOSURE TIME, SET HIGHLIGHT SHADOW.<br>3. **GET SHADING PARMS** → `pixels_per_line`, `n_lines`.<br>4. **Validate the option table as a whole.**<br>5. SET SCAN FRAME.<br>6. `cmd_17`, on slide-transport models that support it.<br>7. GET GAIN OFFSET → SET GAIN OFFSET (read-modify-write).<br>8. SET MODE.<br>9. SLIDE INIT, on slide-transport models. |
| `WARMING_UP` | 10. START SCAN, retrying while the device reports warming up. |
| `CALIBRATING` | 11. Read the shading reference, then the CCD mask, then GET PARAMETERS for authoritative width/lines. |
| `SCANNING` | 12. Read `planes × height` tagged lines in ≤255-line chunks, splitting each read by channel tag, shading-correcting it and firing `progress`. |
| `PROCESSING` | 13. Assemble the planes into the final array. |

Step 4 is where the cross-option checks live that per-option validators structurally cannot
express: `tl < br` on both axes, the frame within the device's reported bed, `sharpen` versus
`fast_infrared`. The C backend does the same, in `sanei_pieusb_analyse_options` at the same point
in the sequence. Getting the frame wrong has already caused one carriage crash in this project,
so this must abort before the device is touched — hence running it on the calling thread, where
it can raise.

Note that the C backend performs the entire read inside `sane_start()`; `sane_read()` only copies
out of an already-filled buffer (`pieusb.c:1363`). There is no streaming mode in this hardware,
which is the other reason a `start()`/`snap()` split buys nothing here.

## Open design questions

- **Nothing reports progress before the pixel read.** `progress` only fires during `SCANNING`,
  because it is defined in terms of image lines. But `CONFIGURING` through `CALIBRATING` includes
  lamp warm-up, which can take minutes on a cold scanner, and `scanner.phase` has to be polled to
  observe it. If that turns out to be inadequate, the options are a separate `on_phase` callback
  or a phase argument on `progress`.
- **Frame coordinate units.** The wire format is native-resolution units (10000 dpi on the
  ProScan 10T), independent of the requested scan dpi. The option table currently exposes those
  raw units. Exposing millimetres instead would be friendlier and is what the C backend's options
  do, at the cost of a conversion layer. Unresolved; see `TODO.md` C-14.
- **`gray` mode.** It maps to `SCAN_COLOR_FORMAT_PIXEL`, so the returned data carries no per-line
  channel tags and the tag-based deinterleave does not apply. `progress`'s `plane` argument is
  meaningless there. Either handle both layouts or reject `gray` until it is implemented.
- **Auto-exposure / preview pass.** The `auto_exp` option exists but nothing implements it. The C
  backend derives gain/offset from a preview scan (`OPT_CALIBRATION_MODE`); that is a second
  acquisition inside the worker, and it should be decided whether it reports progress separately
  or is folded into `CONFIGURING`.

## References

- SANE C backend: `~/backends/backend/pieusb*.c` — `pieusb.c` (entry points, `sane_start`),
  `pieusb_specific.c` (options → device), `pieusb_scancmd.c` (SCSI wrappers),
  `pieusb_buffer.c`, `pieusb_usb.c`. Authoritative for anything on the wire.
- PoC: `~/sw/pieusb_ref/pieusb_proscan10t_poc.py` — linear, single-device implementation of the
  full open → INQUIRY → configure → scan → numpy flow, verified against a real ProScan 10T.
  Authoritative for what has actually been observed to work; ProScan 10T only.
- `TODO.md` — current state and the concrete work remaining.
