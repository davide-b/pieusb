# pieusb — next steps

Status audit of the mid-refactor tree (see `DESIGN.md` for the target shape). The package
imports and `set_options()` now runs to completion, but `Scanner.scan()` still stops right
after START SCAN, so there is no working acquisition path yet.

Cross-checked against the reference PoC (`~/sw/pieusb_ref/pieusb_proscan10t_poc.py`, cited
below as `poc:LINE`) and the SANE C backend (`~/backends/backend/pieusb*.c`).

**Next up:** C-13 (frame bounds check) before any hardware run, then B.

## A. Blocking bugs — DONE

All eight closed:

- [x] 1. `get_devices()` builds a real `list[DeviceInfo]`; debug `print`/`quit()` gone.
- [x] 2. `open()` returns a `Scanner`; the dead `ScannerDevice` is gone with `scanning.py`.
- [x] 3. `set(...)` misuse fixed — `generate_options()` runs.
- [x] 4. Mode names are `'gray'`/`'rgb'`/`'rgbi'` and the default validates.
- [x] 5. Frozen-`Option` solved by the `Option` (immutable descriptor) / `Parameter` (mutable
  value) split, with `Parameter.__init__` asserting the default passes its own validator.
- [x] 6. `.value` added on the four frame coordinates.
- [x] 7. `set_mode` implemented — see the note below.
- [x] 8. `KNOWN_PIDS` renamed; `MODEL_NAMES` KeyError guarded (`inquiry.py:118-122`).

**Note on `set_mode`** (`option.py`, end of `set_options`): built from
`sanei_pieusb_cmd_set_mode()` (`pieusb_scancmd.c:731-800`) rather than the PoC, because the
PoC hardcodes `colorFormat = 0x04` — true only for colour/rgbi. `sanei_pieusb_set_mode_from_options()`
(`pieusb_specific.c:1808-1838`) shows `passes` and `colorFormat` are picked together per mode,
which the `'gray'` mode needs. Byte-verified against the capture documented at
`pieusb_scancmd.c:759-772`; the only difference is byte 13 (line threshold), where we send
`0x80` per the cyberview capture and the PoC rather than that dump's `0x7f`. It only affects
lineart/halftone, which are unimplemented.

Two defaults settled while doing this: `calibrate` now defaults to **True** (the PoC's
proven-working configuration — see C-16), and a `fast_infrared` option was added, defaulting
to False, wiring quality bit `0x80`.

## B. Finish the scan path

- [ ] 9. **`Scanner.scan()` is truncated** (`scanner.py:82-112`): stops right after START SCAN
  and returns nothing despite `-> numpy.ndarray`. Missing downstream: shading-reference read,
  CCD mask, GET PARAMETERS, N-plane read, per-line tag deinterleave, image assembly. The PoC
  has all of it working (`poc:672-880`).
- [ ] 10. **Three commands exist nowhere in the package**, all prerequisites for #9:
  - `get_shading_parms` (`SCSI_CALIBRATION_INFO|0x80` prep write, then 32-byte read →
    `n_lines`/`pixels_per_line`, which size both the shading read *and* the CCD mask) — `poc:409`
  - `get_scan_parameters` (`SCSI_PARAM`, 18 bytes → authoritative width/lines/bytes_per_line) — `poc:487`
  - `get_gain_offset` (`SCSI_READ_GAIN_OFFSET`, 103 bytes) — `poc:432`. The PoC does a
    read-then-write round trip; `set_options` instead writes table values with `light=0` and a
    `# maybe in SANE is 5?` comment. The 29-byte payload offsets in `option.py:292-314` were
    verified to match the PoC byte-for-byte — only the `light` value and the missing round trip
    are open questions.
  - `get_ccd_mask` — was on the deleted `ScannerDevice`, now exists nowhere.
- [ ] 11. **Port the postprocessing** (`poc:561-660` → `postprocess.py`): `calculate_shading`,
  `build_width_to_loc`, `apply_shading_correction`. Pure numpy, and the most testable code in
  the project.
- [ ] 12. **Infrared exposure never sent** (`option.py`, the SET EXPOSURE loop): covers filters
  0x02/0x04/0x08 only; `exp_i` is packed into gain/offset but no SET EXPOSURE for filter 0x10.
- [ ] 12a. **`'gray'` mode returns untagged data.** Now that `set_mode` maps gray to
  `SCAN_COLOR_FORMAT_PIXEL`, gray scans come back as interleaved pixels with *no* per-line
  `RR`/`GG`/`BB`/`II` tag — so the tag-based deinterleave #9 needs must not be applied to them.
  Per `pieusb_scancmd.h:165-173`, in a single-filter scan only the first pixel of each triple
  holds valid data. Either handle both layouts in #9 or reject `'gray'` until it is handled.
- [ ] 12b. **Command order differs from the PoC/C.** `set_options` sends highlight/shadow before
  exposure; both references do exposure first (`poc:672-679`). Probably harmless, but
  `get_shading_parms` (#10) has to land *between* exposure and SET SCAN FRAME, so the ordering
  is worth settling at the same time.

## C. Safety — before the next hardware run

- [ ] 13. **The frame bounds check is gone.** The PoC refuses to send SET SCAN FRAME when the
  frame exceeds `max_scan_width`/`max_scan_height`, with a comment recording that it was added
  *after a real carriage crash* from an X/Y axis mix-up (`poc:660-683`). `option.py` has only
  per-option validators (`br_x <= inq.max_scan_w`), no cross-option check that `tl < br`, and
  `set_options` sends the frame unconditionally.
- [ ] 14. **Frame units are ambiguous.** `option.py` labels `tl_x`/`br_x` as `Unit.PIXEL` with
  defaults in `inq.max_scan_w`, but the PoC establishes these are **native-resolution units**
  (10000 dpi on the ProScan 10T), independent of the requested dpi — while `DESIGN.md:76` wants
  the public API in mm. Pick one, do the mm↔native conversion in a single place, and rename the
  unit accordingly.
- [ ] 15. `resolution` validator uses `v < min(max_x, max_y)` (`option.py:119`) — should be `<=`,
  otherwise the scanner's own maximum resolution is rejected.
- [x] 16. **`calibrate` default flipped to True.** It defaulted to False, which sets quality bit
  `0x08` (`skipShadingAnalysis`) and takes the path the PoC never validated: skipping the
  shading pass makes the device reject the following CCD MASK and GET PARAMETERS as "invalid
  command", and it is the only route to the `must_calibrate` branch in `scanner.py:99-108`.
  True matches the PoC's `skip_shading_analysis=False` default.

## D. API surface still missing vs `DESIGN.md`

- [ ] 16. `Scanner` has `__setitem__` but no `__getitem__`, and none of the
  options-as-attributes `__getattr__`/`__setattr__` magic that is the point of the python-sane shape.
- [ ] 17. Missing methods: `start()`, `snap()`, `arr_snap()`, `arr_scan()`, `get_parameters()`,
  `cancel()`, `close()`, `get_options()`, plus the virtual attrs `optlist` / `area` /
  `scanner_model`. `__exit__` closes but there is no standalone `close()`.
- [ ] 18. `__init__.py` exports only `get_devices` — needs `init()`, `open()`, `exit()`,
  `Scanner`, and the exceptions.
- [ ] 19. Resolve the two documented divergences (`DESIGN.md:107-117`): what `snap()` does for
  4-plane rgbi, and `get_parameters()` before vs. after `start()`.

## E. Module split (`DESIGN.md:44-56`), partially done

- [x] 20. **Delete `scanning.py`** — done.
- [ ] 21. Rename `option.py` → `options.py`, `usb_utils.py` → `usb.py`; fold `_device.py` into
  `__init__.py` + `inquiry.py`.
- [ ] 22. Not yet created: `commands.py`, `protocol.py`, `postprocess.py`, `exceptions.py`.
  Extracting `protocol.py` also fixes that `SCSI_SCAN_FRAME = 0x12` (virtual subcommand)
  currently collides with `SCSI_INQUIRY = 0x12` (opcode) in one namespace, and that
  `SCSI_REQUEST_SENSE` is defined twice (`transport.py:32,53`). The MODE SELECT constants
  added for `set_mode` (`SCAN_*`, `MODE_SETTINGS`, `COLOR_DEPTHS`) belong there too.
- [ ] 23. Dead/unused code:
  - `usb_utils.get_device(bus, address)` is now referenced nowhere — `open()` takes a
    `DeviceInfo`, so the `pieusb:bus:addr` string never needs resolving back to a device.
    Delete it, or keep it and settle #24 first.
  - unused `DeviceInfo` import in `inquiry.py:9` (and `ColorFormat` is imported twice there);
    unused `usb.core` in `scanner.py:2`.

## E2. Enumeration / open, decisions left over from A

- [ ] 24. **`open(DeviceInfo)` diverges from `DESIGN.md:20,45,121`**, which specifies
  `pieusb.open("pieusb:1:5")` and a `(name, vendor, model, type)` tuple from `get_devices()`.
  The comment at `_device.py:34-36` argues the string round-trip is pointless, which is a fair
  call — but then `DESIGN.md` should record the decision, and #23's `get_device()` goes away.
- [ ] 25. **`find_device()` returns only the first match per PID** (`usb_utils.py:30`, no
  `find_all=True`), so two identical scanners enumerate as one.
- [ ] 26. **Verify the re-open path on hardware.** `get_devices()` opens each device and closes
  it through `__exit__` → `dispose_resources()`, then `Scanner.__init__` calls
  `set_configuration()` + `claim_interface()` on that same `usb.core.Device`. Probably fine, but
  the PoC never exercised open → close → reopen.

## F. Tooling

- [ ] 27. **The package is not installed in `.venv`** — `import pieusb` fails without
  `PYTHONPATH=src`, so `tests/exercise.py` cannot run as-is. `uv pip install -e .`.
- [ ] 28. **No tests.** `tests/exercise.py` is two lines and needs hardware. `parse_inquiry` /
  `parse_*` and the whole shading pipeline are pure functions — worth a pytest suite against a
  captured INQUIRY blob and synthetic shading lines. Add pytest as a dev dependency.
  First case to write: feed `set_options` a fake device that records commands and assert the
  MODE SELECT payload equals `00 0f e8 03 80 04 04 00 01 02 00 00 00 80 10 00` for res 1000 /
  RGB / 8-bit / sharpen — that is the C backend's own documented capture and it caught the
  `colorFormat` assumption in the PoC.
- [ ] 29. `pyproject.toml:8` still says `"Add your description here"`; add Pillow if `snap()` is
  to return a `PIL.Image` as designed.
