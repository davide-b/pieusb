# pieusb — next steps

Status audit of the mid-refactor tree (see `DESIGN.md` for the target shape). Everything
compiles, but nothing runs end-to-end: enumeration raises, `Scanner` cannot be constructed,
and `scan()` stops right after START SCAN.

Cross-checked against the reference PoC (`~/sw/pieusb_ref/pieusb_proscan10t_poc.py`, cited
below as `poc:LINE`) and the SANE C backend (`~/backends/backend/pieusb*.c`).

**Suggested first slice:** section A (1–8) + D-20 + F-24 — restores `pieusb.get_devices()`
and drops the dead code, no hardware needed. Then C-13 before any scan attempt, then B.

## A. Blocking bugs — cannot enumerate or scan

- [ ] 1. **`get_devices()` is broken** (`src/pieusb/_device.py:14-32`): `devices` never
  initialized (declaration commented out at :15), `print(r)` + `quit()` debug leftovers at
  :23-24, `DeviceInfo` not imported, and the constructor is called with `id=` which is not a
  field of `types.DeviceInfo` (`dev, vendor, model, inquiry`). This is the one thing
  `tests/exercise.py` calls.
- [ ] 2. **`open()` returns the dead class** (`_device.py:34-37`): builds `ScannerDevice(dev)`,
  not `Scanner(info)`. Unresolved design point: `Scanner.__init__` needs a `DeviceInfo` (hence
  an INQUIRY), so `open(name)` must re-enumerate + inquire, and `DeviceInfo` needs to carry
  both the `usb.core.Device` and the `pieusb:bus:addr` name string that `get_devices()` hands out.
- [ ] 3. **`set(...)` misuse** (`option.py:51,53`): `set(Filter.RED, Filter.GREEN, Filter.BLUE)`
  raises `TypeError` — needs `{...}`. `generate_options()` therefore fails on first call, so
  `Scanner()` can never construct.
- [ ] 4. **Mode name typos + unsatisfiable default** (`option.py:51-61`): `'rbg'`/`'rbgi'` should
  be `'rgb'`/`'rgbi'`; the default `value='rgb'` does not validate against the list as written.
- [ ] 5. **`Option` is frozen but assigned to** (`option.py:30` vs `scanner.py:37`):
  `opt.value = value` raises `FrozenInstanceError`. Decide: make `Option` mutable, or keep it
  frozen and hold values in a separate dict / use `dataclasses.replace`.
- [ ] 6. **`Option` objects passed to `struct.pack`** (`option.py:284-288`):
  `x0 = options["tl_x"]` is missing `.value` for all four frame coordinates.
- [ ] 7. **`set_mode` is unfinished** (`option.py:316-327`): 11 format chars vs 5 args, result
  unassigned, and no `dev.command(SCSI_MODE_SELECT, ...)` — the file simply ends. Port the
  PoC version (`poc:465-483`): 16-byte payload, `data[1]=15`, dpi at 2, `MODE_PASSES[mode]`
  (0x80 color / 0x90 rgbi) at 4, `0x04`/`0x20` depth at 5, `colorFormat=0x04` at 6,
  `byteOrder=0x01` at 8, quality at 9, threshold at 13, `0x10` at 14.
- [ ] 8. **`KNWON_PIDS` → `KNOWN_PIDS`**, and **`MODEL_NAMES[model_nr]` KeyErrors on unknown
  models** (`inquiry.py:15,149`) — both already flagged in `DESIGN.md:125-126`, still open.

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
  - `get_ccd_mask` exists only on the dead `ScannerDevice` (`scanning.py:59`).
- [ ] 11. **Port the postprocessing** (`poc:561-660` → `postprocess.py`): `calculate_shading`,
  `build_width_to_loc`, `apply_shading_correction`. Pure numpy, and the most testable code in
  the project.
- [ ] 12. **Infrared exposure never sent** (`option.py:279-281`): the loop covers filters
  0x02/0x04/0x08 only; `exp_i` is packed into gain/offset but no SET EXPOSURE for filter 0x10.

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
- [ ] 15. `resolution` validator uses `v < min(max_x, max_y)` (`option.py:77`) — should be `<=`.
  `advance`'s validator ignores its argument (`option.py:115`).

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

- [ ] 20. **Delete `scanning.py`** — `ScannerDevice` is dead and duplicates `Scanner`'s
  `ready`/`wait_ready`/`start_scan`/`stop_scan`; only `_device.py`'s broken `open()` still
  references it.
- [ ] 21. Rename `option.py` → `options.py`, `usb_utils.py` → `usb.py`; fold `_device.py` into
  `__init__.py` + `inquiry.py`.
- [ ] 22. Not yet created: `commands.py`, `protocol.py`, `postprocess.py`, `exceptions.py`.
  Extracting `protocol.py` also fixes that `SCSI_SCAN_FRAME = 0x12` (virtual subcommand)
  currently collides with `SCSI_INQUIRY = 0x12` (opcode) in one namespace, and that
  `SCSI_REQUEST_SENSE` is defined twice (`transport.py:32,53`).
- [ ] 23. Minor: unused `DeviceInfo` import in `inquiry.py:9`, unused `usb.core` in `scanner.py:2`.

## F. Tooling

- [ ] 24. **The package is not installed in `.venv`** — `import pieusb` fails without
  `PYTHONPATH=src`, so `tests/exercise.py` cannot run as-is. `uv pip install -e .`.
- [ ] 25. **No tests.** `tests/exercise.py` is two lines and needs hardware. `parse_inquiry` /
  `parse_*` and the whole shading pipeline are pure functions — worth a pytest suite against a
  captured INQUIRY blob and synthetic shading lines. Add pytest as a dev dependency.
- [ ] 26. `pyproject.toml:8` still says `"Add your description here"`; add Pillow if `snap()` is
  to return a `PIL.Image` as designed.
