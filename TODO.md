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

## A2. Blocking bugs in the new worker/callback code

- [ ] 30. **`scan()` always raises on its first call.** `scanner.py:231-235` checks
  `self.on_update`/`self.on_complete` — the *attributes*, still `None` from `__init__` — and only
  assigns the parameters afterwards. So the guard tests the previous call's state, and the first
  call always raises `PieusbError('Must set on_update and on_complete callbacks…')`. Verified: a
  fresh `Scanner` cannot start a scan. Check the parameters instead, and do it before any state
  is mutated.
- [ ] 31. **A failed `scan()` wedges the Scanner permanently.** `scan_in_progress = True` is set
  at `scanner.py:229`, *before* the callback guard at :231. When that guard raises — or the thread
  fails to start — the flag stays `True` and every later `scan()` raises `ScanInProgress` forever.
  Verified. Set the flag last, immediately before `.start()`, and clear it in the worker's
  `finally`.
- [ ] 32. **`on_complete` never fires and the flag never clears.** `_scan_worker` returns after
  its final `wait_ready()` (`scanner.py:210`) without calling `self.on_complete` or resetting
  `scan_in_progress`. Per `DESIGN.md` the completion callback must fire exactly once on *every*
  outcome — it is the only channel through which a caller learns the scan finished.
- [ ] 33. **The worker has no exception handling.** Anything raised inside `_scan_worker` kills
  the thread, prints to stderr, leaves `scan_in_progress` `True` and never notifies the caller.
  Per `DESIGN.md` "Threading contract": catch everything, attempt STOP SCAN, and deliver it as
  `result.error`. Needs #35 first, since `ScanResult` currently has nowhere to put it.
- [ ] 34. **Unreachable branch in `scan()`** (`scanner.py:225-227`). `_why_not_ready()` only
  returns a sense when `not_ready` is true (key 0x02); `must_calibrate` requires key 0x06. No
  sense satisfies both — verified exhaustively — so `reason.not_ready` above it always wins and
  the `must_calibrate` branch is dead. MUST_CALIBRATE arrives at START SCAN, not TEST UNIT READY,
  and the worker already handles it there.
- [ ] 34a. Minor: `raise WarmingUp` / `raise DeviceNotReady` raise the bare classes, discarding
  the sense `_why_not_ready()` just obtained. `wait_ready()` passes a message; these should too.
- [ ] 34b. Minor: `scanner.py` imports from `pieusb.transport` twice (:1, :15) and from
  `pieusb.exceptions` twice (:10, :25), with `WarmingUp`/`DeviceNotReady` in both blocks.

## B. Finish the scan path

- [ ] 9. **`_scan_worker` stops right after START SCAN** (`scanner.py:178-210`). Still missing
  everything downstream: the shading-reference read, CCD mask, GET PARAMETERS, the N-plane read,
  per-line tag deinterleave and image assembly. The PoC has all of it working (`poc:672-880`).
  `_get_shading_parms()` is now called and stashed in `self.shading_params`, but nothing consumes
  it yet.
- [x] 10. **The four missing commands are implemented** as private methods on `Scanner`:
  `_get_shading_parms` (:131), `_get_scan_parameters` (:147), `_get_ccd_mask` (:156) and
  `_get_gain_offset` (:159). Still open from the original item: `_get_gain_offset`'s result is
  never fed back into SET GAIN OFFSET, so the PoC's read-modify-write round trip
  (`poc:432-465`) does not happen yet — `set_options` writes table values with `light=0`. See
  also #22, these belong in `commands.py`.
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
- [ ] 12b. **GET SHADING PARMS is in the wrong place.** `_scan_worker` calls it *after*
  `set_options()` (`scanner.py:187`), but both references read it in the *middle* of the
  configuration sequence — after SET EXPOSURE / SET HIGHLIGHT SHADOW and before SET SCAN FRAME
  (`pieusb.c:944-948`, `poc:672-686`). As written, SET SCAN FRAME / gain-offset / SET MODE have
  all already gone out before the shading parameters are known. Since `set_options()` is one
  opaque block, the fix is either to split it at that seam or to move the read inside it.
  Relatedly, `set_options` still sends highlight/shadow before exposure where both references do
  exposure first.
- [ ] 12c. **Slide-transport models are not supported.** `sane_start()` issues two extra
  commands on scanners with `FLAG_SLIDE_TRANSPORT`, neither of which exists here:
  - `sanei_pieusb_cmd_17(device, 1)` after SET SCAN FRAME (`pieusb.c:1008-1023`), skipped on
    models carrying `FLAG_CMD_17_NOSUPPORT` — so a per-model quirk flag is needed too.
  - `sanei_pieusb_cmd_slide(device, SLIDE_INIT)` after SET MODE (`pieusb.c:1061-1073`), plus a
    further `cmd_slide` after the read to advance the slide (`pieusb.c:1226`), which is what the
    existing `advance` option is meant to drive.

  `inquiry.slide_transport` already identifies these models (DigitDia 6000, DigitDia 4000) and
  the `advance` option validates against it, but nothing sends the commands. The PoC is
  ProScan 10T only, so it is no help here — this is C-backend-only territory.

## C. Safety — before the next hardware run

- [x] 13. **Whole-table validation exists.** `OptionsTable.validate()` (`option.py:88-97`) checks
  `tl_x < br_x` and `tl_y < br_y`, raising the new `ParamError`, and `scan()` calls it on the
  calling thread before starting the worker — the position `DESIGN.md` and
  `sanei_pieusb_analyse_options` both put it in.
- [ ] 13a. **Still open from 13:** `validate()` does not re-check the frame against the device's
  reported bed. Per-option validators cover `br_x <= inq.max_scan_w` at assignment time, but only
  via `__setitem__` — and `__setattr__` (#16) does not exist yet, so `scanner.br_x = …` bypasses
  validation entirely today. Given the carriage crash this check was born from, `validate()`
  should assert the bounds itself rather than trusting that every write went through a validator.
  `sharpen` vs `fast_infrared` (`pieusb_scancmd.h:180`) is the other cross-option case.
- [ ] 14. **Frame units are ambiguous.** `option.py` labels `tl_x`/`br_x` as `Unit.PIXEL` with
  defaults in `inq.max_scan_w`, but the PoC establishes these are **native-resolution units**
  (10000 dpi on the ProScan 10T), independent of the requested dpi — while `DESIGN.md:76` wants
  the public API in mm. Pick one, do the mm↔native conversion in a single place, and rename the
  unit accordingly.
- [ ] 15. `resolution` validator uses `v < min(max_x, max_y)` (`option.py:119`) — should be `<=`,
  otherwise the scanner's own maximum resolution is rejected.
- [x] 15a. **`calibrate` default flipped to True.** It defaulted to False, which sets quality bit
  `0x08` (`skipShadingAnalysis`) and takes the path the PoC never validated: skipping the
  shading pass makes the device reject the following CCD MASK and GET PARAMETERS as "invalid
  command", and it is the only route to the `must_calibrate` branch in `scanner.py:99-108`.
  True matches the PoC's `skip_shading_analysis=False` default.

## D. API surface still missing vs `DESIGN.md`

`DESIGN.md` was reduced to `available_devices()` / `Scanner` / `scan()`; the python-sane-shaped
surface it used to specify is gone, so several old items here went with it.

- [ ] 16. **Options-as-attributes.** `Scanner` has `__setitem__` but no `__getitem__`, and no
  `__getattr__`/`__setattr__`, which is how `scanner.resolution = 5000` has to work. Note
  `__setattr__` needs care: `Scanner.__init__` assigns real attributes (`dev`, `params`), so it
  must fall through for anything not in the option table.
- [ ] 17. **`scan()` is non-blocking — worker landed, lifecycle missing.** `scan()` validates,
  pre-flight-checks readiness and spawns `_scan_worker` on a `threading.Thread`, and
  `scan_in_progress` is the busy guard. Still missing:
  - **`cancel()`** — no method, and no flag for the worker to poll. Must not touch the device
    itself (pyusb is not thread-safe); it sets a flag the worker checks between chunk reads, and
    the worker issues STOP SCAN.
  - **`wait(timeout=None)`** — `scan_thread` is stored but never joined by anything.
  - **`close()`** — no standalone method; and `__exit__` (`scanner.py:55`) still just calls
    `self.dev.close()`, so a `with` block that exits mid-scan releases the USB interface out from
    under the running worker. This was theoretical before; the worker now exists, so it is live.
  - **`phase`** — `ScanPhase` is emitted through `UpdateData` but not readable as a property, and
    only `CONFIGURING` is ever sent.
- [ ] 18. **Rename `get_devices()` → `available_devices()`** and export it, `Scanner`, the types
  and the exceptions from `__init__.py`, which currently exports only `get_devices`.
- [ ] 19. **Callbacks — shape landed, payload incomplete.** `scan(on_update, on_complete)` with
  `UpdateData`/`ScanResult`/`ScanPhase` in `types.py`. Two gaps against `DESIGN.md`:
  - `UpdateData` carries `phase`, `scanned_lines`, `total_lines` but **no `chunk` and no
    `plane`**, so there is nothing to drive a live preview with. Adding `phase` was a genuine
    improvement — it answers the "nothing reports progress before the pixel read" question the
    design left open — but the chunk needs to come back. See #19a.
  - `ScanResult` is `rgb` + `ir` with **no `error` and no `cancelled`**. That split is a neat
    answer to the RGBI-doesn't-fit-an-image-convention problem, but the threading contract has no
    other channel for reporting a worker failure, so those two fields are load-bearing. The
    design also listed `width`/`height`/`mode`/`color_depth`/`resolution`/`shading_corrected`/
    `duration_s`; `width`/`height` in particular come from GET PARAMETERS and can differ from
    what was requested.
  - `DESIGN.md` still says `progress=None, scan_complete=None`; the implementation names them
    `on_update`/`on_complete` and makes them required. Reconcile one way or the other.
- [ ] 19a. **Split each raw read by channel tag before reporting it.** Planes arrive as
  sequential blocks and the 255-line cap does not align to plane boundaries (`poc:794-818`), so
  one device read can straddle a channel transition and must produce two `progress` calls. This
  is also what makes a live preview possible at all: the other channels for a given row do not
  exist until much later in the scan.
- [ ] 19b. **Shading-correct per chunk, not at the end.** The shading reference and CCD mask are
  both read before the pixel data, so correction can be applied to each chunk as it arrives —
  required for `progress`'s `chunk` to be presentable. `apply_shading_correction` (#11) is
  currently written to take the whole assembled array; it needs a per-chunk entry point.
- [x] 19c. **Exception hierarchy** — `exceptions.py` created with `PieusbError` base,
  `DeviceNotReady` → `WarmingUp`, `CheckCondition`, `Timeout`, `TransportError` and
  `ScanInProgress`. `CheckCondition` moved out of `transport.py` (which now imports it), gained a
  `not_ready` predicate, and all six bare `IOError`/`TimeoutError` raises in `transport.py`
  became `TransportError`/`Timeout`.

  `TransportError` was added beyond the set listed in `DESIGN.md` — the bare `IOError`s needed a
  home, and folding them into `PieusbError` directly would have made the base class both a
  category and a concrete error.
- [x] 19d. **`ready()` / `wait_ready()` semantics** — `ready()` now maps every `NOT_READY` sense
  to `False` (non-NOT_READY senses still propagate, since those are real errors), and
  `wait_ready()` raises `WarmingUp` or `DeviceNotReady` instead of a builtin `TimeoutError`. A
  shared `_why_not_ready()` returns the sense so `wait_ready()` can tell the two apart.
- [ ] 19d-i. **Still open from 19d:** `ready()`/`wait_ready()` must raise `ScanInProgress` while a
  worker is running — pyusb is not thread-safe and the worker owns the device. Deferred until the
  worker exists (#17); there is no `_scanning` flag to guard on yet.
- [x] 19e. **`scan()` pre-flight readiness check** — done (`scanner.py:219-227`): `scan()` calls
  `_why_not_ready()` and raises `WarmingUp`/`DeviceNotReady` on the calling thread, and the worker
  keeps its own bounded warm-up retry at START SCAN (`scanner.py:190-209`), matching
  `pieusb.c:1088-1093`. See #34/#34a for two defects in the branch logic.

## E. Module split (`DESIGN.md` "Module structure"), partially done

- [x] 20. **Delete `scanning.py`** — done.
- [ ] 21. Rename `option.py` → `options.py`, `usb_utils.py` → `usb.py`; fold `_device.py` into
  `__init__.py` + `inquiry.py`.
- [ ] 22. Not yet created: `commands.py`, `protocol.py`, `postprocess.py`. (`exceptions.py` done.)
  `commands.py` now has an obvious first tenant: the four SCSI wrappers that landed as private
  `Scanner` methods (`_get_shading_parms`, `_get_scan_parameters`, `_get_ccd_mask`,
  `_get_gain_offset`) plus the ones inside `set_options`.
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

- [x] 24. **`open()` vs device-name strings — settled.** `DESIGN.md` now drops `open()` and the
  `pieusb:bus:addr` string entirely: `available_devices()` returns `DeviceInfo` objects carrying
  the live `usb.core.Device`, and `Scanner(info)` is constructed from one directly. Delete the
  `open()` wrapper in `_device.py:37-38` along with `usb_utils.get_device()` (#23).
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
- [ ] 29. `pyproject.toml:8` still says `"Add your description here"`. No Pillow dependency is
  needed — `DESIGN.md` now returns numpy only.
