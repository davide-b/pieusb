# High-resolution scan stalls — concluded investigation

**Concluded:** 2026-08-11
**Hardware:** PIE ProScan 10T, rev 1.70, native 10000 dpi, bed 14370 × 9570 native units (36.5 × 24.3 mm)

## Outcome

The stall is a device characteristic with no host-side fix — and it fixed itself as
a side effect of the exposure work.

- The host is unattended for **1.9%** of a 5000 dpi scan. It is not the cause.
- The device's buffer **never fills**; it drains to 0 every fifth read.
- The C backend stalls identically at matched bit depth.
- The scanner reads out at ~1.1 MB/s regardless of resolution, and the reversals
  are what hold it there.
- **Raising `exp_rel_*` slows the line rate to match the readout and the reversals
  stop.** A 5000 dpi 16-bit scan at ×11.1 took **150 s against 190 s** at 100%.

So the correct exposure for a colour negative is also the exposure at which the
mechanism runs smoothly. `auto_exp` now derives it (v0.3.6), which means this needs
no further action.

The instrumentation written to measure all this was temporary and has been removed;
§3 records what it was, in enough detail to rebuild if a similar question comes up.

---

## 1. Symptom

At 5000 dpi the scanner pauses mid-scan, backs the carriage up, and re-scans the
same few lines. It does not happen at low resolutions. Working hypothesis: the
device's internal line buffer fills because the host does not drain it fast
enough, and the firmware recovers by reversing to re-acquire the lines it could
not buffer.

Nothing is known to be *wrong* with the resulting images — the symptom is time
lost and mechanical wear, plus the risk of duplicated or shifted lines.

**This hypothesis is wrong.** Measured, the host is unattended for 1.9% of a
5000 dpi scan. See §4.

## 2. Why the hypothesis looked plausible

> The per-read overhead table below is accurate and still the best wire-level
> reference. The *reasoning* built on it — resolution scaling and the host-side
> contributors — was disproved in §4.

### Fixed per-read protocol overhead

Every `GET SCANNED LINES` (max 255 lines) costs the following before one pixel
byte moves, all in `src/pieusb/transport.py`:

| Phase | Cost | Location |
|---|---|---|
| IEEE-1284 handshake | 11 control transfers **+ unconditional 3 ms `sleep`** | `ieee_command`, :139-146 |
| CDB | 6 control transfers, one byte each | `scsi_transaction`, :196-197 |
| Intermediate status | 1 control transfer | :199 |
| Bulk size | 1 control transfer | `_bulk_size`, :114 |
| **Bulk data** | `ceil(bytes / 16 KiB)` reads | `_bulk_in`, :124 |
| Final status | 1 control transfer | :209 |

≈20 control round-trips plus 3 ms of sleep per read. **This is faithful to the C
backend** — the 3 ms is `pieusb_usb.c:477`, the 255-line cap is
`pieusb_specific.c:2248`, the 16 KiB bulk chunking is `pieusb_usb.c:667`. It is
not a porting bug.

### The resolution scaling ~~argument~~ (WRONG)

The gap between reads is roughly constant in wall-clock time, but bytes per line
grow linearly with dpi. The device buffer is a fixed number of *bytes*, so it
holds proportionally fewer *lines* as resolution rises, while the carriage
delivers lines faster.

Measured: the gap is indeed constant (~66 ms at both resolutions) but it is far
too small to matter — 1.9% of a 5000 dpi scan. The premise held; the conclusion
did not.

Estimated 5000 dpi rgb 16-bit full-frame scan (derived from the INQUIRY bed; the
real geometry comes from `GET PARAMETERS` at scan time):

- width ≈ 7185 px, height ≈ 4785 lines, 3 planes → ≈14355 tagged lines
- 14372 bytes/line → 255-line read ≈ **3.66 MB**, ≈**57 reads**, ≈206 MB total

For comparison, at 300 dpi a line is ~860 B — roughly 16× fewer bytes per line.

### Host-side additions to the gap (ALL NEGLIGIBLE)

Three suspects, all measured and all noise:

1. **`self._emit(...)` per chunk** calls `on_update` on the worker thread — in
   `tests/exercise.py`, terminal I/O. Measured **0.1 ms**; the 5000 dpi runs with
   and without the progress bar are identical to three digits.
2. **`bytes(out)` in `_bulk_in`** copies ~3.66 MB per read. ~1 ms out of 4330.
3. **The per-line Python decode loop**, 255 iterations plus page faults on the
   206 MB `planes` array. Measured **1.0 ms** total per read.

## 3. Instrumentation used (since removed)

Written for this investigation, kept out of the repository once it had answered the
question. Not committed, so it exists only as this description.

| Piece | What it did |
|---|---|
| `src/pieusb/profile.py` | `ReadRecord` / `ReadProfile`: one record per pixel read, a `summary()` for the log and `write_csv()` |
| `UASDevice.scsi_transaction` | filled `dev.last_timing` with per-phase timings — `ieee_s`, `cdb_s`, `status_s`, `bulk_size_s`, `bulk_s`, `bulk_bytes`, absolute `bulk_start`/`bulk_end`, `setup_s`, `total_s` — plus a `dev.transactions` counter so a retried command was distinguishable |
| `UASDevice.link_info()` | reported the bulk endpoint's `wMaxPacketSize` (64 = full speed, 512 = high) |
| `transport.BULK_CHUNK` | made overridable to test whether the data phase was chunk-limited |
| `Scanner._scan_pass` | built a `ReadProfile` in the pixel loop and logged the summary at INFO |
| `Scanner.probe_device` | polled GET PARAMETERS after every read for `available_lines` and the motor bit |
| `Scanner._read_state()` | ported READ STATE (0xDD) |
| `Scanner._get_scan_parameters()` | decoded the motor-direction bit at byte 16 |
| `tests/exercise.py` | `--profile FILE`, `--probe-device`, `--quiet-progress`, `--bulk-chunk KIB`, a `USB link:` line |

The field that mattered was **`gap_s`**: end of one bulk transfer to the start of
the next — the window in which the device is scanning and nothing is draining it.
Not a sum of the other fields; it spans the previous cycle's decode and callback
plus this cycle's setup.

Two findings about the device's own status commands are worth keeping:

- **`GET PARAMETERS` byte 14 = `available_lines`** is the buffer-fill gauge,
  non-zero only mid-scan (`pieusb_scancmd.h:140`), and byte 16 bit 0 is the motor
  direction (`:141`), which the C declares and never decodes. Both readable
  mid-scan. `available_lines` survives in `_get_scan_parameters()`.
- **`READ STATE` (0xDD) is rejected mid-scan** by this device: CHECK CONDITION,
  ILLEGAL REQUEST / INVALID COMMAND OPERATION CODE (key 0x05, code 0x20). The C
  only issues it at the top of `sane_start`, before START SCAN.

## 4. RESULTS — measured 2026-08-11

Raw CSVs and logs were kept out of the repository; the numbers below are the whole
of it. **The buffer-starvation hypothesis is wrong.** The host is
idle almost the whole scan.

| | 1000 dpi | 5000 dpi | 5000 dpi + progress bar |
|---|---|---|---|
| reads | 12 | 57 | 57 |
| MB/read | 0.73 | 3.66 | 3.66 |
| total | 8.3 MB in 6.9 s | 206 MB in 192.9 s | 206 MB in 192.9 s |
| `setup_s` mean | 59.5 ms | 59.7 ms | 59.7 ms |
| `bulk_s` mean | 513 ms | 3319 ms | 3319 ms |
| `decode_s` mean | 0.5 ms | 1.0 ms | 1.0 ms |
| `emit_s` mean | 0.0 ms | 0.0 ms | 0.1 ms |
| **`gap_s` mean** | **65.7 ms** | **66.5 ms** | **66.5 ms** |
| **unattended** | **10.5%** | **1.9%** | **1.9%** |
| aggregate rate | 1.20 MB/s | 1.07 MB/s | 1.07 MB/s |
| retries | 0 | 0 | 0 |

### What this kills

- **The gap is 66 ms and 1.9% of a 5000 dpi scan.** The host cannot be
  overflowing the device's buffer; it spends 98% of the scan blocked inside
  `ep_in.read()`.
- **`decode_s` is 1 ms and `emit_s` is 0.1 ms.** The progress bar and the
  per-line Python loop are noise. The 5000 dpi runs with and without the bar are
  identical to three digits. Every host-side fix considered was pointless — the
  ~1 ms `bytes(out)` copy included.
- **The gap is constant at ~66 ms across a 25× change in data volume**, so it is
  pure protocol overhead and does not scale with resolution. It matters *more* at
  low resolution (10.5% at 1000 dpi), the opposite of the prediction.

### What it points at instead

**Scan time is set by data volume at a flat ~1.1 MB/s.** 25× the bytes took 28×
the time. A mechanical limit would scale with *lines* (5×), not bytes.

`setup_s` is 59.7 ms, not the ~11 ms estimated — 20 control transfers at ~3 ms
each. Three milliseconds for a one-byte control transfer is a slow tunnel.

The `bulk_s` series at 5000 dpi is a clean period-3 cycle:

```
4.33  1.34  4.34   4.33  1.34  4.34   4.33  1.34  4.34  ...
```

- slow reads: 3.66 MB / 4.33 s = **0.85 MB/s**
- fast reads: 3.66 MB / 1.34 s = **2.73 MB/s**
- cycle mean: 11 MB / 10.01 s = **1.10 MB/s**

1000 dpi shows the same shape: 0.35 s for the first three reads (2.09 MB/s, buffer
already primed), settling to 0.64 s (1.15 MB/s).

The 2.7 MB/s peaks matter: they are **above full-speed USB's 1.5 MB/s ceiling**,
so the link is not full speed and 1.1 MB/s is not a raw link limit. The device
bursts from a primed buffer, then the read blocks waiting for production.

Working explanation: this transport is SCSI tunnelled over **IEEE-1284** over USB
(`ieee_command`, `PORT_PAR_CTRL`, `PORT_PAR_DATA`). EPP/ECP parallel throughput is
~1–2 MB/s in practice, which is exactly what is measured, and ~3 ms per one-byte
control transfer is what a byte-at-a-time tunnel over that bridge would cost. If
that is the ceiling, **the back-up is the scanner's own flow control** — it fills
its buffer faster than the bridge drains it, and a CCD carriage cannot pause in
place, so it decelerates, reverses and re-approaches. Nothing in the host software
would change that.

### Instrumentation bug found

`--probe-device` crashed 1 read into the 5000 dpi scan:

```
[cmd 0xdd] status=0x02
CHECK CONDITION key=0x05 code=0x20 qualifier=0x00
```

`0xDD` is READ STATE; key 0x05 / code 0x20 is ILLEGAL REQUEST / INVALID COMMAND
OPERATION CODE. **This device rejects READ STATE while a scan is running.** The C
only issues it at the top of `sane_start`, before START SCAN. My mistake.

`GET PARAMETERS` (0x0f) *is* accepted mid-scan — it returned 18 bytes cleanly
right before the crash — and carries the motor bit anyway, so the probe now uses
GET PARAMETERS alone. `_read_state()` is kept for idle use with the restriction
documented. Fixed, uncommitted.

Also from that log: real geometry at 5000 dpi is **7184 × 4786**, 14368 bytes/line,
3 planes → 14358 tagged lines, and `available_lines` was 0 before START SCAN as
expected.

### Round 2 — chunk size ruled out, and a confound in the C comparison

**`--bulk-chunk 512` changed nothing** -- identical to the 16 KiB run to three
significant figures: 1.07 MB/s aggregate, `bulk_s` p50
4327 ms, same period-3 cycle. 229 reads of 16 KiB and 8 reads of 512 KiB perform
the same, so the data phase is not chunk-limited — `ep_in.read()` is waiting on the
device. Cross it off.

**`scanimage` at 5000 dpi reportedly never backed up.** Before concluding the C
avoids something we do not, note that **the C's default bit depth is 8**:
`bpp_list[2] = 8` (`pieusb_specific.c:460`, element 0 is the SANE list count),
assigned to `OPT_BIT_DEPTH` at `:671`. Unless `--depth 16` was passed explicitly,
that run moved **103 MB where ours moved 206 MB**, at half the bytes per line —
which is exactly the quantity that decides whether the mechanism outruns the
bridge. The comparison is not apples-to-apples until the depth is pinned.

Two further observations that argue *against* the C being cleverer:

- Its read loop is `malloc` → `sanei_pieusb_cmd_get_scanned_lines` → copy into the
  buffer → `free` (`pieusb_specific.c:2232-2300`), with no sleeps, no extra
  commands and no waits. Its CDB carries the line count with the byte count passed
  separately (`pieusb_scancmd.c:464`) — byte-for-byte what we do. It also copies
  into a *file-backed* buffer, which is more work than our numpy write.
- On the AUTO path it applies `gain-adjust` 0.8, shortening exposure to ~3706
  against our 4100 — a ~10% *higher* line rate, making it more likely to outrun
  the bridge, not less.

**New reading of the period-3 cycle: it may be the back-up itself.**

| | tagged lines/s | rate |
|---|---|---|
| slow reads (4.33 s) | 59 | 0.85 MB/s |
| fast reads (1.34 s) | 190 | 2.73 MB/s |

If the 4.33 s reads are slow because they contain a reversal, and 190 lines/s is
the clean rate, then a scan with no back-up would move 206 MB at 2.73 MB/s in
**~75 s instead of 193 s**. That is a falsifiable prediction: a 16-bit `scanimage`
run at 5000 dpi should take ~75-95 s if it truly streams, and ~190 s if it backs
up like ours.

### Round 3 — CONCLUDED: the buffer never fills, it empties

57 probed reads at 5000 dpi / 16-bit. `available_lines`:

```
74 31 60 114 72 30 63 114 71 28 66 112 69 27 69 111 66 24 75 107 63 21 78 106
62 20 81 105 60 0 21 81 104 60 0 20 81 104 60 0 20 81 104 60 0 20 81 104 60 0
20 81 104 61 0 21 2
```

min **0**, max **114** (1.64 MB at 14370 B/line), mean 58. From read 29 it settles
into a perfectly regular period-5 cycle — `0, 20, 81, 104, 60` — **draining to
empty every fifth read**. The motor bit was never observed set.

**The buffer never overflows. It repeatedly empties.** The host outpaces the
device, so the device is the slow party and the host cannot be the cause of the
reversal. This holds regardless of whether 114 lines is the buffer's capacity or
merely as much as it happened to accumulate: at the observed ~74 tagged lines/s,
the 66 ms gap (133 ms with probes) adds **5-10 lines** of fill. Overflowing even a
114-line buffer would need the host to stop reading for ~1.5 s.

Both host-side theories are therefore dead:

1. Not a large-gap overflow — the gap is 1.9% of the scan.
2. Not a small-buffer overflow — the buffer drains to zero, and the gap is worth
   ≤10 lines of the ≥114 available.

The oscillation is production against 255-line gulps, and the period tracks the
timing rather than anything in the device: it was 3 without probes, 4-5 with them.

**Conclusion: this is a device characteristic and there is no host-side fix.** The
scanner reads out at ~1.1 MB/s regardless of resolution (1.20 MB/s at 1000 dpi,
1.07 at 5000), which sets a floor of ~190 s on a 206 MB scan. The reversal is
internal — most plausibly the carriage being unable to creep smoothly at the
~0.127 mm/s that 5000 dpi at that readout rate demands — and the dead time during
each reversal is what drains the buffer to zero and holds the average at 1.1 MB/s.

Caveat on the evidence: `available_lines` is sampled once per read, immediately
after a read drains it, so these are post-drain levels rather than peaks. Peaks are
at most ~10 lines higher (production during the gap), which does not change the
conclusion.

### Round 3 addendum — `exp_time` appears to be inert on this device

Raising `exp_time_*` from 4100 to 8200 produced **no change in image brightness**.
That is consistent with two earlier oddities that were noted and not chased:

- Sending exposure `(0, 0, 0)` produced R max 10218, not a black frame.
- The device always reports exposure as a flat 4100 for all four channels, which
  `pieusb_scancmd.h:194-199` describes as a *reset default* pattern rather than a
  per-channel optimum.

This was the thread that led to §6, where every control was swept in isolation:
`exp_rel_*` turned out to be the only one that does anything, and `gain_*`,
`exp_time_*` and `light` are all inert.

## 5. Device quirks already established

Confirmed on this unit, from `scan.log` and a SANE backend trace:

- **Saturation levels are always `(0, 0, 0)`**, cold and warm. Auto-exposure by
  metering is therefore impossible — `dg = 0` → gain 0, exposure 0. This is why
  the "calibration from preview" port was removed.
- **A cold scanner reports zeros for its whole calibration**: exposure
  `(0, 0, 0, 4100)`, gain `(0, 0, 0, 15)`, offset all 0, light 0. It fills them in
  during its **first scan**. Irrelevant now that nothing reads those values, but it
  is why `auto_exp` used to need two scans.
- **Warm values** on this unit: exposure `(4100, 4100, 4100, 4100)`, gain
  `(44, 28, 37, 57/58)`, offset `(12..18, 16..25, 16..26, 16..25)`, light `4`.
  Exposure is a flat constant, not a per-channel optimum.
- **The light byte does nothing.** Measured directly: light 4, 5, 6 and 7 produce
  the same image to within 0.1%. An earlier claim here that it was "worth roughly
  the entire exposure range" came from a comparison confounded by resolution and was
  wrong. Sending 0 was still incorrect — outside the documented 4..7 band and not
  what the C sends — so the default of 4 stands on correctness grounds alone.
- **The C's `gain-adjust` default is `* 0.8`**, not the 1.0 its comment at
  `pieusb_specific.c:743` claims (`gain_adjust_list[2]`). We send the device's
  values unscaled, equivalent to `* 1.0`.

## 6. What the exposure work settled

Chasing the stall turned up that most of the exposure controls do nothing. Measured
by sweeping each in isolation at 300 dpi / 16-bit:

| control | values tried | effect |
|---|---|---|
| `gain_*` | 10, 30, 60 | none (±0.1%) |
| `exp_time_*` | 500, 1500, 2500, 2937, 6000, 10000 | none (±0.2%) |
| `light` | 4, 5, 6, 7 | none (±0.1%) |
| `exp_rel_*` | 50 | none — clamps at 100 |
| `exp_rel_*` | 200, 250, 400, 800, 1086, 1500 | **×1.97, ×2.46, ×3.93, ×8.03, ×10.5, ×14.7** |
| `exp_rel_*` | 1598 / 1599 | ×15.98 / **×1.00** — the Timer 1 overflow step |
| `exp_rel_*` | 2200, 3300, 5000, 7500, 10000 | wrapped: ×5.8, ×1.1, ×2.0, ×10.7, ×3.9 |

`exp_rel_*` is the exposure system: linear to within 4%, per channel, upward only,
and bounded by a **16-bit Timer 1 overflow**. It scales the device's own absolute
exposure time into that register:

    timer = exposure_time * exp_rel / 100      (mod 65536)

so the ceiling is `65535 * 100 // exposure_time`, which for the 4100 this unit
reports is 1598. A binary search put the step at exactly 1598/1599 for all three
channels. Above it the exposure *drops* — and lands on ×1.00 rather than near zero
because a wrapped result below 100 meets the same floor that makes `exp_rel=50`
behave as 100. The model predicts every measurement to within 4%:

    exp_rel   1086   1500  |  1598  1599  |  2200  3300  5000  7500  10000
    predicted x10.9  x15.0 | x15.98 x1.00 | x6.02 x1.03 x2.05 x11.1  x4.09
    measured  x10.5  x14.7 |   clip x1.00 | x5.79 x1.06 x2.01 x10.7  x3.94

Which also explains why `exp_time_*` looked inert without being irrelevant: the
device ignores what is written to it and keeps its own value, and that value is the
multiplicand. It is the per-line integration period, so the line rate halves
as it doubles — which is why it also cures the stall.

Per-channel 247/563/1086 puts all three channels within 1% of each other at ~88% of
full scale with nothing clipped, against 4.39 : 1.93 : 1.00 with red at 34%
untouched. `auto_exp` derives exactly those numbers from a metering pass (v0.3.6).

Driving any channel to saturation produces **vertical banding**, not flat white: the
shading correction's per-column gain is >1 where the lamp falls off, so edge columns
reach the ceiling at a lower raw value than centre columns. At `exp_rel` 400 red
clipped 12.7% of samples, concentrated at the left and right edges with 55
percentage points of column-to-column swing. `apply_shading_correction` now warns
when it clamps.

## 7. Open question, unresolved

`~/scan_tests/scanner2.c` sets `calibration = "from preview"` on both passes
(`:369`, `:386`), so its real scan should take the `SCAN_CALIBRATION_PREVIEW`
branch, compute `dg = 0` from the zero saturation levels, and send exposure 0 /
gain 0 — which produced a dark scan when this library did the same thing. It
instead produces a good scan. Either `preview_done` is false there, or the device
tolerates exposure 0 differently than observed.

Not on the critical path — `auto_exp` now adopts the device's calibration and
works — but it means the mechanism behind that C program's good output is not
fully understood. One command settles it:

```bash
SANE_DEBUG_PIEUSB=5 ./scanner2 ... 2>&1 | grep -E "mode = |calibration data from|additional gain"
```

- `get calibration data from scanner` → it took the AUTO path; everything is
  explained by warm-vs-cold.
- `from preview` with `additional gain 0.000000` and a good scan → the device
  substitutes its own values for zeros, and something else explains the dark scan
  this library produced.

## 8. Key C source references

Paths relative to `~/backends/backend/`.

| What | Where |
|---|---|
| 3 ms per-command sleep | `pieusb_usb.c:477` |
| Bulk read, 16 KiB chunks | `pieusb_usb.c:651-670` |
| 255-line read cap | `pieusb_specific.c:2248` |
| `SET GAIN OFFSET` payload layout | `pieusb_scancmd.c:1102-1116` |
| `GET GAIN OFFSET` field offsets | `pieusb_scancmd.c:1042-1051` |
| Light level semantics (band 4..7) | `pieusb_scancmd.h:208-213` |
| `available_lines`, motor field | `pieusb_scancmd.h:140-141` |
| `READ STATE` decode, scanning/motor bits | `pieusb_scancmd.c:1167-1193`, `pieusb_scancmd.h:129` |
| Calibration modes; AUTO is the default | `pieusb_specific.c:733`, `:1885-2046` |
| `updateGain2` | `pieusb_specific.c:2528` |
| `gain-adjust` option, default `* 0.8` | `pieusb_specific.c:735-743` |
| `sane_start` sequence | `pieusb.c:865-1140` |
