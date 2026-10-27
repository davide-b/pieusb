#!/usr/bin/env python3
'''
Unattended hardware suite: characterise a scanner, or check that a verified one
still behaves as before.

    python tests/hwsuite.py --output rps10m-suite                    # characterize
    python tests/hwsuite.py --profile regression --output 10t-suite  # scans only
    python tests/hwsuite.py --allow focus --output new-model         # unknown model

Every stage declares a risk, and runs only if the profile admits that risk and
the scanner has the hardware it drives:

    read-only   identity                       nothing changes
    scan        default_scan, exposure         ordinary scans
    focus       autofocus                      moves the focus motor
    transport   offsets, stepping, redetect,   moves film; also needs film loaded
                rewind, walk, recovery

--profile characterize (the default) admits every risk; regression admits
read-only and scan stages and runs the library exactly as configured for the
model. The suite then drives no motor itself, but the library still does what
it does in normal use: on a model it drives focus on, a SLIDE focus command
precedes every scan. Which motors a model has comes from pieusb.inquiry.MODEL_MOTORS. A model
not in it gets none unless --allow names them -- an unknown scanner may have no
transport, and a ProScan 10T must never be sent a SLIDE -- and --allow cannot
add a motor a known model lacks.

Model-specific numbers are not kept here. Test positions derive from what the
scanner reports (scan window, focus range) and library constants (offset step);
expected results from the library's configuration for the model (the exposure
stage checks Capabilities.exposure_control). What is left are tolerances.

Load film (for transport stages: a strip of colour negative at its first frame),
start it, and leave it. It stops on its own: at the end of the strip, or at the
first transport or focus command that fails, since carrying on after one risks
the film. Every run ends by packing the output folder as <folder>.zip beside
it, the one file to send back: suite.log has everything the terminal showed
plus every command and reply, summary.json what each stage found and a
suggested capability entry, and each stage's folder its scans as .npy.

Transport stages, in an order that needs no reloading:

    offsets      small offsets at 600 dpi, and the centring move CyberView uses,
                 each scanned and returned
    stepping     NEXT 1, NEXT 2, PREV with byte 3 = 1, PREV with byte 3 = 0
    redetect     NEXT after deliberately shifting the film each way: does NEXT
                 find the frame, or move a fixed distance? Then PREV and the
                 shift undone, so later stages start where this one did
    stepsize     offsets of 32, 64 and 128 steps, each out and back, the image
                 shift measured: millimetres per step, direction and return error
    rewind       PREV back to frame 0, then a scan to compare with the start
    walk         every frame to the end: home and both centring offsets
    recovery     PREV from the end, and a scan
    framing      back to frame 0, then every frame to the end as roll scanning
                 would: a preview, the frame located (pieusb.framing), one offset
                 to centre it and a check preview of where it landed. Preview
                 timings at a few resolutions on the first frame

A stage that runs out of film is recorded as partial and the suite goes on.
'''

import argparse
import json
import logging
import shutil
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from enum import IntEnum
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parent))

import exercise as ex
from exercise import log

import numpy

import pieusb
from pieusb.framing import FRAME_MM, FramePosition, find_bands, locate_frame
from pieusb.exceptions import DeviceNotReady, PieusbError, WarmingUp
from pieusb.inquiry import MODEL_MOTORS
from pieusb.option import EXPOSURE_TIME_MAX
from pieusb.scanner import MM_PER_INCH, OFFSET_STEPS_PER_MM, Scanner
from pieusb.transport import SLIDE_PREV
from pieusb.types import ExposureControl


class Risk(IntEnum):
    READ_ONLY = 0
    SCAN = 1
    FOCUS = 2
    TRANSPORT = 3


# The motor a risk drives, as a Capabilities field name.
RISK_MOTOR = {Risk.FOCUS: 'focus', Risk.TRANSPORT: 'film_transport'}
MOTORS = tuple(RISK_MOTOR.values())


@dataclass(frozen=True)
class Stage:
    name: str
    risk: Risk
    # Transport stages also need film loaded, which only READ STATE can tell.
    needs_medium: bool = False


STAGES = (
    Stage('identity', Risk.READ_ONLY),
    Stage('default_scan', Risk.SCAN),
    Stage('exposure', Risk.SCAN),
    Stage('autofocus', Risk.FOCUS),
    Stage('offsets', Risk.TRANSPORT, needs_medium=True),
    Stage('stepsize', Risk.TRANSPORT, needs_medium=True),
    Stage('stepping', Risk.TRANSPORT, needs_medium=True),
    Stage('redetect', Risk.TRANSPORT, needs_medium=True),
    Stage('rewind', Risk.TRANSPORT, needs_medium=True),
    Stage('walk', Risk.TRANSPORT, needs_medium=True),
    Stage('recovery', Risk.TRANSPORT, needs_medium=True),
    Stage('framing', Risk.TRANSPORT, needs_medium=True),
)
STAGE_NAMES = tuple(s.name for s in STAGES)

# The highest risk each profile admits.
PROFILES = {'characterize': Risk.TRANSPORT, 'regression': Risk.SCAN}

PREVIEW_DPI = 300
OFFSET_DPI = 600
PATCH_DPI = 2500
PATCH_MM = 10
FOCUS_SWEEP_POINTS = 8
SMALL_OFFSET_MM = 2.0
REDETECT_OFFSET_MM = 3.0
# CyberView's centring move on an RPS 10M, 70 steps for a 36.5 mm window; other
# windows scale it.
CENTRING_STEPS = 70
CENTRING_WINDOW_MM = 36.5
MAX_WALK_FRAMES = 60
# Offsets the stepsize stage measures, in steps; the field carries 1..255. The
# largest must move the image less than half the window, or phase correlation
# wraps: 128 steps is about 15 mm against a 36.5 mm window.
STEP_SIZES = (32, 64, 128)
# The framing stage leaves a frame alone closer to centre than this: one step is
# about 0.11 mm, and an out-and-back move returns only to 0.3-1 mm.
FRAMING_MIN_SHIFT_MM = 0.25
# A correction at least this long is measured well enough to refine the offset
# scale: 0.5 mm is 6 px at 300 dpi.
FRAMING_SCALE_MIN_MM = 0.5
# Preview resolutions timed on the first frame, and whether each finds the same
# frame; the loop itself previews at PREVIEW_DPI.
PREVIEW_TIMING_DPI = (100, 150, 300)

# Exposure response: a control is live if it moves the level by more than
# EXPOSURE_LIVE, and doubles it if every channel is within EXPOSURE_LINEAR of 2.
# Levels are compared over pixels below EXPOSURE_MASK of full scale in the base
# scan, so a doubled pixel cannot clip, as the slope between EXPOSURE_BLOCK
# square block means: averaging keeps noise from flattening the slope, and a
# slope ignores the black level, which moves between passes that run the
# shading analysis (ProScan 10T blue: -348 of ~2100, same slope).
EXPOSURE_LIVE = 0.10
EXPOSURE_LINEAR = 0.25
EXPOSURE_MASK = 0.4
EXPOSURE_BLOCK = 8
EXPOSURE_MIN_BLOCKS = 16
GAIN_STEP = 8
GAIN_MAX = 63

# A focus sweep whose sharpest scan beats its softest by this much shows the
# focus motor does something.
FOCUS_EFFECT = 1.2


class Halt(Exception):
    '''A motor command failed: stop the suite rather than move anything further.'''


class EndOfStrip(Exception):
    '''NEXT left the frame counter where it was.'''


class Skip(Exception):
    '''The stage cannot run on this scanner as it is now.'''


def motors_for(model: int, allow: frozenset) -> frozenset:
    '''The motors the suite may drive: the known hardware, or what --allow names.'''
    if model in MODEL_MOTORS:
        lacking = allow - MODEL_MOTORS[model]
        if lacking:
            raise SystemExit(f"--allow {', '.join(sorted(lacking))}: model 0x{model:04x} "
                             f"is known to have no such hardware")
        return MODEL_MOTORS[model]
    return allow


def select_stages(profile: str, motors: frozenset, only=None) -> list[tuple[Stage, str | None]]:
    '''Every stage, with the reason it will not run, or None if it will.'''
    plan = []
    for stage in STAGES:
        reason = None
        motor = RISK_MOTOR.get(stage.risk)
        if only is not None and stage.name not in only:
            reason = 'not selected by --only'
        elif stage.risk > PROFILES[profile]:
            reason = f"the {profile} profile does not run {stage.risk.name.lower()} stages"
        elif motor is not None and motor not in motors:
            reason = f"no {motor} motor known on this model (--allow {motor} on an unknown one)"
        plan.append((stage, reason))
    return plan


def block_means(a: numpy.ndarray, k: int = EXPOSURE_BLOCK) -> numpy.ndarray:
    h, w = a.shape[0] // k * k, a.shape[1] // k * k
    if not h or not w:
        return numpy.zeros((0, 0, a.shape[2]))
    return a[:h, :w].astype(numpy.float64).reshape(h // k, k, w // k, k, -1).mean(axis=(1, 3))


def level_ratio(base: numpy.ndarray, other: numpy.ndarray) -> list[float]:
    '''Per channel, how `other` scales `base`: the slope between their block
    means below EXPOSURE_MASK of full scale in `base`. An image too small or too
    flat for a slope gets the ratio of its means instead.'''
    full = numpy.iinfo(base.dtype).max
    b_blocks, o_blocks = block_means(base), block_means(other)
    ratios = []
    for c in range(base.shape[2]):
        b, o = b_blocks[:, :, c].ravel(), o_blocks[:, :, c].ravel()
        mask = b < EXPOSURE_MASK * full
        if mask.sum() >= EXPOSURE_MIN_BLOCKS and b[mask].std() > 1e-6 * full:
            ratios.append(round(float(numpy.polyfit(b[mask], o[mask], 1)[0]), 4))
            continue
        b = base[:, :, c].astype(numpy.float64)
        o = other[:, :, c].astype(numpy.float64)
        mask = b < EXPOSURE_MASK * full
        if mask.sum() < 0.01 * mask.size:
            mask = numpy.ones_like(mask)
        ratios.append(round(float(o[mask].mean() / max(b[mask].mean(), 1e-9)), 4))
    return ratios


def doubles(r: list[float]) -> bool:
    return all(abs(v - 2) <= EXPOSURE_LINEAR for v in r)


def live(r: list[float]) -> bool:
    return any(abs(v - 1) > EXPOSURE_LIVE for v in r)


def exposure_verdict(ratios: dict, configured: str) -> tuple[str, list[str]]:
    '''Which exposure modes work on a model, and the one to use.

    The modes differ on a pass that runs the shading analysis, which re-derives
    the exposure on an ABSOLUTE model. RELATIVE works if exp_rel doubles the image
    on such a pass; ABSOLUTE if exp_time doubles it on a pass that skips it. A
    skipping pass alone cannot tell them apart: on a ProScan 10T and 4000 both
    controls double there. The mode the library is configured with wins if it
    works.
    '''
    works = []
    if 'exp_rel_x2_analysed' in ratios and doubles(ratios['exp_rel_x2_analysed']):
        works.append(ExposureControl.RELATIVE.name)
    if doubles(ratios['exp_time_x2']):
        works.append(ExposureControl.ABSOLUTE.name)
    if configured in works:
        return configured, works
    return (works[0] if works else 'undetermined'), works


def focus_sweep(lo: int, hi: int, points: int = FOCUS_SWEEP_POINTS) -> list[int]:
    if hi - lo + 1 <= points:
        return list(range(lo, hi + 1))
    return sorted({round(lo + i * (hi - lo) / (points - 1)) for i in range(points)})


def image_sharpness(rgb: numpy.ndarray) -> float:
    '''Red channel, as autofocus scores it: mean |gradient| over level, both axes.'''
    red = rgb[:, :, 0].astype(numpy.float64)
    level = red.mean()
    if level <= 0:
        return 0.0
    return float((numpy.abs(numpy.diff(red, axis=0)).mean()
                  + numpy.abs(numpy.diff(red, axis=1)).mean()) / level)


def image_shift_px(a: numpy.ndarray, b: numpy.ndarray) -> int:
    '''How many columns the content of `b` lies towards +x of `a`'s, by phase
    correlation of log red. Both at one resolution.'''
    h, w = min(a.shape[0], b.shape[0]), min(a.shape[1], b.shape[1])
    x, y = (numpy.log(img[:h, :w, 0].astype(numpy.float64) + 64) for img in (a, b))
    window = numpy.hanning(h)[:, None] * numpy.hanning(w)[None, :]
    fa, fb = numpy.fft.fft2((x - x.mean()) * window), numpy.fft.fft2((y - y.mean()) * window)
    cross = fb * numpy.conj(fa)
    cross /= numpy.abs(cross) + 1e-12
    corr = numpy.real(numpy.fft.ifft2(cross))
    dx = int(numpy.unravel_index(numpy.argmax(corr), corr.shape)[1])
    return dx - w if dx > w // 2 else dx


def placement_fields(bands, frame: FramePosition | None) -> dict:
    fields = {'bands': [{'kind': b.kind.value, 'start': b.start, 'stop': b.stop,
                         'clipped': [b.clipped_start, b.clipped_stop],
                         'confidence': round(b.confidence, 2)} for b in bands]}
    if frame is None:
        return {**fields, 'frame': None}
    return {**fields, 'frame': {
        'start': round(frame.start, 1), 'stop': round(frame.stop, 1),
        'measured': [frame.measured_start, frame.measured_stop],
        'confidence': round(frame.confidence, 2),
        'shift_mm': round(frame.shift_mm, 3),
        'overhang_mm': [round(v, 2) for v in frame.overhang_mm],
        'width_mm': round(frame.width_mm, 2)}}


class Suite:
    def __init__(self, scanner: Scanner, out: Path) -> None:
        self.scanner = scanner
        self.out = out
        self.stages: list[dict] = []
        self.record: dict | None = None
        self.started = time.monotonic()
        self.prev_byte3 = 0x01
        self.medium_present: bool | None = None
        # Offsets sent since the last NEXT or PREV and not yet sent back.
        self.unreturned_mm = 0.0
        self.findings: dict = {}
        # Image millimetres per commanded millimetre of offset, signed: what the
        # stepsize stage and each framing correction measured. offset_scale is
        # their median, None before any.
        self.scale_samples: list[float] = []
        self.scan_args = SimpleNamespace(png=False, tiff=False)

        inq = scanner.params.inq
        self.default_frame = tuple(scanner.params[n].value
                                   for n in ('tl_x', 'tl_y', 'br_x', 'br_y'))
        x0, y0, x1, y1 = (ex.native_to_mm(v, inq) for v in self.default_frame)
        self.window_mm = (x1 - x0, y1 - y0)
        cx, cy, half = (x0 + x1) / 2, (y0 + y1) / 2, PATCH_MM / 2
        self.patch_mm = (max(x0, cx - half), max(y0, cy - half),
                         min(x1, cx + half), min(y1, cy + half))
        steps = round(CENTRING_STEPS * self.window_mm[0] / CENTRING_WINDOW_MM)
        # offset_frame() truncates mm to steps; a hair over lands on `steps` exactly.
        self.centring_mm = round((steps + 0.01) / OFFSET_STEPS_PER_MM, 3)
        max_dpi = inq.max_resolution_x
        self.preview_dpi = min(PREVIEW_DPI, max_dpi)
        self.offset_dpi = min(OFFSET_DPI, max_dpi)
        self.patch_dpi = min(PATCH_DPI, max_dpi)

    # --- recording --------------------------------------------------------

    def event(self, **fields) -> None:
        fields['t'] = round(time.monotonic() - self.started, 2)
        self.record['events'].append(fields)

    def write_summary(self, **extra) -> None:
        summary = {'stages': self.stages, 'findings': self.findings,
                   'suggested_capabilities': self.suggested_capabilities(), **extra}
        (self.out / 'summary.json').write_text(json.dumps(summary, indent=1, default=str))

    def suggested_capabilities(self) -> dict:
        '''A DEVICE_CAPABILITIES entry from what the run measured; None where it
        measured nothing.'''
        f = self.findings
        return {
            'film_transport': f.get('frame_counter_moves'),
            'focus': f.get('focus_changes_sharpness'),
            'exposure_control': f.get('exposure_control'),
        }

    def run(self, plan: list[tuple[Stage, str | None]]) -> int:
        status = 0
        for stage, reason in plan:
            self.record = {'stage': stage.name, 'risk': stage.risk.name.lower(),
                           'status': 'running', 'events': []}
            self.stages.append(self.record)
            if reason is None and stage.needs_medium and not self.medium_present:
                reason = 'no film loaded according to READ STATE'
            if reason is not None:
                self.record['status'] = 'skipped'
                self.record['note'] = reason
                log.info(f"[suite] stage {stage.name} skipped: {reason}")
                continue
            print(f"\n{'=' * 70}\n== stage: {stage.name}\n{'=' * 70}")
            log.info(f"[suite] stage {stage.name}")
            stage_started = time.monotonic()
            try:
                getattr(self, stage.name)()
                self.record['status'] = 'ok'
            except Skip as e:
                self.record['status'] = 'skipped'
                self.record['note'] = str(e)
            except EndOfStrip as e:
                self.record['status'] = 'partial'
                self.record['note'] = f"end of strip: {e}"
                print(f"\nStage {stage.name} ran out of film: {e}")
            except Halt as e:
                self.record['status'] = 'halted'
                self.record['note'] = str(e)
                print(f"\nSTOPPING THE SUITE: {e}")
                status = 1
            except (PieusbError, ValueError, RuntimeError) as e:
                self.record['status'] = 'failed'
                self.record['note'] = f"{type(e).__name__}: {e}"
                log.exception(f"[suite] stage {stage.name} failed")
            finally:
                self.record['duration_s'] = round(time.monotonic() - stage_started, 1)
                self.write_summary()
            print(f"\n-- stage {stage.name}: {self.record['status']} "
                  f"in {self.record['duration_s']:.0f}s")
            if status:
                break
        return status

    # --- primitives -------------------------------------------------------

    def focus(self) -> int | None:
        try:
            return self.scanner.params['focus'].value
        except KeyError:
            return None

    def state(self, label: str = 'state') -> dict | None:
        try:
            s = self.scanner.transport_state()
        except PieusbError as e:
            self.event(op=label, error=f"{type(e).__name__}: {e}")
            return None
        if s is None:
            return None
        fields = dict(frame=s.frame, focus=s.focus, focus_max=s.focus_max,
                      flags=f"0x{s.flags:02x}", medium_present=s.medium_present,
                      raw=s.raw.hex(' '))
        self.event(op=label, **fields)
        return fields

    def scan(self, name: str):
        stage_dir = self.out / self.record['stage']
        n = sum(1 for e in self.record['events'] if e['op'] == 'scan') + 1
        prefix = stage_dir / f"{n:02d}_{name}"
        stage_dir.mkdir(parents=True, exist_ok=True)
        print(f"\nscan {prefix.name}: {self.scanner.params['resolution'].value} dpi, "
              f"focus {self.focus()}")
        status, r = ex.run_one_pass(self.scanner, self.scan_args, prefix)
        if r is not None and r.cancelled:
            raise KeyboardInterrupt
        if r is None or r.error is not None:
            raise RuntimeError(f"scan {name} did not complete")
        self.event(op='scan', name=name, file=f"{prefix.relative_to(self.out)}_rgb.npy",
                   width=r.width, height=r.height, resolution=r.resolution,
                   focus=self.focus(), shading_acquired=r.shading_acquired,
                   exposure_factor=r.exposure_factor, exposure=r.exposure, gain=r.gain,
                   mean=[round(float(r.rgb[:, :, c].mean()), 1) for c in range(3)],
                   duration_s=round(r.duration_s, 1))
        return r

    def move(self, op: str, value) -> tuple[dict | None, dict | None]:
        """One transport command, with READ STATE either side. Raises Halt on failure."""
        before = self.state('before')
        started = time.monotonic()
        print(f"\n{op} {value}...")
        try:
            if op == 'next':
                self.scanner.advance(value)
            elif op == 'prev':
                self.scanner._slide(SLIDE_PREV, value, byte3=self.prev_byte3)
            elif op == 'prev0':
                self.scanner._slide(SLIDE_PREV, value, byte3=0x00)
            elif op == 'prev1':
                self.scanner._slide(SLIDE_PREV, value, byte3=0x01)
            elif op == 'offset':
                self.scanner.offset_frame(value)
            else:
                raise ValueError(op)
        except (PieusbError, ValueError) as e:
            self.event(op=op, value=value, error=f"{type(e).__name__}: {e}")
            raise Halt(f"{op} {value} failed: {type(e).__name__}: {e}") from e
        busy = round(time.monotonic() - started, 2)
        if op == 'offset':
            self.unreturned_mm = round(self.unreturned_mm + value, 3)
        elif self.unreturned_mm:
            # Whether a frame move absorbs an offset is what 'redetect' measures,
            # so record what was outstanding rather than guess.
            self.event(op='unreturned_offset', mm=self.unreturned_mm, before=op)
        after = self.state('after')
        self.event(op=op, value=value, busy_s=busy,
                   frame_before=before and before['frame'],
                   frame_after=after and after['frame'])
        print(f"{op} {value}: {busy:.1f}s, frame "
              f"{before and before['frame']} -> {after and after['frame']}")
        if op != 'offset' and before and after and after['frame'] != before['frame']:
            self.findings['frame_counter_moves'] = True
        return before, after

    def next(self, frames: int = 1) -> None:
        before, after = self.move('next', frames)
        if before and after and after['frame'] == before['frame']:
            raise EndOfStrip(f"frame counter stayed at {after['frame']} after NEXT {frames}")

    def set(self, **options) -> None:
        for name, value in options.items():
            setattr(self.scanner, name, value)

    def unset(self, *names) -> None:
        '''Back to the option default, as if never assigned.'''
        for name in names:
            par = self.scanner.params[name]
            par.value = par.opt.default
            par.assigned = False

    def full_frame(self, dpi: int) -> None:
        ex.set_frame(self.scanner, *self.default_frame)
        self.set(resolution=dpi)

    # --- read-only and scan stages ----------------------------------------

    def identity(self) -> None:
        ex.describe(0, self.scanner.info)
        ex.report_device_settings(self.scanner)
        self.record['inquiry'] = {k: getattr(self.scanner.info.inquiry, k) for k in (
            'max_resolution_x', 'max_scan_w', 'max_scan_h', 'x0', 'y0', 'x1', 'y1',
            'minimum_exposure', 'maximum_exposure', 'firmware_version', 'model')}
        self.record['derived'] = {
            'window_mm': [round(v, 2) for v in self.window_mm],
            'patch_mm': [round(v, 2) for v in self.patch_mm],
            'centring_offset_mm': self.centring_mm,
        }
        if self.scanner.info.capabilities.film_transport:
            state = self.state()
            self.medium_present = bool(state and state['medium_present'])
            if state and state['frame'] != 0:
                print(f"\nNote: the frame counter reads {state['frame']}, not 0. "
                      f"Running from here.")

    def default_scan(self) -> None:
        '''One scan exactly as the library configures this model.'''
        self.full_frame(self.preview_dpi)
        self.scan('default')

    def exposure(self) -> None:
        '''Which device field moves the image: exp_time, exp_rel or gain.

        The compared passes skip the shading analysis and correct from one
        reference, since an analysing pass may re-derive its own exposure. One
        more, exp_rel doubled on an analysing pass against the calibrating one,
        tells whether the RELATIVE mode works (exposure_verdict).
        '''
        names = [f'{f}_{c}' for f in ('exp_time', 'exp_rel', 'gain') for c in 'rgb']
        self.full_frame(self.preview_dpi)
        self.set(reuse_calibration=False)
        calibrate = self.scan('calibrate')
        self.set(reuse_calibration=True)
        ratios = {}
        try:
            base = self.scan('base')
            exp_time = [min(2 * base.exposure[c], EXPOSURE_TIME_MAX) for c in range(3)]
            gain = [min(base.gain[c] + GAIN_STEP, GAIN_MAX) for c in range(3)]
            variants = (
                ('exp_time_x2', {f'exp_time_{s}': exp_time[c] for c, s in enumerate('rgb')}),
                ('exp_rel_x2', {f'exp_rel_{s}': 200 for s in 'rgb'}),
                (f'gain_plus{GAIN_STEP}', {f'gain_{s}': gain[c] for c, s in enumerate('rgb')}),
                ('base_again', {}),
            )
            for name, values in variants:
                self.set(**values)
                try:
                    r = self.scan(name)
                finally:
                    self.unset(*values)
                ratios[name] = level_ratio(base.rgb, r.rgb)
                print(f"{name}: level x {ratios[name]}")
            # Last, since an analysing pass replaces the cached reference.
            self.set(reuse_calibration=False, **{f'exp_rel_{s}': 200 for s in 'rgb'})
            r = self.scan('exp_rel_x2_analysed')
            ratios['exp_rel_x2_analysed'] = level_ratio(calibrate.rgb, r.rgb)
            print(f"exp_rel_x2_analysed: level x {ratios['exp_rel_x2_analysed']}")
        finally:
            self.unset(*names, 'reuse_calibration')
            self.record['ratios'] = ratios
        configured = self.scanner.info.capabilities.exposure_control.name
        verdict, works = exposure_verdict(ratios, configured)
        response = {name: ('doubles' if doubles(r) else 'live' if live(r) else 'no effect')
                    for name, r in ratios.items() if name != 'base_again'}
        self.record.update(verdict=verdict, works=works, response=response,
                           library_configured=configured,
                           matches_library=configured in works)
        self.findings['exposure_control'] = verdict
        print(f"\nexposure control: measured {verdict}, library configured {configured}"
              + ("" if verdict in (configured, 'undetermined') else "   <- MISMATCH"))

    # --- focus ------------------------------------------------------------

    def autofocus(self) -> None:
        limits = self.scanner.focus_range()
        if limits is None:
            raise Skip("the scanner reports no focus range")
        inq = self.scanner.params.inq
        ex.set_frame(self.scanner, *(ex.mm_to_native(v, inq) for v in self.patch_mm))
        self.set(resolution=self.patch_dpi)
        try:
            af = self.scanner.autofocus()
        except PieusbError as e:
            raise Halt(f"autofocus failed: {type(e).__name__}: {e}") from e
        path = self.out / self.record['stage'] / 'autofocus_lines.npy'
        path.parent.mkdir(parents=True, exist_ok=True)
        numpy.save(path, af.lines)
        self.event(op='autofocus', focus=af.focus, peaked=af.peaked,
                   scores=[round(s, 5) for s in af.scores],
                   file=str(path.relative_to(self.out)))
        print(f"\nautofocus: {'picked' if af.peaked else 'no clear peak, kept'} "
              f"focus {af.focus}")

        self.set(reuse_calibration=True)
        sharpness = {}
        try:
            for focus in focus_sweep(*limits):
                self.set(focus=focus)
                sharpness[focus] = round(image_sharpness(self.scan(f"focus{focus:02d}").rgb), 5)
        finally:
            self.unset('reuse_calibration')
            self.record['sweep_sharpness'] = sharpness
        best = max(sharpness, key=sharpness.get)
        values = list(sharpness.values())
        self.findings['focus_changes_sharpness'] = max(values) > FOCUS_EFFECT * min(values)
        # Later stages scan at the sweep's best: it is measured, the pick inferred.
        self.set(focus=best)
        self.record.update(sweep_best=best, autofocus_pick=af.focus if af.peaked else None,
                           focus_used_from_here=best)
        print(f"sweep sharpest at {best}; autofocus picked "
              f"{af.focus if af.peaked else 'nothing'}")

    # --- transport --------------------------------------------------------

    def offsets(self) -> None:
        self.full_frame(self.offset_dpi)
        for mm in (SMALL_OFFSET_MM, -SMALL_OFFSET_MM):
            self.scan('home')
            self.move('offset', mm)
            self.scan(f"offset{mm:+g}")
            self.move('offset', -mm)
        self.scan('home')
        self.full_frame(self.preview_dpi)
        for mm in (self.centring_mm, -self.centring_mm):
            self.scan('home')
            self.move('offset', mm)
            self.scan(f"offset{mm:+g}")
            self.move('offset', -mm)
        self.scan('home')

    def stepsize(self) -> None:
        '''What an offset step moves, measured in the image: the slope of image
        shift against commanded steps, out and back, and what each return misses.'''
        self.full_frame(self.preview_dpi)
        px_per_mm = self.preview_dpi / MM_PER_INCH
        home = self.scan('home')
        moves = []
        for steps in STEP_SIZES:
            mm = round((steps + 0.01) / OFFSET_STEPS_PER_MM, 3)
            self.move('offset', mm)
            out = self.scan(f"plus{steps}")
            self.move('offset', -mm)
            back = self.scan(f"back{steps}")
            shift = image_shift_px(home.rgb, out.rgb)
            missed = image_shift_px(home.rgb, back.rgb)
            moves.append({'steps': steps, 'commanded_mm': round(steps / OFFSET_STEPS_PER_MM, 3),
                          'image_px': shift, 'image_mm': round(shift / px_per_mm, 3),
                          'return_missed_mm': round(missed / px_per_mm, 3)})
            print(f"{steps} steps: {shift} px = {shift / px_per_mm:.2f} mm; "
                  f"return missed by {missed / px_per_mm:+.2f} mm")
            home = back
        x = numpy.array([m['commanded_mm'] for m in moves])
        y = numpy.array([m['image_mm'] for m in moves])
        slope, intercept = numpy.polyfit(x, y, 1)
        self.scale_samples.append(float(slope))
        self.record.update(moves=moves, image_mm_per_commanded_mm=round(slope, 4),
                           intercept_mm=round(intercept, 3),
                           mm_per_step=round(slope / OFFSET_STEPS_PER_MM, 5))
        self.findings['offset_mm_per_step'] = round(slope / OFFSET_STEPS_PER_MM, 5)
        print(f"\nan offset step moves the image {slope / OFFSET_STEPS_PER_MM:.4f} mm "
              f"(nominal {1 / OFFSET_STEPS_PER_MM:.4f}), "
              f"{'towards +x' if slope > 0 else 'towards -x'}")

    def stepping(self) -> None:
        self.full_frame(self.preview_dpi)
        self.scan('start')
        self.next(1)
        self.scan('next1')
        self.next(2)
        self.scan('next2')
        for op in ('prev1', 'prev0'):
            before, after = self.move(op, 1)
            self.scan(op)
            moved = before and after and after['frame'] == before['frame'] - 1
            self.record[f'{op}_moved_counter'] = bool(moved)
        # Rewinds later in the suite use whichever form moved the counter,
        # the library's own if both did.
        if not self.record['prev1_moved_counter'] and self.record['prev0_moved_counter']:
            self.prev_byte3 = 0x00
        self.record['prev_byte3_used_from_here'] = self.prev_byte3

    def redetect(self) -> None:
        self.full_frame(self.preview_dpi)
        self.scan('home')
        for shift in (REDETECT_OFFSET_MM, -REDETECT_OFFSET_MM):
            self.move('offset', shift)
            try:
                self.next(1)
            except EndOfStrip:
                self.move('offset', -shift)
                raise
            self.scan(f"after_offset{shift:+g}")
            self.move('prev', 1)
            self.move('offset', -shift)
            self.scan('restored')

    def to_first_frame(self) -> None:
        '''PREV until the frame counter reads 0.'''
        state = self.state()
        if state is None:
            raise RuntimeError("no READ STATE to rewind by")
        for _ in range(state['frame'] + 2):
            state = self.state()
            if state is None or state['frame'] == 0:
                break
            before, after = self.move('prev', 1)
            if before and after and after['frame'] == before['frame']:
                self.record['note'] = f"PREV stopped moving the counter at {after['frame']}"
                break

    def rewind(self) -> None:
        self.full_frame(self.preview_dpi)
        self.to_first_frame()
        self.scan('start_again')

    def walk(self) -> None:
        self.full_frame(self.preview_dpi)
        for _ in range(MAX_WALK_FRAMES):
            self.scan('home')
            self.move('offset', self.centring_mm)
            self.scan('plus')
            self.move('offset', -2 * self.centring_mm)
            self.scan('minus')
            self.move('offset', self.centring_mm)
            self.next(1)

    def recovery(self) -> None:
        self.full_frame(self.preview_dpi)
        self.state()
        self.move('prev', 1)
        self.scan('back_from_end')

    @property
    def offset_scale(self) -> float | None:
        return float(numpy.median(self.scale_samples)) if self.scale_samples else None

    def locate(self, r, frame_mm: float = FRAME_MM, label: str = 'located'):
        px_per_mm = r.resolution / MM_PER_INCH
        bands = find_bands(r.rgb, px_per_mm)
        frame = locate_frame(bands, r.rgb.shape[1], px_per_mm, frame_mm)
        self.event(op=label, **placement_fields(bands, frame))
        return frame

    def framing(self) -> None:
        '''Roll scanning's positioning loop, every frame from the first.

        Each frame: NEXT, a preview, the frame located, one offset by its shift
        from centre (converted with the measured offset scale, learnt from the
        first correction if stepsize did not run), and a check preview whose
        remaining shift is the loop's error. Every correction is also a sample
        of the offset scale, so one bad measurement cannot steer the rest.
        Corrections are not undone: NEXT carries them to the next frame. The frame width is learnt from frames
        whose edges both showed.
        '''
        self.full_frame(self.preview_dpi)
        self.to_first_frame()
        frame_mm = FRAME_MM
        widths, residuals = [], []
        self.record['frames'] = frames = []
        try:
            self.preview_timings()
            self.full_frame(self.preview_dpi)
            self.set(reuse_calibration=True)
            for index in range(MAX_WALK_FRAMES):
                if index:
                    self.next(1)
                entry = {'index': index}
                frames.append(entry)
                frame = self.locate(self.scan('preview'), frame_mm)
                if frame is None:
                    entry['result'] = 'no frame'
                    continue
                entry.update(shift_mm=round(frame.shift_mm, 3), measured=frame.measured,
                             confidence=round(frame.confidence, 2))
                if frame.measured_start and frame.measured_stop:
                    widths.append(frame.width_mm)
                    frame_mm = float(numpy.median(widths))
                if not frame.measured:
                    entry['result'] = 'no edge seen'
                    continue
                if abs(frame.shift_mm) < FRAMING_MIN_SHIFT_MM:
                    entry['result'] = 'centred'
                    continue
                steps = round(abs(frame.shift_mm / (self.offset_scale or 1.0))
                              * OFFSET_STEPS_PER_MM)
                if not 1 <= steps <= 0xFF:
                    entry['result'] = 'centred' if not steps else 'shift beyond one offset'
                    continue
                direction = 1 if frame.shift_mm * (self.offset_scale or 1.0) > 0 else -1
                # What the steps sent amount to; a hair over so offset_frame's
                # truncation lands on them exactly.
                commanded = direction * steps / OFFSET_STEPS_PER_MM
                self.move('offset', round(direction * (steps + 0.01) / OFFSET_STEPS_PER_MM, 3))
                after = self.locate(self.scan('check'), frame_mm, 'checked')
                entry.update(commanded_steps=direction * steps,
                             commanded_mm=round(commanded, 3))
                if after is None or not after.measured:
                    entry['result'] = 'lost after the move'
                    continue
                moved = frame.shift_mm - after.shift_mm
                entry.update(result='corrected', residual_mm=round(after.shift_mm, 3),
                             moved_mm=round(moved, 3))
                residuals.append(after.shift_mm)
                if abs(commanded) >= FRAMING_SCALE_MIN_MM:
                    self.scale_samples.append(moved / commanded)
                    entry['offset_scale_sample'] = round(moved / commanded, 4)
                print(f"frame {index}: shift {frame.shift_mm:+.2f} mm, "
                      f"left {after.shift_mm:+.2f} mm")
        finally:
            self.unset('reuse_calibration')
            self.record.update(frame_mm=round(frame_mm, 2),
                               frame_widths_mm=[round(w, 2) for w in widths],
                               offset_scale=self.offset_scale and round(self.offset_scale, 4))
            if residuals:
                r = numpy.abs(residuals)
                self.record['residual_mm'] = {'median': round(float(numpy.median(r)), 3),
                                              'max': round(float(r.max()), 3)}

    def preview_timings(self) -> None:
        '''One preview at each PREVIEW_TIMING_DPI on the current frame: how long it
        takes, and where it puts the frame. Each reuses the shading reference
        where the scanner allows, as the loop's previews do.'''
        timings = []
        self.set(reuse_calibration=True)
        try:
            for dpi in PREVIEW_TIMING_DPI:
                if dpi > self.scanner.info.inquiry.max_resolution_x:
                    continue
                self.full_frame(dpi)
                r = self.scan(f"timing{dpi}")
                frame = self.locate(r)
                timings.append({'dpi': dpi, 'duration_s': round(r.duration_s, 1),
                                'shading_acquired': r.shading_acquired,
                                'shift_mm': frame and round(frame.shift_mm, 3)})
        finally:
            self.unset('reuse_calibration')
            self.record['preview_timings'] = timings


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--output', metavar='DIR',
                   help='folder for the log, images and summary '
                        '(default: hwsuite-YYYYmmdd-HHMMSS)')
    p.add_argument('--profile', choices=tuple(PROFILES), default='characterize',
                   help='characterize: every stage the scanner has hardware for '
                        '(default). regression: read-only and scan stages only')
    p.add_argument('--allow', metavar='MOTOR[,MOTOR]', default='',
                   help=f"on a model the suite does not know, drive these motors: "
                        f"{', '.join(MOTORS)}. Refused on a known model that lacks them")
    p.add_argument('--device', type=int, default=0, metavar='N',
                   help='which enumerated device to use (default: 0)')
    p.add_argument('--only', metavar='STAGE[,STAGE...]',
                   help=f"run only these stages, in suite order: {', '.join(STAGE_NAMES)}")
    p.add_argument('--note', action='append', default=[], metavar='TEXT',
                   help='record TEXT in the log, e.g. what film is loaded; repeatable')
    p.add_argument('--no-ask', action='store_true',
                   help='do not ask the closing question')
    p.add_argument('--warmup-timeout', type=float, default=300, metavar='S',
                   help='how long to wait for the lamp (default: 300)')
    return p.parse_args(argv)


def parse_names(spec: str, valid, flag: str) -> frozenset:
    names = frozenset(s.strip() for s in spec.split(',') if s.strip())
    unknown = names - set(valid)
    if unknown:
        raise SystemExit(f"{flag}: unknown {', '.join(sorted(unknown))}; "
                         f"choose from {', '.join(valid)}")
    return names


def main(argv=None) -> int:
    args = parse_args(argv)
    allow = parse_names(args.allow, MOTORS, '--allow')
    only = parse_names(args.only, STAGE_NAMES, '--only') if args.only else None

    out = Path(args.output or datetime.now().strftime('hwsuite-%Y%m%d-%H%M%S'))
    out.mkdir(parents=True, exist_ok=True)
    ex.setup_logging(SimpleNamespace(verbose=False, log=str(out / 'suite.log'),
                                     note=args.note))
    code = 1
    try:
        code = run_suite(args, out, allow, only)
    except SystemExit as e:
        log.error(f"[run] {e}")
        raise
    except BaseException:
        # Logged here too: the zip is written before the interpreter prints the
        # traceback to stderr.
        log.exception("[run] hwsuite.py stopped on an exception")
        raise
    finally:
        log.info(f"[run] exit status {code}")
        zip_output(out)
    return code


def zip_output(out: Path) -> None:
    '''Pack the output folder as <folder>.zip beside it, the one file to send back.'''
    for handler in logging.getLogger().handlers:
        handler.flush()
    archive = Path(shutil.make_archive(str(out), 'zip', root_dir=out.resolve().parent,
                                       base_dir=out.resolve().name))
    print(f"\nSend back this one file: {archive.resolve()} "
          f"({archive.stat().st_size / 1e6:.0f} MB)")


def run_suite(args, out: Path, allow: frozenset, only) -> int:
    print("Enumerating...")
    devices = pieusb.get_devices()
    if not 0 <= args.device < len(devices):
        print(f"No scanner at --device {args.device} ({len(devices)} found).")
        return 1
    info = devices[args.device]
    model = info.inquiry.model
    motors = motors_for(model, allow)
    if args.profile == 'regression':
        # Regression runs the library exactly as configured for the model.
        motors = frozenset()
    plan = select_stages(args.profile, motors, only)
    log.info(f"[suite] {info.model} (model 0x{model:04x}), profile {args.profile}, "
             f"motors {', '.join(sorted(motors)) or 'none'}")
    for stage, reason in plan:
        log.info(f"[suite]   {stage.name:<13} {stage.risk.name.lower():<10} "
                 f"{'runs' if reason is None else 'skipped: ' + reason}")
    if motors:
        info = ex.assume_capabilities(info, ','.join(sorted(motors)))

    with Scanner(info) as scanner:
        scanner.mode = 'rgb'
        scanner.color_depth = 16
        print(f"\nWaiting for the scanner to be ready (up to {args.warmup_timeout:.0f}s)...")
        try:
            scanner.wait_ready(args.warmup_timeout)
        except (WarmingUp, DeviceNotReady) as e:
            print(f"Not ready: {e}")
            return 1
        suite = Suite(scanner, out)
        extra = dict(output=str(out), notes=args.note, profile=args.profile,
                     model=f"0x{model:04x}", model_name=info.model,
                     motors=sorted(motors))
        try:
            status = suite.run(plan)
        except KeyboardInterrupt:
            print("\nInterrupted.")
            if suite.record is not None:
                suite.record['status'] = 'interrupted'
            status = 130
        finally:
            suite.write_summary(**extra)

    print(f"\nSuggested capabilities: {suite.suggested_capabilities()}")
    if not args.no_ask:
        ex.ask_observation("Anything about the film or the scanner during the run worth "
                           "noting -- noises, the film coming out, frames skipped")
    return status


if __name__ == '__main__':
    sys.exit(main())
