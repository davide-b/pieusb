#!/usr/bin/env python3
'''
Hardware exercise script for pieusb -- enumerate, configure, scan, save.

This is the manual test for the whole acquisition path. It needs a real scanner;
nothing here is a unit test.

    python tests/exercise.py --list                      # what is connected
    python tests/exercise.py --dry-run                   # resolve + validate options, touch nothing
    python tests/exercise.py                             # 300 dpi, device defaults, full frame
    python tests/exercise.py --mode rgbi --resolution 2000 --frame-mm 0 0 36 24
    python tests/exercise.py -o sharpen=true -o gain_r=25 --output run7

Ctrl-C during a scan cancels it cleanly (STOP SCAN at the next chunk boundary)
rather than leaving the device mid-acquisition.

Results are written as .npy: <prefix>_rgb.npy with shape (height, width, 3) and,
for rgbi scans, <prefix>_ir.npy with shape (height, width). --png also writes an
8-bit preview if Pillow happens to be installed.
'''

import argparse
import logging
import sys
import time
from pathlib import Path

# The package is not installed in .venv yet (TODO 27), so make `python
# tests/exercise.py` work from a checkout without PYTHONPATH=src.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src'))

import numpy

import pieusb
from pieusb.exceptions import DeviceNotReady, ParamError, PieusbError, WarmingUp
from pieusb.option import Unit
from pieusb.scanner import Scanner  # not exported from __init__ yet (TODO 18)
from pieusb.types import DeviceInfo, ScanPhase, ScanResult, UpdateData

MM_PER_INCH = 25.4

log = logging.getLogger('exercise')


# ---------------------------------------------------------------------------
# Device reporting
# ---------------------------------------------------------------------------

def describe(index: int, info: DeviceInfo) -> None:
    inq = info.inquiry
    print(f"[{index}] {info.vendor} {info.model}  (rev {inq.revision})")
    print(f"     native resolution : {inq.max_resolution_x} x {inq.max_resolution_y} dpi")
    print(f"     bed               : {inq.max_scan_w} x {inq.max_scan_h} native units "
          f"= {native_to_mm(inq.max_scan_w, inq):.1f} x {native_to_mm(inq.max_scan_h, inq):.1f} mm")
    print(f"     filters           : {', '.join(inq.filters)}")
    print(f"     colour depths     : {', '.join(str(d) for d in inq.color_depths)}")
    print(f"     colour formats    : {', '.join(inq.color_formats)}")
    print(f"     image formats     : {', '.join(inq.image_formats)}")
    print(f"     capabilities      : {', '.join(inq.scan_capabilities) or '-'}")
    print(f"     optional devices  : {', '.join(inq.optional_devices) or '-'}")
    print(f"     slide transport   : {inq.slide_transport}")
    print(f"     exposure range    : {inq.minimum_exposure} .. {inq.maximum_exposure}")
    print(f"     production        : {inq.production} {inq.timestamp}")


def native_to_mm(value: int, inq) -> float:
    '''Frame coordinates are in native-resolution units, not scan dpi.

    Confirmed on hardware (poc:648-658) and matching the C backend, which
    converts with a single `maximum_resolution / MM_PER_INCH` for both axes
    (sanei_pieusb_set_frame_from_options, pieusb_specific.c:1791).
    '''
    return value / inq.max_resolution_x * MM_PER_INCH


def mm_to_native(value: float, inq) -> int:
    return round(value / MM_PER_INCH * inq.max_resolution_x)


def show_options(scanner: Scanner) -> None:
    print("\nOption table:")
    for par in scanner.params.table:
        unit = '' if par.opt.unit is Unit.NONE else f" {par.opt.unit.name.lower()}"
        changed = '*' if par.value != par.opt.default else ' '
        print(f"  {changed} {par.opt.name:<14} = {par.value!r:<8}{unit:<14}"
              f"(default {par.opt.default!r})")
    print("  (* = changed from the default)")


# ---------------------------------------------------------------------------
# Option parsing
# ---------------------------------------------------------------------------

def coerce(name: str, raw: str, target: type):
    if target is bool:
        low = raw.strip().lower()
        if low in ('1', 'true', 'yes', 'on'):
            return True
        if low in ('0', 'false', 'no', 'off'):
            return False
        raise SystemExit(f"option {name}: {raw!r} is not a boolean "
                         f"(use true/false, yes/no, on/off, 1/0)")
    try:
        return target(raw)
    except ValueError:
        raise SystemExit(f"option {name}: {raw!r} is not a valid {target.__name__}")


def set_frame(scanner: Scanner, x0: int, y0: int, x1: int, y1: int) -> None:
    '''Assign the four frame corners, reporting a bed overrun in both unit systems.

    Scanner.__setitem__ validates each corner but can only say "invalid value";
    for the one option where a bad number moves the carriage, the caller deserves
    the actual limit.
    '''
    inq = scanner.params.inq
    for name, value, limit in (('tl_x', x0, inq.max_scan_w), ('tl_y', y0, inq.max_scan_h),
                               ('br_x', x1, inq.max_scan_w), ('br_y', y1, inq.max_scan_h)):
        try:
            setattr(scanner, name, value)
        except ValueError:
            raise SystemExit(
                f"{name} = {value} native units ({native_to_mm(value, inq):.2f} mm) is outside "
                f"the scanner's bed of 0..{limit} ({native_to_mm(limit, inq):.2f} mm). "
                f"Note X is the long axis and Y the short one -- check they are not swapped."
            )


def apply_options(scanner: Scanner, args) -> None:
    inq = scanner.params.inq

    if args.mode is not None:
        scanner.mode = args.mode
    if args.resolution is not None:
        scanner.resolution = args.resolution
    if args.depth is not None:
        scanner.color_depth = args.depth

    if args.frame_mm is not None:
        set_frame(scanner, *(mm_to_native(v, inq) for v in args.frame_mm))
    elif args.frame is not None:
        set_frame(scanner, *args.frame)

    # -o NAME=VALUE for everything else, typed from the option table itself.
    for assignment in args.option:
        name, sep, raw = assignment.partition('=')
        if not sep:
            raise SystemExit(f"--option expects NAME=VALUE, got {assignment!r}")
        name = name.strip()
        try:
            par = scanner.params[name]
        except KeyError:
            names = ', '.join(p.opt.name for p in scanner.params.table)
            raise SystemExit(f"no such option {name!r}. Available: {names}")
        setattr(scanner, name, coerce(name, raw, par.opt.type))


def report_frame(scanner: Scanner) -> None:
    '''Print the frame in both unit systems before anything is sent.

    An X/Y mix-up here once drove the carriage off its rail, which is why
    OptionsTable.validate() re-checks the bounds and why this prints them.
    '''
    inq = scanner.params.inq
    x0, y0 = scanner.params['tl_x'].value, scanner.params['tl_y'].value
    x1, y1 = scanner.params['br_x'].value, scanner.params['br_y'].value
    print(f"\nFrame : ({x0}, {y0}) - ({x1}, {y1}) native units"
          f"  =  ({native_to_mm(x0, inq):.2f}, {native_to_mm(y0, inq):.2f}) - "
          f"({native_to_mm(x1, inq):.2f}, {native_to_mm(y1, inq):.2f}) mm")
    print(f"Bed   : (0, 0) - ({inq.max_scan_w}, {inq.max_scan_h}) native units"
          f"  =  {native_to_mm(inq.max_scan_w, inq):.2f} x "
          f"{native_to_mm(inq.max_scan_h, inq):.2f} mm")
    print(f"Scan  : {scanner.params['mode'].value}, "
          f"{scanner.params['color_depth'].value}-bit, "
          f"{scanner.params['resolution'].value} dpi")


# ---------------------------------------------------------------------------
# Progress reporting
# ---------------------------------------------------------------------------

class Progress:
    '''Renders UpdateData as a single rewritten terminal line.

    Both callbacks run on the worker thread, so this must not block -- a slow
    callback stalls the read loop.
    '''

    def __init__(self) -> None:
        self.started = time.monotonic()
        self.phase: ScanPhase | None = None
        self.phase_started = self.started
        self.tty = sys.stdout.isatty()
        self.line_open = False

    def __call__(self, u: UpdateData) -> None:
        now = time.monotonic()
        if u.phase is not self.phase:
            self._finish_line()
            self.phase = u.phase
            self.phase_started = now
            header = f"[{now - self.started:7.1f}s] {u.phase}"
            # Without a TTY the bar cannot be rewritten in place, so the header
            # gets its own line and only the final count is reported.
            keep_open = self.tty and u.scanned_lines is not None
            print(header, end='' if keep_open else '\n', flush=True)
            self.line_open = keep_open

        if u.scanned_lines is None or not u.total_lines:
            return

        frac = u.scanned_lines / u.total_lines
        elapsed = now - self.phase_started
        eta = (elapsed / frac - elapsed) if frac > 0 else 0
        bar_w = 28
        filled = int(bar_w * frac)
        line = (f"[{now - self.started:7.1f}s] {u.phase} "
                f"[{'#' * filled}{'.' * (bar_w - filled)}] "
                f"{u.scanned_lines}/{u.total_lines} lines "
                f"{frac * 100:5.1f}%  ETA {eta:5.1f}s")
        if self.tty:
            print('\r' + line, end='', flush=True)
            self.line_open = True
        elif u.scanned_lines == u.total_lines:
            print(line, flush=True)

    def _finish_line(self) -> None:
        if self.line_open:
            print()
            self.line_open = False


# ---------------------------------------------------------------------------
# Result reporting
# ---------------------------------------------------------------------------

def summarize_plane(name: str, plane: numpy.ndarray) -> None:
    full = numpy.iinfo(plane.dtype).max
    zero = int((plane == 0).sum())
    sat = int((plane == full).sum())
    total = plane.size
    print(f"    {name}: min={plane.min():>6} max={plane.max():>6} "
          f"mean={plane.mean():>9.1f}  "
          f"zero={zero * 100 / total:5.2f}%  saturated={sat * 100 / total:5.2f}%")


def report_result(r: ScanResult, args) -> int:
    print()
    if r.error is not None:
        print(f"FAILED after {r.duration_s:.1f}s: {type(r.error).__name__}: {r.error}")
        log.debug("scan failure", exc_info=r.error)
        return 1
    if r.cancelled:
        print(f"CANCELLED after {r.duration_s:.1f}s -- no image (partial data is discarded)")
        return 130

    print(f"OK in {r.duration_s:.1f}s: {r.width} x {r.height}, mode {r.mode}, "
          f"{r.color_depth}-bit, {r.resolution} dpi, "
          f"shading {'corrected' if r.shading_corrected else 'NOT corrected'}")
    print(f"  rgb {r.rgb.shape} {r.rgb.dtype}"
          + (f", ir {r.ir.shape} {r.ir.dtype}" if r.ir is not None else ", no ir plane"))

    for idx, name in enumerate(('R', 'G', 'B')):
        summarize_plane(name, r.rgb[:, :, idx])
    if r.ir is not None:
        summarize_plane('I', r.ir)

    if r.rgb.max() == 0:
        print("  WARNING: the image is entirely black -- check lamp, exposure and gain")

    prefix = Path(args.output)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    rgb_path = prefix.with_name(prefix.name + '_rgb.npy')
    numpy.save(rgb_path, r.rgb)
    print(f"  saved {rgb_path}")
    if r.ir is not None:
        ir_path = prefix.with_name(prefix.name + '_ir.npy')
        numpy.save(ir_path, r.ir)
        print(f"  saved {ir_path}")

    if args.png:
        save_png(r, prefix)
    return 0


def save_png(r: ScanResult, prefix: Path) -> None:
    '''Optional eyeball check. Pillow is not a dependency -- DESIGN.md returns
    numpy only -- so this is best-effort and 8-bit.'''
    try:
        from PIL import Image
    except ImportError:
        print("  --png: Pillow is not installed, skipping "
              "(pip install pillow, or load the .npy in numpy)")
        return
    rgb = r.rgb
    if rgb.dtype == numpy.uint16:
        rgb = (rgb >> 8).astype(numpy.uint8)
        note = " (16-bit downshifted to 8 for preview)"
    else:
        note = ""
    png_path = prefix.with_name(prefix.name + '_rgb.png')
    Image.fromarray(rgb, mode='RGB').save(png_path)
    print(f"  saved {png_path}{note}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument('--list', action='store_true',
                   help='enumerate scanners, print their INQUIRY, and exit')
    p.add_argument('--device', type=int, default=0, metavar='N',
                   help='which enumerated device to use (default: 0)')
    p.add_argument('--dry-run', action='store_true',
                   help='resolve and validate the options, print them, and stop before scanning')

    g = p.add_argument_group('scan settings (unset options keep the device default)')
    g.add_argument('--mode', choices=('gray', 'rgb', 'rgbi'))
    g.add_argument('--resolution', type=int, metavar='DPI')
    g.add_argument('--depth', type=int, choices=(8, 16), metavar='BITS')
    g.add_argument('--frame-mm', type=float, nargs=4, metavar=('X0', 'Y0', 'X1', 'Y1'),
                   help='scan frame in millimetres')
    g.add_argument('--frame', type=int, nargs=4, metavar=('X0', 'Y0', 'X1', 'Y1'),
                   help='scan frame in native-resolution units (see --list)')
    g.add_argument('-o', '--option', action='append', default=[], metavar='NAME=VALUE',
                   help='set any other option from the table; repeatable')

    g = p.add_argument_group('output')
    g.add_argument('--output', default='scan', metavar='PREFIX',
                   help='write PREFIX_rgb.npy / PREFIX_ir.npy (default: scan)')
    g.add_argument('--png', action='store_true',
                   help='also write an 8-bit PNG preview, if Pillow is installed')

    g = p.add_argument_group('diagnostics')
    g.add_argument('-v', '--verbose', action='store_true',
                   help='DEBUG logging to stderr -- one line per SCSI command, very chatty')
    g.add_argument('--log', metavar='FILE',
                   help='write a DEBUG log to FILE (independent of -v)')
    g.add_argument('--warmup-timeout', type=float, default=180, metavar='S',
                   help='how long to wait for the lamp before giving up (default: 180)')
    return p.parse_args(argv)


def setup_logging(args) -> None:
    root = logging.getLogger()
    root.setLevel(logging.DEBUG if (args.verbose or args.log) else logging.INFO)

    console = logging.StreamHandler(sys.stderr)
    console.setLevel(logging.DEBUG if args.verbose else logging.WARNING)
    console.setFormatter(logging.Formatter('%(levelname)s %(name)s: %(message)s'))
    root.addHandler(console)

    if args.log:
        handler = logging.FileHandler(args.log, mode='w')
        handler.setLevel(logging.DEBUG)
        handler.setFormatter(logging.Formatter(
            '%(asctime)s %(levelname)s %(name)s: %(message)s'))
        root.addHandler(handler)
        print(f"debug log -> {args.log}")


def main(argv=None) -> int:
    args = parse_args(argv)
    setup_logging(args)

    print("Enumerating...")
    devices = pieusb.get_devices()
    if not devices:
        print("No supported scanner found. Check the USB connection and that you have "
              "permission to claim the interface (udev rule, or run as root).")
        return 1

    for i, info in enumerate(devices):
        describe(i, info)

    if args.list:
        return 0

    if not 0 <= args.device < len(devices):
        print(f"--device {args.device} out of range (0..{len(devices) - 1})")
        return 1

    with Scanner(devices[args.device]) as scanner:
        try:
            apply_options(scanner, args)
        except (TypeError, ValueError) as e:
            print(f"Bad option: {e}")
            return 1

        show_options(scanner)
        report_frame(scanner)

        try:
            # scan() validates too; doing it here means --dry-run reports the
            # same errors a real run would, before the device is touched.
            scanner.params.validate()
        except ParamError as e:
            print(f"\nInvalid option combination: {e}")
            return 1

        if args.dry_run:
            print("\n--dry-run: options are valid, stopping without scanning.")
            return 0

        print(f"\nWaiting for the scanner to be ready (up to {args.warmup_timeout:.0f}s)...")
        try:
            scanner.wait_ready(args.warmup_timeout)
        except WarmingUp as e:
            print(f"Still warming up: {e}")
            return 1
        except DeviceNotReady as e:
            print(f"Not ready: {e}")
            return 1

        box: list[ScanResult] = []

        print("Scanning. Ctrl-C cancels.\n")
        try:
            scanner.scan(Progress(), box.append)
        except PieusbError as e:
            print(f"Could not start the scan: {type(e).__name__}: {e}")
            return 1

        # Short waits in a loop rather than one open-ended wait(), so Ctrl-C
        # stays responsive; join() itself does not deliver KeyboardInterrupt.
        try:
            while not scanner.wait(0.2):
                pass
        except KeyboardInterrupt:
            print("\nCancelling -- the worker stops at the next chunk boundary...")
            scanner.cancel()
            scanner.wait()

        # wait() returned True, so the worker thread is dead and on_complete has
        # already run. Leaving this scope closes the device (Scanner.close()).
        return report_result(box[0], args)


if __name__ == '__main__':
    sys.exit(main())
