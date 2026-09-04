#!/usr/bin/env python3
'''
Hardware exercise script for pieusb -- enumerate, configure, scan, save.

This is the manual test for the whole acquisition path. It needs a real scanner;
nothing here is a unit test.

    python tests/exercise.py --list                      # what is connected
    python tests/exercise.py --gain-offset               # what the device says its own
                                                         #   exposure/gain/offset/light are
    python tests/exercise.py --dry-run                   # resolve + validate options, touch nothing
    python tests/exercise.py                             # 300 dpi, device defaults, full frame
    python tests/exercise.py --mode rgbi --resolution 2000 --frame-mm 0 0 36 24
    python tests/exercise.py -o sharpen=true -o gain_r=25 --output run7

auto_exp meters a preview pass and sets exp_rel_* per channel from it -- the only
exposure control this hardware honours. It costs one extra pass:

    python tests/exercise.py -o auto_exp=true --output auto

Or set the relative exposures by hand. 247/563/1086 balances a colour negative on a
ProScan 10T, all three channels at ~88% of full scale:

    python tests/exercise.py -o exp_rel_r=247 -o exp_rel_g=563 -o exp_rel_b=1086 \
        --output manual

gain_*, exp_time_* and light have no measurable effect on that unit at any value.

Every scan logs the exposure, gain, offset and light it sent, at INFO.

Ctrl-C during a scan cancels it cleanly (STOP SCAN at the next chunk boundary).

Results are written as <prefix>_rgb.npy, shape (height, width, 3), plus
<prefix>_ir.npy for rgbi scans -- and as <prefix>_rgb.tif / <prefix>_ir.tif,
uncompressed TIFF at the scan's own bit depth, for anything that opens images.
Pixels are exactly as scanned: a negative stays a negative. --png also writes an
8-bit PNG if Pillow is installed.
'''

import argparse
import logging
import struct
import sys
import time
from pathlib import Path

# Run from a checkout without PYTHONPATH=src or an install.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src'))

import numpy

import pieusb
from pieusb.exceptions import DeviceNotReady, ParamError, PieusbError, WarmingUp
from pieusb.option import Unit
from pieusb.scanner import Scanner
from pieusb.types import (
    DeviceInfo,
    EjectDirection,
    ScanPhase,
    ScanResult,
    UpdateData,
)

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
    cap = info.capabilities
    print(f"     model number      : 0x{inq.model:04x}")
    print(f"     film transport    : {cap.film_transport}")
    print(f"     focus             : {cap.focus}")
    print(f"     exposure control  : {cap.exposure_control}")



def native_to_mm(value: int, inq) -> float:
    '''Frame coordinates are in native-resolution units, not scan dpi.

    Both axes convert with maximum_resolution / MM_PER_INCH, as in
    sanei_pieusb_set_frame_from_options (pieusb_specific.c:1791).
    '''
    return value / inq.max_resolution_x * MM_PER_INCH


def mm_to_native(value: float, inq) -> int:
    return round(value / MM_PER_INCH * inq.max_resolution_x)


def report_gain_offset(scanner: Scanner) -> None:
    '''Print the scanner's own calibration: GET GAIN OFFSET.

    These are the values auto_exp adopts. Zeros mean the firmware has not measured
    them yet, which it does during its first scan.
    '''
    s = scanner._get_gain_offset()
    print("\nScanner-reported calibration (GET GAIN OFFSET):")
    print(f"  saturation levels : {s['saturation_level']}  "
          f"({', '.join(f'{v * 100 / 65536:.0f}%' for v in s['saturation_level'])} of full scale)")
    print(f"  exposure times    : {s['exposure_time']} timer counts (R, G, B, I)")
    print(f"  gain              : {s['gain']}")
    print(f"  offset            : {s['offset']}")
    print(f"  light             : {s['light']}", end='')
    if s['light'] == 0:
        print("   <- 0 means the device did not report one; scans use the 'light' option")
    else:
        print(f"   <- lamp level; scan manually with -o light={s['light']} to match it")
    print("  (the lamp decrements from 7 as it warms and settles at 4; "
          "pieusb_scancmd.h:208-213)")


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

    Scanner validates each corner but can only say "invalid value"; a bad number
    here moves the carriage, so report the actual limit.
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


def report_transport_state(scanner: Scanner) -> int:
    """Print READ STATE and the focus range, or say why there is none."""
    try:
        state = scanner.transport_state()
    except PieusbError as e:
        print(f"\nCould not read the transport state: {type(e).__name__}: {e}")
        print("  (a warming-up scanner answers this with NOT READY; try again in "
              "a minute)")
        return 1
    if state is None:
        print(f"\n{scanner.info.model} reports no film transport.")
        return 0
    print("\nTransport state (READ STATE):")
    print(f"  raw           : {state.raw.hex(' ')}")
    print(f"  frame         : {state.frame}")
    print(f"  focus         : {state.focus}")
    print(f"  focus maximum : {state.focus_max}   <- depends on the loaded carrier")
    print(f"  medium present: {state.medium_present}")
    print(f"  carrier       : {'filmstrip' if state.filmstrip_carrier else 'slide'}")
    print(f"  flags         : 0x{state.flags:02x}   <- bits 3-5 unexplained")
    limits = scanner.focus_range()
    print(f"  focus range   : {limits if limits else 'no focus control'}")
    return 0


def report_frame(scanner: Scanner) -> None:
    '''Print the frame in both unit systems before anything is sent.

    An X/Y mix-up drives the carriage off its rail, hence printing the bounds.
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

    Runs on the worker thread, so it must not block: a slow callback stalls the
    read loop.
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
            # gets its own line and only the phase's last update is printed.
            print(header, end='' if self.tty else '\n', flush=True)
            self.line_open = self.tty

        # Phases with no reported length stay at 0.0; the header is all there is.
        if u.progress <= 0:
            return

        elapsed = now - self.phase_started
        eta = elapsed / u.progress - elapsed
        bar_w = 28
        filled = int(bar_w * u.progress)
        line = (f"[{now - self.started:7.1f}s] {u.phase} "
                f"[{'#' * filled}{'.' * (bar_w - filled)}] "
                f"{u.progress * 100:5.1f}%  ETA {eta:5.1f}s")
        if self.tty:
            print('\r' + line, end='', flush=True)
            self.line_open = True
        elif u.progress >= 1.0:
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

    save_tiffs(r, prefix)

    if args.png:
        save_png(r, prefix)
    return 0


def write_tiff(path: Path, image: numpy.ndarray) -> None:
    '''Write a baseline TIFF: uncompressed, little-endian, 8- or 16-bit, grey or RGB.

    Hand-rolled because the package has no image-library dependency, and because
    Pillow cannot write 16-bit RGB TIFF -- it drops the low byte of every sample.

    Pixels are written exactly as scanned: no inversion, gamma or level stretch.
    Same data as the .npy, in something a viewer will open.
    '''
    a = numpy.ascontiguousarray(image)
    if a.ndim == 2:
        a = a[:, :, numpy.newaxis]
    height, width, samples = a.shape
    if samples not in (1, 3):
        raise ValueError(f"can only write 1 or 3 samples per pixel, not {samples}")
    if a.dtype.kind != 'u' or a.dtype.itemsize not in (1, 2):
        raise ValueError(f"can only write uint8 or uint16 pixels, not {a.dtype}")
    a = a.astype(f'<u{a.dtype.itemsize}', copy=False)
    bits = a.dtype.itemsize * 8

    SHORT, LONG = 3, 4
    # (tag, type, count, value). A value of None is patched below with the offset
    # of its out-of-line data, which is what a value wider than 4 bytes carries.
    tags = [
        (256, SHORT, 1, width),                 # ImageWidth
        (257, SHORT, 1, height),                # ImageLength
        (258, SHORT, samples, None if samples > 2 else bits),  # BitsPerSample
        (259, SHORT, 1, 1),                     # Compression: none
        (262, SHORT, 1, 2 if samples == 3 else 1),  # Photometric: RGB / BlackIsZero
        (273, LONG, 1, None),                   # StripOffsets
        (277, SHORT, 1, samples),               # SamplesPerPixel
        (278, SHORT, 1, height),                # RowsPerStrip: the whole image
        (279, LONG, 1, a.nbytes),               # StripByteCounts
        (284, SHORT, 1, 1),                     # PlanarConfiguration: chunky
    ]

    header_size = 8
    ifd_size = 2 + 12 * len(tags) + 4
    # Out-of-line BitsPerSample, when there is one, then the pixels.
    extra = struct.pack(f'<{samples}H', *([bits] * samples)) if samples > 2 else b''
    bits_offset = header_size + ifd_size
    pixel_offset = bits_offset + len(extra)

    out = bytearray()
    out += struct.pack('<2sHI', b'II', 42, header_size)
    out += struct.pack('<H', len(tags))
    for tag, typ, count, value in tags:
        if value is None:
            value = bits_offset if tag == 258 else pixel_offset
        # A SHORT with count 1 sits in the low half of the 4-byte value field,
        # which in a little-endian file is what packing it as a LONG produces.
        out += struct.pack('<HHII', tag, typ, count, value)
    out += struct.pack('<I', 0)                 # no next IFD
    out += extra
    assert len(out) == pixel_offset, (len(out), pixel_offset)
    out += a.tobytes()

    path.write_bytes(bytes(out))


def save_tiffs(r: ScanResult, prefix: Path) -> None:
    '''Write the scan as viewable TIFFs alongside the .npy.'''
    rgb_path = prefix.with_name(prefix.name + '_rgb.tif')
    write_tiff(rgb_path, r.rgb)
    print(f"  saved {rgb_path}  ({r.rgb.dtype.itemsize * 8}-bit RGB, as scanned)")
    if r.ir is not None:
        ir_path = prefix.with_name(prefix.name + '_ir.tif')
        write_tiff(ir_path, r.ir)
        print(f"  saved {ir_path}  ({r.ir.dtype.itemsize * 8}-bit grey, as scanned)")


def save_png(r: ScanResult, prefix: Path) -> None:
    '''Optional 8-bit preview. Pillow is not a dependency, so this is
    best-effort.'''
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

def dump_inquiry(info: DeviceInfo) -> int:
    """Print the raw INQUIRY reply, for building a byte-exact test fixture."""
    from pieusb.transport import SCSI_INQUIRY, UASDevice
    with UASDevice(info.dev) as dev:
        header = dev.command(SCSI_INQUIRY, in_size=5, cdb_length=5)
        total = header[4] + 4
        raw = dev.command(SCSI_INQUIRY, in_size=total, cdb_length=total)
    print(f"\nINQUIRY, {len(raw)} bytes (additional_length {header[4]}):")
    for offset in range(0, len(raw), 16):
        print(f"  {offset:3d}: {raw[offset:offset + 16].hex(' ')}")
    return 0


def move_transport(scanner: Scanner, what: str, action) -> int:
    """Run one transport move and report the state it left behind."""
    print(f"\n{what}...")
    try:
        action()
    except (PieusbError, ValueError) as e:
        print(f"Could not {what.lower()}: {type(e).__name__}: {e}")
        return 1
    print("Done.")
    return report_transport_state(scanner)


def advance_slide(scanner: Scanner, frames: int = 1) -> int:
    return move_transport(scanner, f"Advancing {frames} frame(s)",
                          lambda: scanner.advance(frames))


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument('--inquiry-hex', action='store_true',
                   help='dump the raw INQUIRY reply as hex and exit')
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

    g = p.add_argument_group('slide transport (models with a film transport)')
    g.add_argument('--advance', action='store_true',
                   help='advance to the next slide once the scan has completed')
    g.add_argument('--advance-only', action='store_true',
                   help='advance to the next slide and exit, without scanning anything')
    g.add_argument('--frames', type=int, default=1, metavar='N',
                   help='how many frames --advance-only or --rewind moves (default 1)')
    g.add_argument('--rewind', action='store_true',
                   help='move back --frames frames and exit')
    g.add_argument('--eject', choices=('up', 'down'),
                   help='eject the medium and exit')
    g.add_argument('--offset-mm', type=float, metavar='MM',
                   help='shift the medium off its detected frame position and exit')

    g = p.add_argument_group('output')
    g.add_argument('--output', default='scan', metavar='PREFIX',
                   help='write PREFIX_rgb.npy / PREFIX_ir.npy (default: scan)')
    g.add_argument('--png', action='store_true',
                   help='also write an 8-bit PNG preview, if Pillow is installed')

    g = p.add_argument_group('focus (models with a focus motor)')
    g.add_argument('--transport-state', action='store_true',
                   help='print READ STATE: frame, focus, focus range, flags')
    g.add_argument('--focus', type=int, metavar='N',
                   help='move the focus motor to N and stop')

    g = p.add_argument_group('diagnostics')
    g.add_argument('--gain-offset', action='store_true',
                   help="read and print the scanner's own exposure/gain/offset/light and "
                        'saturation levels before doing anything else; combine with --dry-run '
                        'for the readout alone')
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
    # INFO rather than WARNING: a handful of lines per scan, including the
    # settings it sent. DEBUG is a line per SCSI command and stays behind -v.
    console.setLevel(logging.DEBUG if args.verbose else logging.INFO)
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

    if args.inquiry_hex:
        if not 0 <= args.device < len(devices):
            print(f"--device {args.device} out of range (0..{len(devices) - 1})")
            return 1
        return dump_inquiry(devices[args.device])

    if not 0 <= args.device < len(devices):
        print(f"--device {args.device} out of range (0..{len(devices) - 1})")
        return 1

    with Scanner(devices[args.device]) as scanner:
        if args.advance_only:
            return advance_slide(scanner, args.frames)

        if args.rewind:
            return move_transport(scanner, f"Rewinding {args.frames} frame(s)",
                                  lambda: scanner.rewind(args.frames))

        if args.eject is not None:
            return move_transport(scanner, f"Ejecting {args.eject}",
                                  lambda: scanner.eject(EjectDirection.UP
                                                        if args.eject == 'up'
                                                        else EjectDirection.DOWN))

        if args.offset_mm is not None:
            return move_transport(scanner, f"Offsetting by {args.offset_mm} mm",
                                  lambda: scanner.offset_frame(args.offset_mm))

        if args.transport_state:
            return report_transport_state(scanner)

        if args.focus is not None:
            try:
                scanner.set_focus(args.focus)
            except (PieusbError, ValueError) as e:
                print(f"Could not set the focus: {type(e).__name__}: {e}")
                return 1
            print(f"Focus moved to {args.focus}.")
            return report_transport_state(scanner)

        try:
            apply_options(scanner, args)
        except (TypeError, ValueError) as e:
            print(f"Bad option: {e}")
            return 1

        show_options(scanner)

        if args.gain_offset:
            try:
                report_gain_offset(scanner)
            except PieusbError as e:
                print(f"\nCould not read the scanner's calibration: "
                      f"{type(e).__name__}: {e}")
                print("  (a warming-up scanner may refuse this read; try again in a minute)")

        report_frame(scanner)

        try:
            # scan() validates too; doing it here lets --dry-run report the same
            # errors a real run would.
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

        # Short waits in a loop keep Ctrl-C responsive; join() does not deliver
        # KeyboardInterrupt.
        try:
            while not scanner.wait(0.2):
                pass
        except KeyboardInterrupt:
            print("\nCancelling -- the worker stops at the next chunk boundary...")
            scanner.cancel()
            scanner.wait()

        # wait() returned True, so the worker is done and on_complete has run.
        # Leaving this scope closes the device.
        result = box[0]
        status = report_result(result, args)

        if args.advance:
            if result.cancelled or result.error is not None:
                print("\nNot advancing: the scan did not complete.")
            elif advance_slide(scanner) != 0:
                return 1

        return status


if __name__ == '__main__':
    sys.exit(main())
