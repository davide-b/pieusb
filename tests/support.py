'''Reference loaders and a recording stand-in for UASDevice. Asserts nothing.'''

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'src'))

from pieusb import transport
from pieusb.exceptions import CheckCondition, PieusbError, TransportError

# Transcript names, resolved through the library's constants. Listed explicitly:
# transport.py's subcommand constants share values with opcodes (SCSI_SCAN_FRAME
# and SCSI_INQUIRY are both 0x12), so a dir() sweep would let one shadow the other.
_OPCODE_NAMES = (
    'TEST_UNIT_READY', 'REQUEST_SENSE', 'READ', 'WRITE', 'PARAM', 'INQUIRY',
    'MODE_SELECT', 'COPY', 'MODE_SENSE', 'SCAN', 'READ_GAIN_OFFSET',
    'WRITE_GAIN_OFFSET', 'READ_STATE', 'SLIDE',
)
_SUBCMD_NAMES = (
    'SCAN_FRAME', 'EXPOSURE', 'HIGHLIGHT_SHADOW', 'CALIBRATION_INFO',
)
_OPCODES = {getattr(transport, 'SCSI_' + n): n for n in _OPCODE_NAMES}
_SUBCMDS = {getattr(transport, 'SCSI_' + n): n for n in _SUBCMD_NAMES}


def describe(opcode, out=b''):
    name = _OPCODES.get(opcode, 'OP_%02X' % opcode)
    if opcode == transport.SCSI_WRITE and len(out) >= 2:
        code = out[0] | (out[1] << 8)
        name += ':' + _SUBCMDS.get(code, 'sub_%04X' % code)
    return name

REFERENCE_DIR = Path(__file__).resolve().parent / 'reference'
GOLDEN_DIR = Path(__file__).resolve().parent / 'golden'
CAPTURE_DIR = ROOT / 'captures'


# --- reference corpora -------------------------------------------------------

class Reference:
    '''One transaction from a reference log: what was sent, what came back.'''

    def __init__(self, call, cdb, out=b'', data=b'', in_len=None):
        self.call = call
        self.cdb = cdb
        self.out = out
        self.data = data
        # True reply length: exceeds len(data) where SANE's dump clipped, and set
        # with data empty for bulk image reads, whose payload is not kept.
        self.in_len = len(data) if in_len is None else in_len

    @property
    def clipped(self):
        return self.in_len - len(self.data)

    opcode = property(lambda s: s.cdb[0] if s.cdb else None)

    @property
    def cdb_len(self):
        return (self.cdb[3] << 8) | self.cdb[4] if len(self.cdb) >= 5 else None

    @property
    def sub(self):
        '''The 16-bit subcommand code of a WRITE (0x0A) parameter list.'''
        if self.opcode == 0x0A and len(self.out) >= 2:
            return self.out[0] | (self.out[1] << 8)
        return None

    def __repr__(self):
        return '<%s cdb=%s out=%d in=%d>' % (
            self.call, self.cdb.hex(' '), len(self.out), len(self.data))


def load_sane_reference(name='proscan10t_sane.txt'):
    '''Parse the checked-in SANE debug extract into Reference objects.'''
    records = []
    cur = {}
    for line in (REFERENCE_DIR / name).read_text().splitlines():
        line = line.rstrip()
        if line.startswith('#'):
            continue
        if not line:
            if cur:
                records.append(_record(cur))
                cur = {}
            continue
        key, _, value = line.partition(' ')
        cur[key] = value
    if cur:
        records.append(_record(cur))
    return records


def _record(fields):
    data = bytes.fromhex(fields.get('in', '').replace(' ', ''))
    in_len = fields.get('in_len')
    return Reference(
        fields.get('call', '?'),
        bytes.fromhex(fields.get('cdb', '').replace(' ', '')),
        bytes.fromhex(fields.get('out', '').replace(' ', '')),
        data,
        len(data) if in_len is None else int(in_len),
    )


def load_inquiry_hex(name):
    """A raw INQUIRY reply from a hex file under tests/reference/."""
    text = (REFERENCE_DIR / name).read_text()
    body = ''.join(line for line in text.splitlines()
                   if not line.startswith('#'))
    return bytes.fromhex(body.replace(' ', ''))


def only(records, opcode=None, sub=None, index=0):
    '''The index-th record matching opcode and/or subcommand.'''
    hits = [r for r in records
            if (opcode is None or r.opcode == opcode)
            and (sub is None or r.sub == sub)]
    if not hits:
        raise LookupError('no record with opcode=%r sub=%r' % (opcode, sub))
    return hits[index]


def load_capture(relative_path):
    '''Transactions from a usbmon capture, or None if `captures/` is absent.'''
    path = CAPTURE_DIR / relative_path
    if not path.exists():
        return None
    sys.path.insert(0, str(CAPTURE_DIR / 'tools'))
    from txns import scsi
    return scsi(str(path))


# --- fake device -------------------------------------------------------------

class FakeDevice:
    '''Stands in for UASDevice, recording every command instead of sending it.

    `reject` maps (opcode, cdb_length) to a CheckCondition to raise, scripting a
    device that refuses something. `replies` maps an opcode to the bytes to
    return.
    '''

    def __init__(self, reject=None, replies=None):
        self.sent = []
        self.reject = reject or {}
        self.replies = replies or {}
        # Mirrors UASDevice: option.write_gain_offset() caches here.
        self.gain_offset_length = None

    def command(self, opcode, out_data=b'', in_size=0, cdb_length=None):
        if cdb_length is None:
            cdb_length = len(out_data) if out_data else in_size
        self.sent.append({
            'opcode': opcode,
            'cdb_length': cdb_length,
            'out': bytes(out_data),
            'in_size': in_size,
        })
        error = self.reject.get((opcode, cdb_length))
        if error is not None:
            raise error
        return self.replies.get(opcode, b'')

    # -- views over what was recorded

    def of(self, opcode, sub=None):
        '''Every recorded command with this opcode, optionally by subcommand.'''
        out = []
        for c in self.sent:
            if c['opcode'] != opcode:
                continue
            if sub is not None:
                if len(c['out']) < 2 or (c['out'][0] | (c['out'][1] << 8)) != sub:
                    continue
            out.append(c)
        return out

    def one(self, opcode, sub=None):
        hits = self.of(opcode, sub)
        assert len(hits) == 1, (
            'expected exactly one 0x%02x%s, got %d'
            % (opcode, '' if sub is None else '/0x%04x' % sub, len(hits)))
        return hits[0]

    def transcript(self):
        '''A stable, readable rendering, for golden-master comparison.'''
        lines = []
        for c in self.sent:
            line = '%-22s cdb_len=%-5d' % (describe(c['opcode'], c['out']),
                                           c['cdb_length'])
            if c['out']:
                line += ' out=' + c['out'].hex(' ')
            elif c['in_size']:
                line += ' in_size=%d' % c['in_size']
            lines.append(line.rstrip())
        return '\n'.join(lines) + '\n'


class FakeStrip(FakeDevice):
    '''A FakeDevice holding a strip of `frames` frames, as a ProScan 4000 does.

    READ STATE reports the frame counter in byte 2 of `state`. NEXT and PREV
    move it, stopping at the last frame and at frame 0, as the hardware does.
    '''

    def __init__(self, frames, state, frame=0, **kwargs):
        super().__init__(**kwargs)
        self.frames = frames
        self.frame = frame
        self.state = bytearray(state)

    def command(self, opcode, out_data=b'', in_size=0, cdb_length=None):
        reply = super().command(opcode, out_data, in_size, cdb_length)
        if opcode == transport.SCSI_SLIDE:
            action, value = out_data[0], out_data[1]
            if action == transport.SLIDE_NEXT:
                self.frame = min(self.frame + value, self.frames - 1)
            elif action == transport.SLIDE_PREV:
                self.frame = max(self.frame - value, 0)
        elif opcode == transport.SCSI_READ_STATE:
            self.state[2] = self.frame
            return bytes(self.state)
        return reply
