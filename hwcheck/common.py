"""Shared helpers for the hardware check scripts.

These scripts probe protocol questions on a real MCA8000A (see docs/protocol.md §16).
They build commands and parse responses themselves, so a known bug in MCA8000A.py
doesn't hide or fake a result. The library is used for the low-level handshakes
(SendCommand, status prompts, baud rate setting).
"""

import argparse
import contextlib
import datetime
import io
import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import MCA8000A as lib  # noqa: E402

DEFAULT_PORT = os.environ.get('MCA_PORT', '/dev/cu.usbserial-A10KR3LP')
RESULTS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'results')

STATUS_SIZE = 20


class HWError(Exception):
    pass


@contextlib.contextmanager
def quiet():
    """Swallow the library's prints."""
    with contextlib.redirect_stdout(io.StringIO()):
        yield


# ---------------------------------------------------------------- arguments

def make_parser(description):
    p = argparse.ArgumentParser(description=description)
    p.add_argument('--port', default=DEFAULT_PORT,
                   help='serial device (default: $MCA_PORT or %(default)s)')
    p.add_argument('--baud', type=int, default=57600,
                   help='baud rate to switch the MCA to (default: %(default)s)')
    p.add_argument('--power-on', action='store_true',
                   help='run the remote power-on sequence before connecting')
    return p


# ---------------------------------------------------------------- commands

def with_checksum(b0, b1, b2, b3):
    return bytearray([b0, b1, b2, b3, (b0 + b1 + b2 + b3) % 256])


def block_command(code, start, nwords, layout='dll'):
    """Encode a 40h/42h/50h/52h block data command.

    layout='dll': DLL SendCommandDataEx / USB doc §3.11 example (believed correct):
        byte1 = S[5:0] << 2 | W[9:8], byte2 = S[13:6], byte3 = W[7:0]
    layout='doc': USB doc §3.4/§3.6 bit table:
        byte1 = S[5:0] << 2 | W[1:0], byte2 = S[13:6], byte3 = W[9:2]
    layout='py':  MCA8000A.Command_SendData as currently written:
        byte1 = S[13:8] << 2 | W[9:8], byte2 = S[7:0], byte3 = W[7:0]
    where S = start channel, W = nwords - 1.
    """
    assert 0 <= start <= 16383 and 1 <= nwords <= 1024
    w = nwords - 1
    if layout == 'dll':
        b1 = ((start & 0x3F) << 2) | (w >> 8)
        b2 = start >> 6
        b3 = w & 0xFF
    elif layout == 'doc':
        b1 = ((start & 0x3F) << 2) | (w & 0x3)
        b2 = start >> 6
        b3 = w >> 2
    elif layout == 'py':
        b1 = ((start >> 8) << 2) | (w >> 8)
        b2 = start & 0xFF
        b3 = w & 0xFF
    else:
        raise ValueError(layout)
    return with_checksum(code, b1, b2, b3)


# ---------------------------------------------------------------- status

def parse_status(b):
    """Parse a 20-byte status (docs/protocol.md §8). Doesn't touch the MCA object."""
    if len(b) < STATUS_SIZE:
        raise HWError('short status: {} bytes'.format(len(b)))
    b = bytes(b[:STATUS_SIZE])
    thr_word = int.from_bytes(b[16:18], 'big')
    flags = b[18]
    return {
        'checksum_ok': sum(b[:19]) % 256 == b[19],
        'datachk': int.from_bytes(b[0:4], 'big'),
        'serial': int.from_bytes(b[0:2], 'big'),
        'group': int.from_bytes(b[2:4], 'big') & 0x3FF,
        'preset': int.from_bytes(b[4:7], 'big'),
        'battery': b[7],
        'real': int.from_bytes(b[8:11], 'big') + 1 - b[11] / 75,
        'live': int.from_bytes(b[12:15], 'big') + 1 - b[15] / 75,
        'threshold': thr_word & 0x7FFF,
        'absolute_peak': bool(thr_word >> 15),
        'flags': flags,
        'resolution': 16384 >> (flags & 7),
        'live_timer': bool(flags & 0x08),
        'acquiring': bool(flags & 0x10),
        'protected': bool(flags & 0x20),
        'raw': b.hex(),
    }


# ---------------------------------------------------------------- connection

def collect(mca, idle=0.5, limit=10.0):
    """Read everything the MCA sends until `idle` seconds pass with no new bytes."""
    port = mca.serial_connection
    out = bytearray()
    t_end = time.monotonic() + limit
    t_idle = time.monotonic() + idle
    while time.monotonic() < t_end:
        n = port.in_waiting
        if n:
            out += port.read(n)
            t_idle = time.monotonic() + idle
        elif time.monotonic() > t_idle:
            break
        else:
            time.sleep(0.001)
    return out


def prompt_status(mca):
    """Prompt for status; return the parsed status, or None on failure."""
    with quiet():
        stat = mca.PromptForStatus()
    if stat:
        return None
    time.sleep(0.05)
    raw = collect(mca, idle=0.3)
    if len(raw) != STATUS_SIZE:
        return None
    st = parse_status(raw)
    if not st['checksum_ok']:
        return None
    # keep the library's cached state in sync for methods that rely on it
    with quiet():
        mca.UpdateStatusFromData(bytearray(raw), hasSerialNumber=True)
    return st


def prompt_status_retry(mca, tries=5):
    for _ in range(tries):
        st = prompt_status(mca)
        if st is not None:
            return st
        time.sleep(0.2)
    return None


def connect(args):
    """Open the port and get the MCA talking at args.baud. Returns (mca, status).

    Observed on hardware (2026-10-13): opening the port (pyserial raises DTR and RTS)
    puts the MCA back to 4800 baud, and a baud change only works when sent at 4800.
    So: open, check status at 4800, then switch.
    """
    with quiet():
        mca = lib.MCA8000A(args.port)
    if args.power_on:
        print('Powering on (takes ~4 s)...')
        with quiet():
            mca.PowerOn()
        print('  USB MCA detected:', mca.is_USB_MCA)
    mca.serial_connection.baudrate = 4800
    st = prompt_status_retry(mca, tries=3)
    if st is None:
        raise HWError('No valid status at 4800 baud. Is the MCA on? Try --power-on.')
    print('MCA S/N {} answering at 4800 baud.'.format(st['serial']))
    if args.baud != 4800:
        with quiet():
            stat = mca.SetBaudRate(args.baud)
        if stat:
            raise HWError('SetBaudRate failed')
        st = prompt_status_retry(mca)
        if st is None:
            raise HWError('No valid status after switching to {} baud'.format(args.baud))
        print('Switched to {} baud.'.format(args.baud))
    return mca, st


def reopen(mca):
    """Close and reopen the port. The MCA returns to 4800 baud (observed)."""
    mca.serial_connection.close()
    mca.serial_connection.open()
    mca.serial_connection.baudrate = 4800
    mca.RememberCTS()


def send(mca, command):
    """Send a 5-byte command with the library's USB handshake. Raises on failure."""
    with quiet():
        stat = mca.SendCommand(bytearray(command))
    if stat:
        raise HWError('SendCommand failed for {}'.format(bytes(command).hex()))


def block_read(mca, command, settle=0.05):
    """Send a block data command; return (status, data_bytes) of everything received."""
    send(mca, command)
    time.sleep(settle)
    raw = collect(mca)
    if len(raw) < STATUS_SIZE:
        raise HWError('got {} bytes, less than a status'.format(len(raw)))
    return parse_status(raw[:STATUS_SIZE]), bytes(raw[STATUS_SIZE:])


def words(data):
    """Little-endian unsigned 16-bit words."""
    return [int.from_bytes(data[i:i + 2], 'little') for i in range(0, len(data) - 1, 2)]


# ---------------------------------------------------------------- reporting

class Report:
    """Collects check results; prints them and saves JSON to hwcheck/results/."""

    def __init__(self, name, args):
        self.name = name
        self.data = {'check': name, 'port': args.port, 'baud': args.baud,
                     'started': datetime.datetime.now().isoformat(timespec='seconds'),
                     'results': []}

    def info(self, key, value):
        self.data[key] = value

    def result(self, label, verdict, detail=''):
        """verdict: PASS / FAIL / INFO / INCONCLUSIVE"""
        self.data['results'].append({'label': label, 'verdict': verdict, 'detail': detail})
        print('[{}] {}{}'.format(verdict, label, (': ' + detail) if detail else ''))

    def save(self):
        os.makedirs(RESULTS_DIR, exist_ok=True)
        stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
        path = os.path.join(RESULTS_DIR, '{}-{}.json'.format(self.name, stamp))
        with open(path, 'w') as f:
            json.dump(self.data, f, indent=1, default=str)
        print('Saved', os.path.relpath(path))
