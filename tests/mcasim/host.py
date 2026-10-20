"""A minimal host that drives the MCA by hand, following docs/protocol.md.

Independent of MCA8000A.py, so the simulator can be tested without trusting the library.
The same host drives a real MCA in tier 2 (`Host.on_port`), where it is the oracle the
library's results are checked against, and runs the contract scenarios (tests/contract.py)
on both.
"""

from .clock import RealClock
from .mca import BITS_PER_BYTE, STATUS_SIZE
from .port import FakeSerial


def with_checksum(*b):
    return bytes(b) + bytes([sum(b) % 256])


def block_command(code, start, nwords):
    """40h/42h/50h/52h with the DLL bit layout (§9)."""
    w = nwords - 1
    return with_checksum(code, ((start & 0x3F) << 2) | (w >> 8), start >> 6, w & 0xFF)


def parse_status(b):
    b = bytes(b)
    flags = b[18]
    word = int.from_bytes(b[16:18], 'big')
    return {
        'checksum_ok': sum(b[:19]) % 256 == b[19],
        'field': int.from_bytes(b[0:4], 'big'),
        'serial': int.from_bytes(b[0:2], 'big'),
        'group': int.from_bytes(b[2:4], 'big') & 0x3FF,
        'preset': int.from_bytes(b[4:7], 'big'),
        'battery': b[7],
        'real': int.from_bytes(b[8:11], 'big') + 1 - b[11] / 75,
        'live': int.from_bytes(b[12:15], 'big') + 1 - b[15] / 75,
        'threshold': word & 0x7FFF,
        'absolute_peak': bool(word >> 15),
        'flags': flags,
        'resolution': 16384 >> (flags & 7),
        'acquiring': bool(flags & 0x10),
        'protected': bool(flags & 0x20),
    }


class Host:
    idle = 0.05             # collect(): quiet time that ends a response

    def __init__(self, sim, baud=4800):
        self.sim = sim
        self.clock = sim.clock
        self.port = FakeSerial(sim, '/dev/sim', baudrate=baud, parity='E', timeout=0)
        self._settle()

    @classmethod
    def on_port(cls, port, clock=None, sim=None):
        """A host on an open port, at the port's current baud rate: a real port, or
        (with `sim`) the FakeSerial the library uses, to share it with the library."""
        self = cls.__new__(cls)
        self.sim = sim
        self.clock = clock or (sim.clock if sim else RealClock())
        self.port = port
        if sim is None:
            self.idle = 0.2     # FTDI latency timer and USB scheduling
        self._settle()
        return self

    def _settle(self):
        # Opening raised DTR and RTS. Drop both and drain whatever the MCA sends
        # in answer (a status, when RTS falls).
        self.port.dtr = False
        self.port.rts = False
        self.collect()

    def wait(self, dt):
        self.clock.sleep(dt)
        if self.sim is not None:
            self.sim.advance()

    def wait_flip(self, ref, timeout=0.2):
        end = self.clock.now + timeout
        while self.clock.now < end:
            if self.port.cts != ref:
                return True
            self.wait(50e-6)
        return False

    def rts_low(self, settle=0.001):
        self.port.rts = False
        self.wait(settle)

    def prompt(self):
        """RTS pulse without a command. True if CTS toggled."""
        self.rts_low()
        ref = self.port.cts
        self.port.rts = True
        ok = self.wait_flip(ref)
        self.port.reset_input_buffer()
        self.port.rts = False
        return ok

    def command(self, cmd, purge_after_write=None):
        """Send a command with the USB handshake. Returns (toggle1, toggle2).

        purge_after_write: None = don't purge between write and toggle #2;
        a number = purge RX that many seconds after the write.
        """
        self.rts_low()
        ref = self.port.cts
        self.port.rts = True
        t1 = self.wait_flip(ref)
        if not t1:
            self.port.rts = False
            return False, False
        ref = self.port.cts
        self.port.write(cmd)
        if purge_after_write is not None:
            self.wait(purge_after_write)
            self.port.reset_input_buffer()
        t2 = self.wait_flip(ref)
        self.port.reset_input_buffer()
        self.port.rts = False
        return True, t2

    def collect(self, idle=None, limit=30.0):
        """Read until `idle` s pass with no new bytes."""
        idle = self.idle if idle is None else idle
        out = bytearray()
        end = self.clock.now + limit
        quiet_until = self.clock.now + idle
        while self.clock.now < end:
            n = self.port.in_waiting
            if n:
                out += self.port.read(n)
                quiet_until = self.clock.now + idle
            elif self.clock.now > quiet_until:
                break
            self.wait(0.0005)
        return bytes(out)

    def status(self):
        """Prompt and return the raw bytes received (normally the 20-byte status)."""
        if not self.prompt():
            return b''
        return self.collect()

    def set_baud(self, baud):
        """§5: 00h with DTR high, switch the host, then end the stream as EndDataStream."""
        self.port.dtr = False
        self.rts_low(0.1)
        self.port.dtr = True
        ok = self.command(with_checksum(0, 0, 0, 115200 // baud))
        self.port.dtr = False
        self.port.baudrate = baud
        return ok

    def connect(self, baud):
        """set_baud, then end the stream with a 1-word data request (as EndDataStream)."""
        ok = self.set_baud(baud) if baud != 4800 else (True, True)
        if baud != 4800:
            self.command(block_command(0x40, 0, 1))
            self.wait(0.3)
            self.port.reset_input_buffer()
        return ok

    def read_block(self, code, start, nwords):
        """One block transfer. Returns (status dict, data bytes); None if no answer."""
        t1, t2 = self.command(block_command(code, start, nwords))
        if not t2:
            return None
        raw = self.collect()
        if len(raw) < STATUS_SIZE:
            return None
        return parse_status(raw[:STATUS_SIZE]), raw[STATUS_SIZE:]

    def wire_time(self, nbytes):
        return nbytes * BITS_PER_BYTE / self.port.baudrate


__all__ = ['Host', 'with_checksum', 'block_command', 'parse_status', 'STATUS_SIZE']
