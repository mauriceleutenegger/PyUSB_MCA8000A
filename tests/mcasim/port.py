"""FakeSerial: the part of pyserial's Serial that MCA8000A.py uses, wired to a SimMCA.

Host-side behaviour mimics pyserial on macOS with an FT232R (`timing.md`):
- opening the port raises DTR and RTS, at least briefly (this resets the MCA to 4800 baud);
- `out_waiting` always reads 0 and `flush()` returns at once;
- `reset_input_buffer()` also discards output not yet sent (SimMCA.purge_drops_unsent).
"""

import types

import serial


class FakeSerial:
    def __init__(self, sim, port=None, baudrate=9600, bytesize=8, parity='N',
                 stopbits=1, timeout=None, write_timeout=None, **kwargs):
        self.sim = sim
        sim.port = self
        self.port = port
        self.baudrate = baudrate
        self.bytesize = bytesize
        self.parity = parity
        self.stopbits = stopbits
        self.timeout = timeout
        self.write_timeout = write_timeout
        self.is_open = False
        self._rts = True
        self._dtr = True
        self._rx = bytearray()
        self.trace = []             # (t, operation, detail), host side
        if port is not None:
            self.open()

    # ------------------------------------------------------------ helpers

    def _now(self):
        self.sim.advance()
        return self.sim.t

    def _check_open(self):
        if not self.is_open:
            raise serial.PortNotOpenError()

    def _receive(self, chunk, baud):
        """Called by the simulator when bytes reach the host."""
        if not self.is_open:
            return
        if baud != self.baudrate:
            chunk = bytes([0xFF]) * len(chunk)
        self._rx += chunk

    def _lines_changed(self):
        if self.is_open:
            self.sim.host_lines(self._rts, self._dtr)

    # ------------------------------------------------------------ open / close

    def open(self):
        if self.is_open:
            raise serial.SerialException('Port is already open.')
        self.is_open = True
        self._rx = bytearray()
        self.trace.append((self._now(), 'open', None))
        # The OS raises DTR and RTS when the device is opened; pyserial then applies
        # its stored states (True unless the program set them otherwise).
        self.sim.host_lines(True, True)
        self.sim.clock.sleep(self.sim.open_duration)
        if not (self._rts and self._dtr):
            self._lines_changed()

    def close(self):
        if not self.is_open:
            return
        self.trace.append((self._now(), 'close', None))
        self.sim.host_lines(False, False)   # HUPCL drops the lines (assumed)
        self.is_open = False
        self._rx = bytearray()

    # ------------------------------------------------------------ control lines

    @property
    def rts(self):
        return self._rts

    @rts.setter
    def rts(self, value):
        value = bool(value)
        self.trace.append((self._now(), 'rts', value))
        self._rts = value
        self._lines_changed()

    @property
    def dtr(self):
        return self._dtr

    @dtr.setter
    def dtr(self, value):
        value = bool(value)
        self.trace.append((self._now(), 'dtr', value))
        self._dtr = value
        self._lines_changed()

    @property
    def cts(self):
        self._check_open()
        self._now()
        return self.sim.cts

    @property
    def dsr(self):
        self._check_open()
        self._now()
        return self.sim.dsr

    # ------------------------------------------------------------ data

    @property
    def in_waiting(self):
        self._check_open()
        self._now()
        return len(self._rx)

    @property
    def out_waiting(self):
        return 0                    # macOS FTDI drivers: always 0 (timing.md)

    def read(self, size=1):
        self._check_open()
        self._now()
        if len(self._rx) < size and self.timeout:
            deadline = self.sim.t + self.timeout
            while len(self._rx) < size and self.sim.t < deadline:
                self.sim.clock.sleep(0.0005)
                self._now()
        data = bytes(self._rx[:size])
        del self._rx[:size]
        self.trace.append((self.sim.t, 'read', data.hex()))
        return data

    def write(self, data):
        self._check_open()
        data = bytes(data)
        self.trace.append((self._now(), 'write', data.hex()))
        self.sim.host_write(data, self.baudrate)
        return len(data)

    def flush(self):
        self._check_open()           # returns at once (timing.md)

    def reset_input_buffer(self):
        self._check_open()
        self.trace.append((self._now(), 'purge_rx', len(self._rx)))
        self._rx = bytearray()
        self.sim.host_purge_rx()

    def reset_output_buffer(self):
        self._check_open()
        self.trace.append((self._now(), 'purge_tx', None))
        self.sim.host_purge_tx()


def serial_module(sim):
    """A stand-in for the `serial` module whose Serial() opens a FakeSerial on `sim`.

    Use with monkeypatch.setattr(MCA8000A, 'serial', serial_module(sim)) to test
    the library opening the port itself.
    """
    return types.SimpleNamespace(
        Serial=lambda *args, **kwargs: FakeSerial(sim, *args, **kwargs),
        SerialException=serial.SerialException,
        PortNotOpenError=serial.PortNotOpenError,
    )
