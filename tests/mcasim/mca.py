"""Simulated MCA8000A, USB protocol, for tier-1 tests.

Behaviour follows docs/protocol.md (section numbers below as §n). Where the documents and
the hardware disagree, the simulator does what the hardware was observed to do (🔬 in the
protocol doc, results from hwcheck/). Where nothing is known, the choice is marked
"assumed" and can usually be changed through a setting.

The simulator sits behind FakeSerial (port.py) and runs on a VirtualClock (clock.py).
Every port operation first brings the simulator up to the clock's current time
(`advance`), so the library sees the MCA react with realistic delays.

Model in brief:
- Control lines (§3, §6, §7). RTS rising: the MCA toggles CTS (toggle #1) and listens for
  a 5-byte command. A valid command: CTS toggles again (#2, "accepted", I-12) and the
  command runs. RTS falling: the MCA sends the command's response (data commands), or
  nothing (other accepted commands), or a 20-byte status (no command, or a rejected one,
  i.e. a status prompt). The answer starts `response_latency` after RTS falls and is
  dropped if RTS is high again by then (assumed; fits 🔬 2026-10-19: the library's prompt,
  RTS low for 0.2 ms then high, right after opening the port gets exactly one status).
- Checksum test is lenient by default: accepted if checksum byte >= true sum (I-23).
- Baud rate (§5): DTR + RTS both high resets the MCA to 4800 (assumed mechanism; it
  explains both observations of I-25: port open resets to 4800, and a baud command sent
  at another rate fails). Command 00h with DTR high sets the rate and starts the
  old-protocol data stream when RTS falls (I-24). RTS rising stops the stream; the
  stream's last bytes are held and sent ahead of the next response. At 115200 the MCA then
  answers no plain prompt until it has received a byte in SEND mode (I-24).
- Bytes sent at a rate the receiver isn't using arrive as 0xFF (framing garbage, count
  kept; assumed). Status checksums then fail.
- macOS driver bug (`timing.md`): purging the host RX buffer also discards host output
  that hasn't reached the MCA yet (`purge_drops_unsent`, on by default).
- Status (§8): DataChkSum carries the data checksum of the previous transfer in the status
  of the next 40h/42h command, else S/N + group. Any status clears the pending checksum
  (🔬 2026-10-19: after a prompt, the next 40h status reports 0).
- Memory (§9): 32768 channels of 32 bits, in groups of R = resolution channels. Data
  commands address the current group and run on into the following groups.
- Acquisition: real/live time advance with the clock; preset stops acquisition (§10).
  Counts are not simulated; tests write `memory` directly.
- Power: automatic power-down after `idle_powerdown` s on battery (§13, I-20), power-on by
  a DTR square wave with RTS high (§4).
"""

import heapq
import itertools

import numpy as np

TOTAL_CHANNELS = 32768
STATUS_SIZE = 20
BITS_PER_BYTE = 11          # start + 8 data + parity + stop, both directions
GARBLED = 0xFF

KNOWN_CODES = {0x00, 0x01, 0x02, 0x05, 0x10, 0x11, 0x19, 0x20, 0x25, 0x30,
               0x40, 0x42, 0x50, 0x52, 0x75}
DATA_CODES = {0x40, 0x42, 0x50, 0x52}

FLAG_LIVE = 0x08
FLAG_ACQUIRING = 0x10
FLAG_PROTECTED = 0x20
FLAG_NICAD = 0x40
FLAG_BACKUP_BAD = 0x80
HOST_FLAGS = 0x5F           # bits the CONTROL command writes (assumed: not 5 and 7)


def byte_time(baud):
    return BITS_PER_BYTE / baud


def checksum8(data):
    return sum(data) % 256


class Faults:
    """Fault injection. Each counter applies to the next n occurrences."""

    def __init__(self):
        self.drop_toggle1 = 0       # RTS rise not acknowledged (MCA "not ready")
        self.reject_commands = 0    # complete commands rejected (no toggle #2)
        self.silent_responses = 0   # nothing sent when RTS falls
        self.corrupt_status = 0     # status sent with a wrong checksum byte
        self.truncate_response = 0  # last byte of the response missing
        self.extra_bytes = 0        # one stray byte appended to the response
        self.corrupt_data = 0       # first data byte of a block transfer flipped (the
                                    # checksum reported later is that of the true data)
        self.corrupt_block = None   # same, for the block transfer with this index
                                    # (SimMCA.block_transfers counts them from 0)

    def take(self, name):
        n = getattr(self, name)
        if n > 0:
            setattr(self, name, n - 1)
            return True
        return False


class _Job:
    """Bytes the MCA transmits, one after another from `start` at `baud`."""

    def __init__(self, start, data, baud, stream=False):
        self.start = start
        self.data = bytes(data)
        self.baud = baud
        self.bt = byte_time(baud)
        self.pos = 0                # bytes already handed to the host
        self.stream = stream

    def end(self):
        return self.start + len(self.data) * self.bt

    def sent_by(self, t):
        """Number of bytes completely on the wire by MCA time t."""
        if t <= self.start:
            return 0
        return min(len(self.data), int((t - self.start) / self.bt + 1e-9))


class SimMCA:
    def __init__(self, clock, serial_number=1224, battery=0):
        self.clock = clock
        self.port = None            # the FakeSerial attached to this MCA

        # --- settings (seconds); change them in tests as needed
        self.line_latency = 0.0005      # host line change -> seen by the MCA
        self.usb_latency = 0.001        # host write -> first bit on the wire; MCA byte -> host
        self.cts_latency = 0.001        # RTS seen -> CTS toggle #1 seen by the host
        self.accept_latency = 0.002     # last command byte -> toggle #2 (1-3 ms, I-12)
        self.response_latency = 0.0005  # RTS low -> MCA starts sending (cancelled if RTS rises)
        self.open_duration = 0.02       # time a real port open takes (assumed)
        self.checksum_rule = 'lenient'  # or 'strict'
        self.data_checksum_modulus = 2**32   # I-11 open: 2**32 (DLL) or 2**16 (docs)
        self.purge_drops_unsent = True  # macOS driver bug
        self.stream_tail_bytes = 16     # stream bytes held back when RTS stops a stream
        self.mute_after_stream_baud = 115200  # I-24: rates at which a stopped stream mutes
        self.live_fraction = 1.0        # live time / real time while acquiring
        self.delete_busy = lambda R: R / 8000 + 0.05   # assumed; DLL waits R/8 ms + 256 ms
        self.start_delay = 0.97         # START -> counting; no answers meanwhile (S/N 1224)
        self.lock_zero_busy = 0.0       # assumed
        self.idle_powerdown = None      # s idle on battery before power-down (~180 real)
        self.floating_cts_after_powerdown = True   # I-20: one CTS change seen after power-down
        self.power_on_edges = 100       # DTR edges with RTS high that switch it on (assumed)
        self.power_on_delay = 1.0       # burst -> CTS toggle at power-up (assumed)

        self.faults = Faults()

        # --- MCA state
        self.serial_number = serial_number
        self.battery = battery          # status byte 7; 0 = external power
        self.powered = True
        self.baud = 4800
        self.flags = 0x04               # 1024 channels, real timer, stopped
        self.threshold_word = 0
        self.preset = 0
        self.group = 0
        self.real = [0.0] * 128         # per group
        self.live = [0.0] * 128
        self.stamp = [bytearray(8) for _ in range(128)]
        self.memory = np.zeros(TOTAL_CHANNELS, dtype=np.uint32)
        self.lock_code = 0
        self.data_checksum = None       # checksum of the last transfer, not yet reported
        self.block_transfers = 0        # block data responses sent so far

        # --- line / protocol state (as seen by the MCA)
        self.rts = False
        self.dtr = False
        self.cts = False                # as seen by the host (latency already applied)
        self.dsr = False                # unused by USB MCAs; never changes
        self.send_mode = False          # RTS high and acknowledged
        self.cmd = bytearray()
        self.cmd_state = 'none'         # none / accepted / rejected
        self.response = None            # callable building the response at RTS fall
        self.held = b''                 # stream tail held back (I-24)
        self.muted = False              # I-24 quirk at 115200
        self.busy_until = 0.0
        self.count_from = 0.0           # acquisition time counts from here
        self.last_comm = 0.0
        self.floating_cts = False
        self.dtr_edges = 0

        # --- transport
        self.t = 0.0
        self._events = []
        self._seq = itertools.count()
        self.jobs = []                  # MCA -> host, in order
        self.host_tx = []               # host -> MCA bytes in flight: [time, byte, baud, alive]
        self.host_tx_end = 0.0

        # --- record of what happened, for assertions
        self.log = []                   # (t, kind, detail)
        self.commands = []              # (t, bytes, accepted)

    # ------------------------------------------------------------ time

    @property
    def resolution(self):
        return 16384 >> (self.flags & 7)

    @property
    def acquiring(self):
        return bool(self.flags & FLAG_ACQUIRING)

    def _schedule(self, t, fn, *args):
        heapq.heappush(self._events, (t, next(self._seq), fn, args))

    def advance(self, t=None):
        """Run the MCA up to time t (default: the clock's current time)."""
        if t is None:
            t = self.clock.now
        while self._events and self._events[0][0] <= t:
            te, _, fn, args = heapq.heappop(self._events)
            self._flow(te)
            fn(*args)
        self._flow(t)

    def _flow(self, t):
        """Continuous processes up to t: acquisition time, power-down, output."""
        if t < self.t:
            return
        dt = t - self.t
        if self.powered and self.acquiring and t > self.count_from:
            self._acquire(t - max(self.t, self.count_from))
        self.t = t
        if (self.powered and self.idle_powerdown is not None and self.battery
                and not self.acquiring and t - self.last_comm > self.idle_powerdown):
            self._power_down()
        self._deliver(t)

    def _acquire(self, dt):
        g = self.group
        timer = self.live if self.flags & FLAG_LIVE else self.real
        rate = self.live_fraction if self.flags & FLAG_LIVE else 1.0
        if self.preset and rate > 0:
            dt_left = (self.preset - timer[g]) / rate
            if dt_left <= dt:
                self.real[g] += dt_left
                self.live[g] += dt_left * self.live_fraction
                self.flags &= ~FLAG_ACQUIRING
                self.log.append((self.t + dt_left, 'preset_stop', g))
                return
        self.real[g] += dt
        self.live[g] += dt * self.live_fraction

    # ------------------------------------------------------------ MCA -> host

    def _deliver(self, t):
        """Hand bytes that have reached the host by time t to the port."""
        while self.jobs:
            job = self.jobs[0]
            n = job.sent_by(t - self.usb_latency)
            if n > job.pos:
                chunk = job.data[job.pos:n]
                job.pos = n
                if self.port is not None:
                    self.port._receive(chunk, job.baud)
            if job.pos < len(job.data):
                break
            self.jobs.pop(0)

    def _transmit(self, data, stream=False):
        if not data:
            return
        start = self.t
        if self.jobs:
            start = max(start, self.jobs[-1].end())
        self.jobs.append(_Job(start, data, self.baud, stream))
        self.log.append((self.t, 'send', len(data)))

    def _stop_output(self):
        """RTS rose: stop transmitting. Returns True if a stream was cut off."""
        cut_stream = False
        held = b''
        for job in self.jobs:
            n = job.sent_by(self.t)
            rest = job.data[n:]
            if rest:
                held += rest
                cut_stream = cut_stream or job.stream
                job.data = job.data[:n]
        self.jobs = [j for j in self.jobs if j.pos < len(j.data)]
        if held:
            self.held += held[:self.stream_tail_bytes]
            self.log.append((self.t, 'stop_output', len(held)))
        return cut_stream

    # ------------------------------------------------------------ host -> MCA

    def host_lines(self, rts, dtr):
        """Called by the port when the host changes RTS/DTR (or opens/closes)."""
        self.advance()
        t = self.t + self.line_latency
        self._schedule(t, self._set_lines, rts, dtr)

    def host_write(self, data, host_baud):
        self.advance()
        bt = byte_time(host_baud)
        start = max(self.t, self.host_tx_end) + self.usb_latency
        for i, b in enumerate(data):
            entry = [start + (i + 1) * bt, b, host_baud, True]
            self.host_tx.append(entry)
            self._schedule(entry[0], self._rx_byte, entry)
        self.host_tx_end = start + len(data) * bt
        self.log.append((self.t, 'host_write', bytes(data).hex()))

    def host_purge_rx(self):
        self.advance()
        if self.purge_drops_unsent:
            self._drop_unsent('purge_rx')

    def host_purge_tx(self):
        self.advance()
        self._drop_unsent('purge_tx')

    def _drop_unsent(self, why):
        dropped = 0
        for entry in self.host_tx:
            if entry[3] and entry[0] > self.t:
                entry[3] = False
                dropped += 1
        if dropped:
            self.log.append((self.t, why + '_dropped_output', dropped))
        self.host_tx = [e for e in self.host_tx if e[3] and e[0] > self.t]
        if not self.host_tx:
            self.host_tx_end = self.t

    # ------------------------------------------------------------ line events

    def _set_lines(self, rts, dtr):
        if dtr != self.dtr:
            self.dtr = dtr
            self._on_dtr(dtr)
        if rts != self.rts:
            self.rts = rts
            self.log.append((self.t, 'rts', rts))
            if rts:
                self._on_rts_rise()
            else:
                self._on_rts_fall()
        # DTR + RTS high: baud-setting mode, which resets the rate (I-25)
        if self.powered and self.rts and self.dtr:
            self._baud_mode()

    def _toggle_cts(self, why, need_rts=False):
        if need_rts and not self.rts:
            return
        self.cts = not self.cts
        self.log.append((self.t, 'cts', why))

    def _on_dtr(self, dtr):
        if not self.powered:
            if self.rts:
                self.dtr_edges += 1
                if self.dtr_edges == self.power_on_edges:
                    self._schedule(self.t + self.power_on_delay, self._power_up)
            return

    def _baud_mode(self):
        if self.baud != 4800:
            self.log.append((self.t, 'baud', 4800))
        self.baud = 4800

    def _on_rts_rise(self):
        if not self.powered:
            self.dtr_edges = 0
            if self.floating_cts:
                self.floating_cts = False
                self._schedule(self.t + self.cts_latency, self._toggle_cts, 'floating')
            return
        self.last_comm = self.t
        if self._stop_output() and self.baud >= self.mute_after_stream_baud:
            self.muted = True
            self.log.append((self.t, 'muted', True))
        self.send_mode = True
        self.cmd = bytearray()
        self.cmd_state = 'none'
        self.response = None
        if self.faults.take('drop_toggle1'):
            self.log.append((self.t, 'fault', 'drop_toggle1'))
            return
        t = max(self.t, self.busy_until) + self.cts_latency
        self._schedule(t, self._toggle_cts, 'ready', True)

    def _on_rts_fall(self):
        self.send_mode = False
        if not self.powered:
            return
        if self.cmd_state == 'accepted':
            build = self.response
        else:
            build = self._prompt_status
        self.cmd_state = 'none'
        self.response = None
        self._schedule(self.t + self.response_latency, self._respond, build)

    def _respond(self, build):
        if self.rts or not self.powered:
            self.log.append((self.t, 'response_cancelled', None))
            return
        if self.muted:
            self.log.append((self.t, 'muted_no_answer', None))
            return
        if self.faults.take('silent_responses'):
            self.log.append((self.t, 'fault', 'silent_response'))
            return
        data = self.held + (build() if build else b'')
        self.held = b''
        if data and self.faults.take('truncate_response'):
            data = data[:-1]
        if data and self.faults.take('extra_bytes'):
            data = data + b'\x55'
        stream = build is not None and getattr(build, 'stream', False)
        self._transmit(data, stream=stream)

    def _rx_byte(self, entry):
        if not entry[3]:
            return
        entry[3] = False
        _, b, host_baud, _ = entry
        if not self.powered:
            return
        if host_baud != self.baud:
            b = GARBLED
        if not self.send_mode:
            self.log.append((self.t, 'ignored_byte', b))
            return
        self.last_comm = self.t
        if self.muted:
            self.muted = False
            self.log.append((self.t, 'muted', False))
        if len(self.cmd) >= 5:
            return                  # extra bytes after a command are ignored (I-12)
        self.cmd.append(b)
        if len(self.cmd) == 5:
            self._evaluate(bytes(self.cmd))

    # ------------------------------------------------------------ commands

    def _checksum_ok(self, cmd):
        s = checksum8(cmd[:4])
        if self.checksum_rule == 'strict':
            return cmd[4] == s
        return cmd[4] >= s

    def _evaluate(self, cmd):
        ok = cmd[0] in KNOWN_CODES and self._checksum_ok(cmd)
        if ok and self.faults.take('reject_commands'):
            ok = False
            self.log.append((self.t, 'fault', 'reject_command'))
        self.commands.append((self.t, cmd, ok))
        if not ok:
            self.cmd_state = 'rejected'
            return
        self.cmd_state = 'accepted'
        self._schedule(self.t + self.accept_latency, self._toggle_cts, 'accepted')
        self.response = self._execute(cmd)

    def _execute(self, cmd):
        """Run an accepted command. Returns a callable building the response, or None."""
        code, b1, b2, b3 = cmd[:4]
        if code in DATA_CODES:
            start = (b1 >> 2) | (b2 << 6)
            nwords = (((b1 & 3) << 8) | b3) + 1
            return self._block_response(code, start, nwords)
        if code in (0x00, 0x10):
            if code == 0x00 and self.dtr:
                if b3:
                    self.baud = 115200 // b3
                    self.log.append((self.t, 'baud', self.baud))
            addr = b1 | (b2 << 8)
            return self._stream_response(code, addr >> 2, bool(addr & 2))
        if code == 0x01:
            self._control(b1, b2 | (b3 << 8))
        elif code == 0x02:
            self.preset = b1 | (b2 << 8) | (b3 << 16)
        elif code == 0x05:
            self._delete(bool(b1), bool(b2))
        elif code == 0x11:
            if not self.acquiring and b2 < TOTAL_CHANNELS // self.resolution:
                self.group = b2
        elif code in (0x19, 0x20):
            if not self.acquiring:
                st = self.stamp[self.group]
                st[4], st[5], st[6], st[7] = b3, b2, b1, code
        elif code == 0x25:
            if not self.acquiring:
                st = self.stamp[self.group]
                st[0], st[1], st[2] = b3, b2, b1
        elif code == 0x30:
            return lambda: bytes(self.stamp[self.group])
        elif code == 0x75:
            self._lock(b1 | (b2 << 8))
        return None

    def _control(self, flags, threshold_word):
        was = self.acquiring
        self.flags = (self.flags & ~HOST_FLAGS & 0xFF) | (flags & HOST_FLAGS)
        # clamped to R/2 - 1 (seen at 2048 only, I-8; assumed for other R)
        limit = self.resolution // 2 - 1
        self.threshold_word = (threshold_word & 0x8000) | min(threshold_word & 0x7FFF, limit)
        if self.acquiring and not was:
            self._acquire(0.0)      # preset already reached: stops at once (assumed)
            # 🔬 then busy for ~1 s: no CTS toggle, and the times don't run yet
            self.busy_until = self.count_from = self.t + self.start_delay

    def _delete(self, data, time):
        R = self.resolution
        base = self.group * R
        if data:
            self.memory[base:base + R] = 0
        if time:
            self.real[self.group] = 0.0
            self.live[self.group] = 0.0
        self.busy_until = self.t + self.delete_busy(R)

    def _lock(self, n):
        # Simulated only; the real behaviour of lock 0 is unconfirmed (I-21).
        if n:
            self.lock_code = n
            self.flags |= FLAG_PROTECTED
        elif self.flags & FLAG_PROTECTED:
            self.flags &= ~FLAG_PROTECTED
            self.lock_code = 0
            self.memory[:] = 0
        self.busy_until = self.t + self.lock_zero_busy

    # ------------------------------------------------------------ responses

    def _words(self, start, nwords, upper):
        base = self.group * self.resolution
        idx = (base + start + np.arange(nwords)) % TOTAL_CHANNELS
        vals = self.memory[idx]
        w = (vals >> 16) if upper else (vals & 0xFFFF)
        return w.astype('<u2').tobytes()

    def _block_response(self, code, start, nwords):
        def build():
            if code in (0x40, 0x42):
                field = self.data_checksum or 0
            else:
                field = self._sn_field()
            self.data_checksum = None
            data = self._words(start, nwords, upper=bool(code & 2))
            self.data_checksum = sum(data) % self.data_checksum_modulus
            index = self.block_transfers
            self.block_transfers += 1
            if self.faults.take('corrupt_data') or index == self.faults.corrupt_block:
                data = bytes([data[0] ^ 0x01]) + data[1:]
            return self.status(field) + data
        return build

    def _stream_response(self, code, start, upper):
        """Old-protocol answer to 00h/10h: status, then words to the end of memory."""
        def build():
            field = self._sn_field() if code == 0x10 else (self.data_checksum or 0)
            self.data_checksum = None
            base = self.group * self.resolution
            n = TOTAL_CHANNELS - base - start
            return self.status(field) + self._words(start, max(n, 0), upper)
        build.stream = True
        return build

    def _prompt_status(self):
        self.data_checksum = None   # 🔬 a prompt clears the pending checksum
        return self.status(self._sn_field())

    def _sn_field(self):
        return (self.serial_number << 16) | (self.group & 0x3FF)

    @staticmethod
    def _time_bytes(t):
        """§8: whole seconds (24 bit) + a 75ths byte counting down within the second."""
        s = int(t)
        f = 75 - int((t - s) * 75)
        return s.to_bytes(3, 'big') + bytes([f])

    def status(self, field):
        g = self.group
        b = bytearray(field.to_bytes(4, 'big'))
        b += self.preset.to_bytes(3, 'big')
        b.append(self.battery)
        b += self._time_bytes(self.real[g])
        b += self._time_bytes(self.live[g])
        b += self.threshold_word.to_bytes(2, 'big')
        b.append(self.flags)
        b.append(checksum8(b))
        if self.faults.take('corrupt_status'):
            b[19] ^= 0xFF
        return bytes(b)

    # ------------------------------------------------------------ power

    def _power_down(self):
        self.powered = False
        self.jobs = []
        self.held = b''
        self.muted = False
        self.send_mode = False
        self.flags &= ~FLAG_ACQUIRING
        self.floating_cts = self.floating_cts_after_powerdown
        self.log.append((self.t, 'power', False))

    def _power_up(self):
        if self.powered:
            return
        self.powered = True
        self.baud = 4800
        self.last_comm = self.t
        self.dtr_edges = 0
        self._toggle_cts('power_on')       # USB MCAs toggle CTS, not DSR (§4)
        self.log.append((self.t, 'power', True))

    def power_down_now(self):
        """Test helper: power down as if idle on battery."""
        self.advance()
        self._power_down()
