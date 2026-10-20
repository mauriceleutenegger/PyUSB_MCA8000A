"""State-changing scenarios: the library changes MCA settings, the hand-driven host checks.

The same scenarios run on the simulator (tier 1, unit/test_state_sim.py) and on the real
MCA (tier 2, hw/test_state_hw.py), like the read-only ones in contract.py. Each takes
(mca, host, scratch): the library connected at 115200, the host on the same port, and the
Scratch the test works in.

Safety on hardware:
- Everything happens in a scratch group: the highest group at the original resolution
  (as hwcheck/check_preset.py). The MCA must be stopped, unprotected, not in absolute-peak
  mode (I-13), and not already in that group; otherwise the tests skip.
- `Scratch.restore` puts back flags (resolution, timer), threshold, group and preset and
  checks them. The hw fixture runs it on a freshly opened port, even after a failure.
- `hw_state` scenarios change settings only: no acquisition, no deletion.
  `hw_delete` scenarios also acquire in the scratch group and delete its data and time.
- A state session also compares all memory outside the scratch group, and the original
  group's times, at its start and end (hw/conftest.py).

The library's prompt fails once after someone else's prompt (I-15), so scenarios refresh
its cached status with `sync` rather than a plain ReceiveStatusFromPrompt.
"""

import numpy as np
import pytest

import MCA8000A
from mcasim.host import STATUS_SIZE, parse_status, with_checksum

TOTAL_CHANNELS = 32768
ACQUIRE = 0x10
LIVE_TIMER = 0x08
TICK = 1 / 75
# after START the MCA is busy ~1 s (no status) and only then starts counting; STOP takes
# effect at once (S/N 1224, 2026-10-19, hwcheck/results/hw-state-20261019/probe_start_lag)
START_LAG = 0.97
I14 = 'I-14: CONTROL resends stale cached flags'

# settings Scratch.restore puts back (and the session check compares)
SETTINGS = ('serial', 'group', 'preset', 'threshold', 'absolute_peak', 'flags')


# ---------------------------------------------------------------- raw commands (host)

def fresh(host, tries=3):
    """A valid status from a host prompt."""
    raw = b''
    for _ in range(tries):
        raw = host.status()
        if len(raw) == STATUS_SIZE:
            st = parse_status(raw)
            if st['checksum_ok']:
                return st
        host.wait(0.2)
    raise AssertionError('no valid status: ' + raw.hex())


def send(host, *b):
    t1, t2 = host.command(with_checksum(*b))
    host.collect()
    assert t2, 'command not accepted: ' + bytes(b).hex()


def threshold_word(st):
    return st['threshold'] | (st['absolute_peak'] << 15)


def control(host, flags, word):
    send(host, 0x01, flags & 0xFF, word & 0xFF, word >> 8)


def start(host, st):
    control(host, st['flags'] | ACQUIRE, threshold_word(st))


def stop(host, st):
    control(host, st['flags'] & ~ACQUIRE, threshold_word(st))


def set_group(host, group):
    send(host, 0x11, 0, group, 1)


def set_preset(host, preset):
    send(host, 0x02, preset & 0xFF, (preset >> 8) & 0xFF, preset >> 16)


def delete(host, data, time, resolution):
    send(host, 0x05, int(data), int(time), 1)
    host.wait(resolution / 8000 + 0.256)      # DLL ClearDataAndTime


def read_words(host, start, nwords):
    """Channels start .. start + nwords - 1 of the current group (and the following groups,
    §9) as 32-bit values, in blocks of up to 1024 words."""
    out = []
    for code in (0x40, 0x42):
        data = bytearray()
        for s in range(start, start + nwords, 1024):
            n = min(1024, start + nwords - s)
            got = host.read_block(code, s, n)
            assert got is not None, 'no answer to block {:02x} at {}'.format(code, s)
            st, block = got
            assert st['checksum_ok'] and len(block) == 2 * n
            data += block
        out.append(np.frombuffer(bytes(data), dtype='<u2').astype(np.uint32))
    return out[0] | (out[1] << 16)


def snapshot(host):
    """All 32768 channels: read from group 0 and from the group starting at channel 16384
    (data requests reach 16384 channels past the group start, §9). Restores the group.
    The MCA must be stopped (SET GROUP is ignored while acquiring)."""
    st = fresh(host)
    assert not st['acquiring'], 'snapshot needs the MCA stopped'
    parts = []
    for group in (0, 16384 // st['resolution']):
        set_group(host, group)
        assert fresh(host)['group'] == group
        parts.append(read_words(host, 0, 16384))
    set_group(host, st['group'])
    assert fresh(host)['group'] == st['group']
    return np.concatenate(parts)


def changed_ranges(a, b, skip=()):
    """Channel ranges where a and b differ, outside the slices in `skip`."""
    diff = a != b
    for sl in skip:
        diff[sl] = False
    idx = np.flatnonzero(diff)
    if not len(idx):
        return []
    breaks = np.flatnonzero(np.diff(idx) > 1)
    starts = np.concatenate([[idx[0]], idx[breaks + 1]])
    ends = np.concatenate([idx[breaks], [idx[-1]]])
    return [(int(s), int(e)) for s, e in zip(starts, ends)]


def run_for(host, seconds):
    """Acquire for `seconds` with preset 0 (run until stopped), by raw commands."""
    set_preset(host, 0)
    start(host, fresh(host))
    host.wait(seconds)
    stop(host, fresh(host))
    st = fresh(host)
    assert not st['acquiring']
    return st


# ---------------------------------------------------------------- the library side

def sync(mca):
    """Refresh the library's cached status. Its first prompt after another program's
    prompt fails (I-15); the next one works."""
    if mca.ReceiveStatusFromPrompt():
        assert mca.ReceiveStatusFromPrompt() == 0


# ---------------------------------------------------------------- scratch group

class Scratch:
    def __init__(self, orig):
        self.orig = orig
        self.R = orig['resolution']
        self.group = TOTAL_CHANNELS // self.R - 1
        self.channels = slice(self.group * self.R, TOTAL_CHANNELS)
        self.ordinary = self.R - self.R // 32     # below the top 1/32 (§9)

    def allowed_changes(self, any_memory=False):
        """Memory a state session may change: the scratch group, and the last 4 channels
        (the group's parameters, I-27) of the groups it visits with SET GROUP: the
        original group and the snapshot's groups 0 and 16384 / R. With `any_memory`
        (the resolution test), also those of the scratch group's number at each lower
        resolution, where a resolution change puts it."""
        params = {((g + 1) * self.R) for g in (self.orig['group'], 0, 16384 // self.R)}
        if any_memory:
            R = 256
            while R < self.R:
                params.add((self.group + 1) * R)
                R *= 2
        return [self.channels] + [slice(end - 4, end) for end in sorted(params)]

    @staticmethod
    def check(orig):
        """Why the scratch tests can't run with this status (None if they can)."""
        if orig['acquiring']:
            return 'MCA is acquiring'
        if orig['protected']:
            return 'MCA is protected (locked)'
        if orig['absolute_peak']:
            return 'absolute-peak mode is on; the library can\'t send CONTROL (I-13)'
        group = TOTAL_CHANNELS // orig['resolution'] - 1
        if orig['group'] == group:
            return ('MCA is in group {}, the scratch group; switch to another group '
                    'first'.format(group))
        return None

    @classmethod
    def enter(cls, host):
        orig = fresh(host)
        why = cls.check(orig)
        if why:
            pytest.skip(why)
        self = cls(orig)
        set_group(host, self.group)
        assert fresh(host)['group'] == self.group
        return self

    def restore(self, host):
        """Put back flags, threshold, group and preset. Returns what still differs."""
        o = self.orig
        st = fresh(host)
        if st['acquiring']:
            stop(host, st)
        control(host, o['flags'], threshold_word(o))   # first: the group depends on R
        set_group(host, o['group'])
        set_preset(host, o['preset'])
        st = fresh(host)
        return {k: (o[k], st[k]) for k in SETTINGS if o[k] != st[k]}

    def read(self, host, nwords=None):
        """The scratch group's channels (default: all R)."""
        return read_words(host, 0, nwords or self.R)


# ---------------------------------------------------------------- settings (hw_state)

def threshold(mca, host, scratch):
    """§10 CONTROL: SetThreshold sets the threshold and keeps the flags. Values above
    R/2 - 1 are clamped to it (I-8; seen at 2048)."""
    sync(mca)
    top = scratch.R // 2 - 1
    for t in (0, 1, 100, top):
        assert mca.SetThreshold(t) == 0
        st = fresh(host)
        assert (mca.Threshold, st['threshold'], st['absolute_peak']) == (t, t, False)
        assert st['flags'] == scratch.orig['flags']
    for t in (top + 1, 16383):
        assert mca.SetThreshold(t) == 0
        assert (mca.Threshold, fresh(host)['threshold']) == (top, top)


def preset(mca, host, scratch):
    """§10 PRESET: 24-bit preset time in seconds."""
    for p in (0, 1, 3600, 2**24 - 1):
        assert mca.SetPresetTime(p) == 0
        assert mca.PresetTime == p
        st = fresh(host)
        assert st['preset'] == p
        assert st['flags'] == scratch.orig['flags']


def resolution(mca, host, scratch):
    """SetADCResolution at each resolution from 256 up to the original one. After each
    change the host moves to the highest group at the new resolution, which lies inside
    the scratch group. Hardware: only with --hw-any-memory. Between the two commands
    the MCA is in the scratch group's number at the new resolution, i.e. in other memory,
    and the SET GROUP writes the parameters into that group's last 4 channels (I-27)."""
    sync(mca)
    keep = scratch.orig['flags'] & ~7
    R = 256
    while R <= scratch.R:
        assert mca.SetADCResolution(R) == 0
        assert mca.ADCResolution == R
        st = fresh(host)
        assert st['resolution'] == R
        assert st['flags'] & ~7 == keep
        assert st['threshold'] == scratch.orig['threshold']
        group = TOTAL_CHANNELS // R - 1
        set_group(host, group)
        assert fresh(host)['group'] == group
        R *= 2


# ---------------------------------------------------------------- acquisition, delete (hw_delete)

def start_stop(mca, host, scratch):
    """StartAcquisition / StopAcquisition with preset 0; the real time runs only between.
    The real time is START_LAG shorter than the time between the two commands; `wall`
    also includes StopAcquisition's 0.2 s wait and status read (up to ~0.5 s)."""
    delete(host, False, True, scratch.R)
    set_preset(host, 0)
    sync(mca)
    t0 = host.clock.now
    assert mca.StartAcquisition() == 0          # waits 1 s, then reads the status
    assert mca.isRunning
    assert fresh(host)['acquiring']
    host.wait(2.0)
    assert mca.StopAcquisition() == 0
    wall = host.clock.now - t0
    assert not mca.isRunning
    st = fresh(host)
    assert not st['acquiring']
    assert wall - START_LAG - 0.6 < st['real'] <= wall - START_LAG + 0.05, (
        st['real'], st['live'], wall)
    assert st['live'] <= st['real'] + TICK
    assert mca.RealTime == pytest.approx(st['real'], abs=TICK)
    host.wait(1.0)
    assert fresh(host)['real'] == st['real']


def preset_stop(mca, host, scratch):
    """§10: acquisition stops by itself when the preset timer reaches the preset."""
    delete(host, False, True, scratch.R)
    sync(mca)
    assert mca.SetPresetTime(3) == 0
    assert mca.StartAcquisition() == 0          # counting starts after START_LAG
    host.wait(3.5)
    st = fresh(host)
    assert not st['acquiring']
    timer = 'live' if st['flags'] & LIVE_TIMER else 'real'
    assert st[timer] == pytest.approx(3.0, abs=0.03), (st['real'], st['live'])


def stale_flags_restart(mca, host, scratch):
    """I-14: after a stop by another program, SetThreshold mustn't restart acquisition."""
    delete(host, False, True, scratch.R)
    set_preset(host, 0)
    sync(mca)
    assert mca.StartAcquisition() == 0
    stop(host, fresh(host))                     # library's cache still says acquiring
    assert mca.SetThreshold(scratch.orig['threshold']) == 0
    st = fresh(host)
    if st['acquiring']:
        stop(host, st)
    assert not st['acquiring']


def _delete_case(mca, host, scratch, method, data_cleared, time_cleared):
    run_for(host, 2.5)
    before = fresh(host)
    assert before['real'] > 1.0, before['real']
    n = scratch.ordinary
    data_before = scratch.read(host, n)
    sync(mca)
    assert getattr(mca, method)() == 0
    st = fresh(host)
    if time_cleared:
        assert st['real'] == pytest.approx(0.0, abs=TICK)
        assert st['live'] == pytest.approx(0.0, abs=TICK)
        assert mca.RealTime == pytest.approx(0.0, abs=TICK)
    else:
        assert (st['real'], st['live']) == (before['real'], before['live'])
    data = scratch.read(host, n)
    if data_cleared:
        assert not data.any()
    else:
        assert (data == data_before).all()
    assert st['group'] == scratch.group


def delete_time(mca, host, scratch):
    """DeleteTime clears real and live time and keeps the data."""
    _delete_case(mca, host, scratch, 'DeleteTime', False, True)


def delete_data(mca, host, scratch):
    """DeleteData clears the data (below the top 1/32) and keeps the times.
    Without detector input the scratch data are usually zero already; on the sim not."""
    _delete_case(mca, host, scratch, 'DeleteData', True, False)


def delete_data_and_time(mca, host, scratch):
    _delete_case(mca, host, scratch, 'DeleteDataAndTime', True, True)


# ---------------------------------------------------------------- power-down (hw_slow)

def power_down_and_recover(mca, host, idle):
    """I-20: on battery, idle until the MCA powers down; the library fails cleanly, and
    PowerOn + Initialize bring it back with its state intact."""
    before = fresh(host)
    if not before['battery']:
        pytest.skip('on external power; the MCA powers down only on battery')
    host.wait(idle)                             # no traffic at all
    raw = host.status()
    assert len(raw) < STATUS_SIZE, 'still answering after {} s: {}'.format(idle, raw.hex())
    assert mca.SendCommand(MCA8000A.Command_SendData(0, 1)) == 1
    assert mca.ReceiveStatusFromPrompt() == 1
    mca.serial_connection.baudrate = 4800       # the MCA wakes up at 4800
    mca.PowerOn()
    assert mca.is_USB_MCA
    assert mca.Initialize(baudrate=115200) == 0
    after = fresh(host)
    changed = {k: (before[k], after[k]) for k in SETTINGS + ('real', 'live')
               if before[k] != after[k]}
    assert not changed


# (scenario, marks): the hw marks pick the opt-in flag on hardware; the sim runs them all
SCENARIOS = [
    pytest.param(threshold, marks=pytest.mark.hw_state, id='threshold'),
    pytest.param(preset, marks=pytest.mark.hw_state, id='preset'),
    pytest.param(resolution, marks=pytest.mark.hw_any_memory, id='resolution'),
    pytest.param(start_stop, marks=pytest.mark.hw_delete, id='start_stop'),
    pytest.param(preset_stop, marks=pytest.mark.hw_delete, id='preset_stop'),
    pytest.param(stale_flags_restart, id='stale_flags_restart', marks=[
        pytest.mark.hw_delete, pytest.mark.xfail(reason=I14, strict=True)]),
    pytest.param(delete_time, marks=pytest.mark.hw_delete, id='delete_time'),
    pytest.param(delete_data, marks=pytest.mark.hw_delete, id='delete_data'),
    pytest.param(delete_data_and_time, marks=pytest.mark.hw_delete,
                 id='delete_data_and_time'),
]
