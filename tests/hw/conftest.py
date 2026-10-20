"""Tier-2 fixtures: a guarded port on the real MCA, the library on it, and the oracle.

Every port is a GuardedSerial (tests/lockguard.py): SET LOCK can't be sent. Each test gets
a freshly opened port (which puts the MCA back to 4800 baud, I-25) and closes it at the
end, even after a failure. The checks use the hand-driven Host (mcasim/host.py) as an
oracle independent of MCA8000A.py.

At the end of every session, the settings and the times of the current group must be as
they were at the start. A state session (--hw-state/-delete/-any-memory) also reads all 32768
channels at its start and end (saved in hwcheck/results/) and requires them unchanged
outside the scratch group, except the parameter channels of the groups it visits (I-27,
tests/state.py); state tests work in the scratch group and restore the settings after
each test.
"""

import contextlib
import io
import pathlib
import time

import numpy as np

import pytest

import MCA8000A
import state
from lockguard import GuardedSerial
from mcasim.host import Host, parse_status


def open_port(path):
    return GuardedSerial(path, baudrate=4800, parity='E', timeout=0.2, write_timeout=0.2)


def read_status(port, tries=3):
    host = Host.on_port(port)
    raw = b''
    for _ in range(tries):
        raw = host.status()
        if len(raw) == 20 and sum(raw[:19]) % 256 == raw[19]:
            return parse_status(raw), raw
        time.sleep(0.2)
    return None, raw


def state_session(config):
    return any(config.getoption(o) for o in ('--hw-state', '--hw-delete', '--hw-any-memory'))


RESULTS = pathlib.Path(__file__).parents[2] / 'hwcheck' / 'results'


def memory_snapshot(path, name):
    """All memory, read by the host at 115200 on a fresh port; saved under
    hwcheck/results/ (local) for diagnosis."""
    port = open_port(path)
    try:
        host = Host.on_port(port)
        assert host.connect(115200) == (True, True)
        memory = state.snapshot(host)
    finally:
        port.close()
    if RESULTS.is_dir():
        np.save(RESULTS / 'memory-{}-{}.npy'.format(time.strftime('%Y%m%d-%H%M%S'), name),
                memory)
    return memory


@pytest.fixture(scope='session')
def hw_info(request):
    """Once per session: optional power-on, then a status check at 4800 (and the S/N);
    for a state session, a memory snapshot. At the end, checks that nothing changed
    outside the scratch group."""
    st = connect_once(request)
    memory = None
    if state_session(request.config) and state.Scratch.check(st) is None:
        memory = memory_snapshot(st['path'], 'start')
    yield st
    port = open_port(st['path'])
    try:
        after, raw = read_status(port)
    finally:
        port.close()
    assert after is not None, 'no valid status at the end: ' + raw.hex()
    changed = {k: (st[k], after[k]) for k in state.SETTINGS if st[k] != after[k]}
    if not st['acquiring']:
        changed.update({k: (st[k], after[k]) for k in ('real', 'live')
                        if st[k] != after[k]})
    assert not changed, 'session changed MCA settings or times: {}'.format(changed)
    if memory is not None:
        allowed = state.Scratch(st).allowed_changes(
            any_memory=request.config.getoption('--hw-any-memory'))
        diff = state.changed_ranges(memory, memory_snapshot(st['path'], 'end'), allowed)
        assert not diff, 'memory outside the scratch group changed: channels {}'.format(diff)


def connect_once(request):
    path = request.config.getoption('--mca-port')
    port = open_port(path)
    try:
        if request.config.getoption('--power-on'):
            mca = MCA8000A.MCA8000A(None, serial_connection=port)
            with contextlib.redirect_stdout(io.StringIO()):
                mca.PowerOn()
            if not mca.is_USB_MCA:
                pytest.exit('power-on: no USB MCA detected on ' + path, returncode=3)
        st, raw = read_status(port)
        if st is None:
            pytest.exit('no valid status at 4800 baud on {} (got {!r}). Is the MCA on? '
                        'Try --power-on.'.format(path, raw.hex()), returncode=3)
        want = request.config.getoption('--mca-serial')
        if want is not None and st['serial'] != want:
            pytest.exit('MCA S/N {} answered, expected {}'.format(st['serial'], want),
                        returncode=3)
        st['path'] = path
        return st
    finally:
        port.close()


@pytest.fixture
def port(request, hw_info):
    p = open_port(hw_info['path'])
    yield p
    p.close()
    time.sleep(0.1)


@pytest.fixture
def mca(port):
    """The library on the guarded port (M-1), not yet connected (4800)."""
    return MCA8000A.MCA8000A(None, serial_connection=port)


@pytest.fixture
def connected(mca):
    """The library connected at 115200 (D6)."""
    assert mca.Initialize(baudrate=115200) == 0
    return mca


@pytest.fixture
def oracle(port):
    """The hand-driven host on the same port, at its current rate. Create it after the
    library has set the rate (request `connected` first)."""
    return Host.on_port(port)


@pytest.fixture
def scratch(request, hw_info, connected, oracle):
    """The scratch group (tests/state.py). Afterwards the settings are restored on a
    freshly opened port, whatever the test left behind; if that fails, the session
    stops."""
    s = state.Scratch.enter(oracle)
    yield s
    connected.serial_connection.close()
    time.sleep(0.1)
    port = open_port(hw_info['path'])
    try:
        changed = s.restore(Host.on_port(port))
    except AssertionError as e:
        changed = str(e)
    finally:
        port.close()
    if changed:
        pytest.exit('could not restore the MCA settings: {}; original status: {}'.format(
            changed, s.orig), returncode=3)
