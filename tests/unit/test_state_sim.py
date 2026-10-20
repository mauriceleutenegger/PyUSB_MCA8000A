"""The state-changing scenarios (tests/state.py) on the simulator, plus their helpers."""

import numpy as np
import pytest

import state
from mcasim import SimMCA, TOTAL_CHANNELS
from mcasim.host import Host


@pytest.fixture
def sim(clock):
    """Set up like S/N 1224 (tests/data/capture-1224.json): 2048 channels, group 0,
    preset 10, threshold 64; every group holds data."""
    s = SimMCA(clock)
    s.flags = 0x03
    s.preset = 10
    s.threshold_word = 64
    s.real[0], s.live[0] = 10.0, 9.92
    s.memory[:] = np.random.default_rng(1).integers(1, 2**20, TOTAL_CHANNELS)
    return s


@pytest.fixture
def oracle(connected, sim):
    """The host on the library's port (as on hardware)."""
    return Host.on_port(connected.serial_connection, sim=sim)


@pytest.fixture
def scratch(oracle, sim):
    memory = sim.memory.copy()
    times = (sim.real[0], sim.live[0])
    s = state.Scratch.enter(oracle)
    yield s
    assert s.restore(oracle) == {}
    assert state.changed_ranges(memory, sim.memory, [s.channels]) == []
    assert (sim.real[0], sim.live[0]) == times


@pytest.mark.parametrize('scenario', state.SCENARIOS)
def test_state(connected, oracle, scratch, scenario):
    scenario(connected, oracle, scratch)


def test_scratch_is_the_highest_group(oracle, scratch, sim):
    assert scratch.group == 15 and sim.group == 15
    assert scratch.channels == slice(30720, 32768)
    assert scratch.allowed_changes() == [slice(30720, 32768), slice(2044, 2048),
                                         slice(18428, 18432)]
    ranges = sorted((s.start, s.stop) for s in scratch.allowed_changes(any_memory=True))
    assert ranges == [(2044, 2048), (4092, 4096), (8188, 8192), (16380, 16384),
                      (18428, 18432), (30720, 32768)]      # + 256, 512, 1024


def test_restore_after_a_test_left_a_mess(oracle, sim):
    s = state.Scratch.enter(oracle)
    sim.flags = 0x10 | 0x08 | 0x05                # acquiring, live timer, 512
    sim.threshold_word = 5
    sim.preset = 99
    sim.group = 60
    assert s.restore(oracle) == {}
    assert (sim.flags, sim.threshold_word, sim.preset, sim.group) == (0x03, 64, 10, 0)


@pytest.mark.parametrize('why, change', [
    ('acquiring', lambda s: (setattr(s, 'preset', 0), setattr(s, 'flags', 0x13))),
    ('protected', lambda s: setattr(s, 'flags', 0x23)),
    ('I-13', lambda s: setattr(s, 'threshold_word', 0x8000 | 64)),
    ('scratch group', lambda s: setattr(s, 'group', 15)),
])
def test_scratch_refuses(oracle, sim, why, change):
    change(sim)
    with pytest.raises(pytest.skip.Exception, match=why):
        state.Scratch.enter(oracle)
    assert sim.group == (15 if why == 'scratch group' else 0)


def test_snapshot_reads_all_memory(oracle, sim):
    snap = state.snapshot(oracle)
    assert (snap == sim.memory).all()
    assert sim.group == 0


@pytest.mark.parametrize('flags', [0x00, 0x03, 0x06])
def test_snapshot_at_other_resolutions(oracle, sim, flags):
    sim.flags = flags
    sim.group = 1
    assert (state.snapshot(oracle) == sim.memory).all()
    assert sim.group == 1


def test_changed_ranges():
    a = np.zeros(100, dtype=np.uint32)
    b = a.copy()
    b[[3, 4, 5, 50, 90, 91]] = 1
    assert state.changed_ranges(a, b, [slice(80, 100)]) == [(3, 5), (50, 50)]


def test_power_down_and_recover(connected, oracle, sim):
    sim.battery = 91
    sim.idle_powerdown = 180
    state.power_down_and_recover(connected, oracle, idle=360)
    assert sim.powered
