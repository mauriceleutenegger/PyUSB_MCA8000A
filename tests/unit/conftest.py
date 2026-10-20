"""Tier-1 fixtures: virtual clock, simulated MCA, and the library wired to them."""

import pytest

import MCA8000A
from mcasim import FakeSerial, SimMCA, VirtualClock
from mcasim.host import Host


@pytest.fixture
def clock(monkeypatch):
    """Virtual time, also installed as the library's `timeit`."""
    c = VirtualClock()
    monkeypatch.setattr(MCA8000A, 'timeit', c)
    return c


@pytest.fixture
def sim(clock):
    return SimMCA(clock)


@pytest.fixture
def host(sim):
    """A hand-driven host on the simulator (doesn't use the library)."""
    return Host(sim)


@pytest.fixture
def mca(sim):
    """An MCA8000A instance talking to the simulator (M-1: port passed in)."""
    port = FakeSerial(sim, '/dev/sim')
    return MCA8000A.MCA8000A(None, serial_connection=port)


@pytest.fixture
def connected(mca):
    """The library connected at 115200 (D6: the rate users run at)."""
    assert mca.Initialize(baudrate=115200) == 0
    return mca
