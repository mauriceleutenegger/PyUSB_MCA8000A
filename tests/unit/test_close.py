"""MCA8000A.close and the context manager."""

import types

import pytest

import MCA8000A
from mcasim import FakeSerial


def test_close_closes_port_it_opened(sim, monkeypatch):
    monkeypatch.setattr(MCA8000A, 'serial', types.SimpleNamespace(
        Serial=lambda *args, **kwargs: FakeSerial(sim, *args, **kwargs)))
    mca = MCA8000A.MCA8000A('/dev/cu.test')
    assert mca.serial_connection.is_open
    mca.close()
    assert not mca.serial_connection.is_open


def test_close_closes_port_passed_in(mca):
    # closing means "done with this port", whoever opened it
    mca.close()
    assert not mca.serial_connection.is_open


def test_close_twice(mca):
    mca.close()
    mca.close()
    assert not mca.serial_connection.is_open


def test_close_sends_nothing(connected):
    # no traffic to the MCA, no line changes
    port = connected.serial_connection
    before = len(port.trace)
    connected.close()
    assert [event for _, event, _ in port.trace[before:]] == ['close']


def test_close_after_power_on(mca):
    mca.PowerOn()
    mca.close()
    assert not mca.serial_connection.is_open


def test_with_returns_instance_and_closes(mca):
    with mca as m:
        assert m is mca
        assert mca.serial_connection.is_open
    assert not mca.serial_connection.is_open


def test_with_closes_on_exception_and_reraises(mca):
    with pytest.raises(KeyError):
        with mca:
            raise KeyError('from the block')
    assert not mca.serial_connection.is_open


def test_with_on_constructor(sim):
    port = FakeSerial(sim, '/dev/sim')
    with MCA8000A.MCA8000A(None, serial_connection=port) as mca:
        assert mca.ReceiveStatusFromPrompt() == 0
    assert not port.is_open
