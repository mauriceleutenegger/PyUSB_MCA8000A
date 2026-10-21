"""MCA8000A.__init__: opening the port itself, or using one passed in."""

import types

import MCA8000A
from mcasim import FakeSerial


def test_opens_port_with_library_settings(sim, monkeypatch):
    opened = {}

    def fake_serial(*args, **kwargs):
        opened['args'], opened['kwargs'] = args, kwargs
        return FakeSerial(sim, *args, **kwargs)

    monkeypatch.setattr(MCA8000A, 'serial', types.SimpleNamespace(Serial=fake_serial))
    mca = MCA8000A.MCA8000A('/dev/cu.test')
    assert opened['args'] == ('/dev/cu.test',)
    assert opened['kwargs'] == dict(baudrate=4800, parity='E',
                                    timeout=0.2, write_timeout=0.2)
    assert mca.device_path == '/dev/cu.test'
    assert mca.bits_per_byte == 11


def test_uses_port_passed_in_and_applies_settings(sim, monkeypatch):
    def no_open(*args, **kwargs):
        raise AssertionError('must not open a port')

    monkeypatch.setattr(MCA8000A, 'serial', types.SimpleNamespace(Serial=no_open))
    port = FakeSerial(sim, '/dev/sim', baudrate=115200, parity='N', timeout=None)
    mca = MCA8000A.MCA8000A(None, serial_connection=port)
    assert mca.serial_connection is port
    assert (port.baudrate, port.parity, port.timeout, port.write_timeout) == \
        (4800, 'E', 0.2, 0.2)
    assert mca.bits_per_byte == 11


def test_first_prompt_after_open(mca, clock, capsys):
    # as on S/N 1224, 5/5 (hwcheck/results/sim-contract-20261019/contract2.json):
    # one valid status, no checksum error, nothing left over
    assert mca.ReceiveStatusFromPrompt() == 0
    assert mca.SerialNumber == 1224
    assert mca.StatusChecksumErrors == 0
    clock.sleep(0.3)
    assert mca.serial_connection.in_waiting == 0
