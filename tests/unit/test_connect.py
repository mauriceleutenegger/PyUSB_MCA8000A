"""Initialize, SetBaudRate/EndDataStream, PowerOn. docs/protocol.md §4, §5."""

import pytest

BAUDS = [4800, 9600, 19200, 38400, 57600, 115200]


@pytest.mark.parametrize('baud', BAUDS)
def test_initialize(mca, sim, baud):
    assert mca.Initialize(baudrate=baud) == 0
    assert sim.baud == baud
    assert mca.serial_connection.baudrate == baud
    assert mca.SerialNumber == 1224


@pytest.mark.parametrize('baud', [57600, 115200])
def test_no_stream_leftovers_after_initialize(mca, sim, clock, baud):
    # leftover stream bytes must not end up in front of the next transfer
    mca.Initialize(baudrate=baud)
    clock.sleep(1.0)
    assert mca.serial_connection.in_waiting == 0
    assert not sim.muted and sim.held == b''
    sim.memory[:4] = [1, 2, 3, 4]
    mca.SendCommand(bytes.fromhex('4000000343'))       # 40h, 4 words (DLL layout)
    clock.sleep(0.1)
    stat, raw = mca.ReceiveData(28)
    assert stat == 0 and raw[20:] == bytes([1, 0, 2, 0, 3, 0, 4, 0])


def test_initialize_uses_end_data_stream(mca, sim):
    mca.Initialize(baudrate=115200)
    assert any(kind == 'stop_output' for _, kind, _ in sim.log)       # stream was stopped
    assert not any(kind == 'muted_no_answer' for _, kind, _ in sim.log)


@pytest.mark.parametrize('baud', [7, 100000, 250000])
def test_set_baud_rate_rejects_non_divisors(mca, sim, baud):
    assert mca.SetBaudRate(baud) == 1
    assert sim.commands == []


def test_initialize_fails_when_mca_off(mca, sim):
    sim.power_down_now()
    assert mca.Initialize(baudrate=115200) != 0


def test_initialize_fails_when_status_never_valid(mca, sim):
    sim.faults.corrupt_status = 1000
    assert mca.Initialize(baudrate=115200) != 0


def test_second_initialize_fails_at_high_rate(mca, sim):
    # a baud change only works when sent at 4800
    assert mca.Initialize(baudrate=57600) == 0
    assert mca.SetBaudRate(57600) == 1
    assert sim.baud == 4800


# ---------------------------------------------------------------- power-on

def test_power_on_from_powered_down(sim, clock):
    import MCA8000A
    from mcasim import FakeSerial
    sim.real[0] = 42.0
    sim.power_down_now()
    mca = MCA8000A.MCA8000A(None, serial_connection=FakeSerial(sim, '/dev/sim'))
    t0 = clock.now
    mca.PowerOn()
    assert clock.now - t0 == pytest.approx(4.0, abs=0.1)
    assert sim.powered
    assert mca.is_USB_MCA is True
    assert mca.Initialize(baudrate=115200) == 0
    assert mca.RealTime == pytest.approx(42.0)          # state survives


def test_power_on_burst_shape(mca, sim, clock):
    sim.power_down_now()
    t0 = clock.now
    mca.PowerOn()
    dtr = [(t, v) for t, op, v in mca.serial_connection.trace if op == 'dtr' and t >= t0]
    assert len(dtr) == 400                              # 2 kHz for 0.1 s, both edges
    # 0.1 s nominal; each of the 400 waits overshoots by up to one virtual-clock tick
    assert 0.1 <= dtr[-1][0] - dtr[0][0] < 0.12
