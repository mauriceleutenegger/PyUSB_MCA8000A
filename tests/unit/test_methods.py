"""The ten command methods that send one command and then read a status (M-5).

Pins down their common behaviour on the simulator: return codes, the command sent, the
wait before the status read, the error message on each failure path, and (for the four
CONTROL methods) the fresh status read before the command (I-14).
"""

import pytest

# name, call, command code, wait after the command (s), CONTROL (fresh status first),
# message prefix
METHODS = [
    ('SetThreshold', lambda m: m.SetThreshold(50), 0x01, 0.0, True, 'SetThreshold'),
    ('SetADCResolution', lambda m: m.SetADCResolution(1024), 0x01, 0.0, True,
     'SetADCResolution'),
    ('StartAcquisition', lambda m: m.StartAcquisition(), 0x01, 1.0, True,
     'StartAcquisition'),
    ('StopAcquisition', lambda m: m.StopAcquisition(), 0x01, 0.2, True, 'StopAcquisition'),
    ('SetPresetTime', lambda m: m.SetPresetTime(7), 0x02, 0.2, False, 'SetPresetTime'),
    ('DeleteDataAndTime', lambda m: m.DeleteDataAndTime(), 0x05, 0.2, False,
     'DeleteDataAndTime'),
    ('DeleteData', lambda m: m.DeleteData(), 0x05, 0.2, False, 'DeleteData'),
    ('DeleteTime', lambda m: m.DeleteTime(), 0x05, 0.2, False, 'DeleteTime'),
    ('SetLock', lambda m: m.SetLock(0x1234), 0x75, 0.0, False, 'SetLock'),
    ('SetLockToZero', lambda m: m.SetLockToZero(), 0x75, 4.0, False, 'SetLockToZero'),
]
CASES = [pytest.param(*m[1:], id=m[0]) for m in METHODS]


def commands_since(sim, t0):
    return [(t, bytes(c), ok) for t, c, ok in sim.commands if t >= t0]


def reads_since(mca, t0):
    return [t for t, op, v in mca.serial_connection.trace if t >= t0 and op == 'read']


@pytest.mark.parametrize('call, code, delay, control, prefix', CASES)
def test_success(connected, sim, clock, capsys, call, code, delay, control, prefix):
    if code == 0x01 and delay == 0.2:              # StopAcquisition: start first
        assert connected.StartAcquisition() == 0
    capsys.readouterr()
    t0 = clock.now
    assert call(connected) == 0
    sent = commands_since(sim, t0)
    assert [(c[0], ok) for _, c, ok in sent] == [(code, True)]
    # the status read afterwards shows the command's effect
    assert connected.flags == sim.flags
    assert connected.PresetTime == sim.preset
    assert capsys.readouterr().out == ''


@pytest.mark.parametrize('call, code, delay, control, prefix', CASES)
def test_waits_before_status_read(connected, sim, clock, call, code, delay, control,
                                  prefix):
    t0 = clock.now
    assert call(connected) == 0
    t_cmd = commands_since(sim, t0)[-1][0]
    # the next RTS rise after the accepted command is the status prompt
    t_prompt = min(t for t, op, v in connected.serial_connection.trace
                   if t > t_cmd and op == 'rts' and v)
    assert delay <= t_prompt - t_cmd < delay + 0.05


@pytest.mark.parametrize('call, code, delay, control, prefix', CASES)
def test_command_fails(connected, sim, clock, capsys, call, code, delay, control, prefix):
    sim.faults.reject_commands = 100
    capsys.readouterr()
    t0 = clock.now
    assert call(connected) == 1
    sent = commands_since(sim, t0)
    assert len(sent) == 10 and not any(ok for _, _, ok in sent)
    assert f'{prefix}: error sending command' in capsys.readouterr().out.splitlines()
    assert not reads_since(connected, sent[0][0])  # no status read after the command


@pytest.mark.parametrize('call, code, delay, control, prefix', CASES)
def test_status_fails_after_command(connected, sim, clock, capsys, monkeypatch, call, code,
                                    delay, control, prefix):
    send = connected.SendCommand

    def send_then_fall_silent(command, n_retries=10):
        stat = send(command, n_retries)
        sim.faults.silent_responses = 1000          # CTS still toggles, no status comes
        return stat

    monkeypatch.setattr(connected, 'SendCommand', send_then_fall_silent)
    capsys.readouterr()
    t0 = clock.now
    assert call(connected) == 1
    assert [ok for _, _, ok in commands_since(sim, t0)] == [True]
    assert f'{prefix}: error updating status' in capsys.readouterr().out.splitlines()


@pytest.mark.parametrize('call, code, delay, control, prefix',
                         [c for c in CASES if c.values[3]])
def test_control_fails_on_fresh_status(connected, sim, clock, capsys, call, code, delay,
                                       control, prefix):
    sim.faults.silent_responses = 1000
    capsys.readouterr()
    t0 = clock.now
    assert call(connected) == 1
    assert commands_since(sim, t0) == []           # nothing sent
    assert f'{prefix}: error reading status' in capsys.readouterr().out.splitlines()


@pytest.mark.parametrize('call, code, delay, control, prefix',
                         [c for c in CASES if c.values[1] == 0x05])
def test_delete_waits_for_busy_mca(connected, sim, call, code, delay, control, prefix):
    # I-18: up to 2 s busy after DELETE (DTP); one prompt after 0.2 s isn't enough
    sim.delete_busy = lambda R: 2.0
    assert call(connected) == 0
