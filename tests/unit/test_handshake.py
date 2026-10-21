"""SendCommand, PromptForStatus, ReceiveStatus*, ReceiveData against the simulator.
docs/protocol.md §6, §7.
"""

import pytest

import MCA8000A as lib

PRESET_5 = lib.Command_PresetTime(5)


def host_ops(mca, since=0.0):
    return [(t, op, v) for t, op, v in mca.serial_connection.trace[:] if t >= since]


# ---------------------------------------------------------------- SendCommand

def test_send_command_success(connected, sim, clock):
    t0 = clock.now
    assert connected.SendCommand(PRESET_5) == 0
    assert sim.preset == 5
    accepted = [c for c in sim.commands if c[0] >= t0]
    assert [(bytes(c[1]), c[2]) for c in accepted] == [(bytes(PRESET_5), True)]
    assert connected.serial_connection.rts is False


def test_send_command_line_sequence(connected, clock):
    t0 = clock.now
    connected.SendCommand(PRESET_5)
    ops = [op for _, op, _ in host_ops(connected, t0) if op in ('rts', 'write')]
    assert ops == ['rts', 'write', 'rts']
    rts = [v for _, op, v in host_ops(connected, t0) if op == 'rts']
    assert rts == [True, False]


def test_send_command_survives_macos_purge_bug(connected, sim):
    # the fixed wait after write() keeps the RX purge from eating the command (timing.md)
    assert sim.purge_drops_unsent
    for n in range(20):
        assert connected.SendCommand(lib.Command_PresetTime(n)) == 0
        assert sim.preset == n
    assert not any('dropped_output' in kind for _, kind, _ in sim.log)


@pytest.mark.parametrize('fault', ['drop_toggle1', 'reject_commands'])
def test_send_command_retries(connected, sim, fault):
    setattr(sim.faults, fault, 3)
    assert connected.SendCommand(PRESET_5) == 0
    assert sim.preset == 5


def test_send_command_gives_up_after_10_tries(connected, sim, clock):
    sim.faults.reject_commands = 10
    assert connected.SendCommand(PRESET_5) == 1
    assert sim.preset == 0


def test_send_command_to_powered_down_mca(connected, sim, clock):
    sim.power_down_now()
    t0 = clock.now
    assert connected.SendCommand(PRESET_5) == 1
    assert 1.5 < clock.now - t0 < 3            # 10 x 0.2 s; 2.1 s observed


def test_rts_low_between_retries(connected, sim, clock):
    sim.faults.reject_commands = 2
    t0 = clock.now
    connected.SendCommand(PRESET_5)
    rts = [(t, v) for t, op, v in host_ops(connected, t0) if op == 'rts']
    lows = [t2 - t1 for (t1, v1), (t2, v2) in zip(rts, rts[1:]) if not v1 and v2]
    assert len(lows) == 2
    assert min(lows) >= 100e-6


# ---------------------------------------------------------------- status

def test_prompt_for_status(connected, sim):
    assert connected.PromptForStatus() == 0
    sim.power_down_now()
    connected.PromptForStatus()                     # floating CTS change after power-down
    assert connected.PromptForStatus() == 1


def test_receive_status_from_prompt_updates_state(connected, sim):
    sim.preset = 77
    sim.real[0] = 12.5
    assert connected.ReceiveStatusFromPrompt() == 0
    assert connected.PresetTime == 77
    assert connected.RealTime == pytest.approx(12.5, abs=1 / 75)


def test_status_retry_after_bad_checksum(connected, sim):
    sim.faults.corrupt_status = 2
    sim.preset = 9
    assert connected.ReceiveStatusFromPrompt() == 0
    assert connected.PresetTime == 9
    assert connected.StatusChecksumErrors == 2


def test_status_fails_when_mca_silent(connected, sim):
    sim.faults.silent_responses = 20
    assert connected.ReceiveStatusFromPrompt() != 0


def test_status_fails_when_every_status_is_bad(connected, sim):
    sim.faults.corrupt_status = 100             # CTS handshakes still work
    sim.preset = 9
    assert connected.ReceiveStatusFromPrompt() != 0


def test_status_fails_at_wrong_baud_rate(connected, sim):
    connected.serial_connection.baudrate = 57600    # MCA at 115200: bytes garbled
    assert connected.ReceiveStatusFromPrompt() != 0


def test_prompt_fails_if_cts_changed_since_last_handshake(connected, sim):
    sim.power_down_now()
    sim.floating_cts = False
    sim._toggle_cts('external')                 # CTS changes between handshakes
    assert connected.PromptForStatus() == 1         # no MCA to answer


# ---------------------------------------------------------------- ReceiveData

def test_receive_data_timeout(connected, sim, clock):
    t0 = clock.now
    stat, data = connected.ReceiveData(20)
    assert (stat, data) == (1, None)
    assert clock.now - t0 == pytest.approx(0.4, abs=0.01)


def test_receive_data_leaves_extra_bytes(connected, sim, clock):
    # ReceiveData never reads more than asked; a stray byte stays in the buffer
    # (the next SendCommand/prompt purges it)
    sim.faults.extra_bytes = 1
    assert connected.ReceiveStatusFromPrompt() == 0
    clock.sleep(0.05)
    assert connected.serial_connection.in_waiting == 1
