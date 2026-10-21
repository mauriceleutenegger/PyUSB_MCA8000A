"""Tests of the simulator itself, driven by the hand-written host (not the library).

Each test pins down one behaviour the simulator claims to model, with its source in
docs/protocol.md or the hwcheck results. The last section runs the library against the
simulator as a smoke test.
"""

import numpy as np
import pytest

from mcasim.host import Host, block_command, parse_status, with_checksum

PROMPT_LEN = 20


def status_of(host):
    raw = host.status()
    assert len(raw) == PROMPT_LEN, raw.hex()
    st = parse_status(raw)
    assert st['checksum_ok']
    return st


# ---------------------------------------------------------------- handshake (§6, §7)

def test_prompt_returns_status_with_serial_and_group(host, sim):
    sim.group = 3
    st = status_of(host)
    assert st['serial'] == 1224
    assert st['group'] == 3
    assert st['resolution'] == 1024


def test_valid_command_gets_both_toggles(host):
    assert host.command(with_checksum(0x02, 10, 0, 0)) == (True, True)


def test_rejected_command_gets_no_toggle2_and_a_status(host, sim):
    # checksum below the true sum is rejected; RTS low then gives a status
    t1, t2 = host.command(bytes([0x02, 10, 0, 0, 11]))
    assert (t1, t2) == (True, False)
    assert sim.preset == 0
    raw = host.collect()
    assert len(raw) == PROMPT_LEN and parse_status(raw)['checksum_ok']


def test_checksum_test_is_lenient(host, sim):
    # accepted when checksum byte >= sum (mod 256)
    assert host.command(bytes([0x02, 10, 0, 0, 0xFF])) == (True, True)
    assert sim.preset == 10


def test_strict_checksum_option(host, sim):
    sim.checksum_rule = 'strict'
    assert host.command(bytes([0x02, 10, 0, 0, 0xFF])) == (True, False)


def test_unknown_code_rejected(host):
    assert host.command(with_checksum(0x03, 1, 1, 1)) == (True, False)


def test_truncated_command_gets_no_toggle2(host):
    assert host.command(with_checksum(0x02, 10, 0, 0)[:4]) == (True, False)


def test_extra_byte_after_command_ignored(host, sim):
    assert host.command(with_checksum(0x02, 7, 0, 0) + b'\x00') == (True, True)
    assert sim.preset == 7


def test_accepted_control_command_sends_nothing(host):
    host.command(with_checksum(0x02, 1, 0, 0))
    assert host.collect() == b''


# ---------------------------------------------------------------- commands (§10)

def test_preset_is_little_endian_24_bit(host, sim):
    host.command(with_checksum(0x02, 0x56, 0x34, 0x12))
    assert sim.preset == 0x123456
    assert status_of(host)['preset'] == 0x123456


def test_control_sets_flags_and_threshold(host, sim):
    host.command(with_checksum(0x01, 0x03 | 0x08, 0x34, 0x80 | 0x01))
    st = status_of(host)
    assert st['resolution'] == 2048
    assert st['flags'] & 0x08
    assert st['threshold'] == 0x0134
    assert st['absolute_peak']


def test_control_cannot_set_protected_bit(host, sim):
    host.command(with_checksum(0x01, 0x24, 0, 0))
    assert not status_of(host)['protected']


def test_acquisition_times_and_preset_stop(host, sim):
    sim.start_delay = 0.0
    sim.live_fraction = 0.5
    host.command(with_checksum(0x02, 3, 0, 0))          # preset 3 s, real timer
    host.command(with_checksum(0x01, 0x04 | 0x10, 0, 0))  # start
    host.wait(1.0)
    st = status_of(host)
    assert st['acquiring']
    assert 1.0 <= st['real'] < 1.1                     # reported in 1/75 s
    assert st['live'] == pytest.approx(st['real'] / 2, abs=0.02)
    host.wait(5.0)
    st = status_of(host)
    assert not st['acquiring']
    assert st['real'] == pytest.approx(3.0, abs=1 / 75)


def test_start_delay(host, sim):
    """🔬 S/N 1224: after START no status for ~1 s; the times count from 0.97 s."""
    host.command(with_checksum(0x01, 0x04 | 0x10, 0, 0))
    host.wait(0.5)
    assert not host.prompt()
    host.wait(0.6)
    t = host.clock.now - sim.commands[-1][0]
    st = status_of(host)
    assert st['acquiring']
    assert st['real'] == pytest.approx(t - 0.97, abs=0.02)


def test_live_timer_preset(host, sim):
    sim.live_fraction = 0.5
    host.command(with_checksum(0x02, 1, 0, 0))
    host.command(with_checksum(0x01, 0x04 | 0x08 | 0x10, 0, 0))
    host.wait(3.0)
    st = status_of(host)
    assert not st['acquiring']
    assert st['live'] == pytest.approx(1.0, abs=1 / 75)
    assert st['real'] == pytest.approx(2.0, abs=1 / 75)


def test_preset_zero_runs_until_stopped(host, sim):
    host.command(with_checksum(0x01, 0x04 | 0x10, 0, 0))
    host.wait(10.0)
    assert status_of(host)['acquiring']
    host.command(with_checksum(0x01, 0x04, 0, 0))
    assert not status_of(host)['acquiring']


def test_times_report_75ths_counting_down(sim):
    # §8: T = s + 1 - f/75; whole seconds read as f = 75
    assert sim._time_bytes(10.0) == bytes([0, 0, 10, 75])
    assert sim._time_bytes(9.08) == bytes([0, 0, 9, 75 - 6])


def test_delete_clears_current_group_and_makes_mca_busy(host, sim):
    sim.memory[:] = 7
    sim.real[0] = sim.live[0] = 5.0
    host.command(with_checksum(0x05, 1, 1, 1))
    assert sim.memory[:1024].sum() == 0 and sim.memory[1024] == 7
    assert sim.real[0] == 0
    # busy: no toggle #1 within 200 ms at 1024 channels (busy 0.178 s + latency)
    sim.delete_busy = lambda R: 0.5
    host.command(with_checksum(0x05, 1, 0, 1))
    assert host.command(with_checksum(0x02, 1, 0, 0)) == (False, False)
    host.wait(0.5)
    assert host.command(with_checksum(0x02, 1, 0, 0)) == (True, True)


def test_delete_time_only(host, sim):
    sim.memory[:] = 7
    sim.real[0] = 5.0
    host.command(with_checksum(0x05, 0, 1, 1))
    assert sim.memory[0] == 7 and sim.real[0] == 0


def test_set_group_range_and_ignored_while_acquiring(host, sim):
    host.command(with_checksum(0x11, 0, 31, 1))         # 1024 channels: groups 0-31
    assert sim.group == 31
    host.command(with_checksum(0x11, 0, 32, 1))
    assert sim.group == 31
    host.command(with_checksum(0x01, 0x04 | 0x10, 0, 0))
    host.command(with_checksum(0x11, 0, 2, 1))
    assert sim.group == 31


def test_start_stamp(host, sim):
    host.command(with_checksum(0x20, 0x26, 0x10, 0x19))   # 2026-10-19
    host.command(with_checksum(0x25, 0x13, 0x45, 0x07))   # 13:45:07
    host.command(with_checksum(0x30, 1, 1, 1))
    raw = host.collect()
    assert raw == bytes([0x07, 0x45, 0x13, 0, 0x19, 0x10, 0x26, 0x20])


def test_lock_is_simulated(host, sim):
    host.command(with_checksum(0x75, 0x34, 0x12, 1))
    assert status_of(host)['protected'] and sim.lock_code == 0x1234
    sim.memory[:] = 3
    host.command(with_checksum(0x75, 0, 0, 1))
    assert not status_of(host)['protected']
    assert sim.memory.sum() == 0


# ---------------------------------------------------------------- data (§9)

def test_block_read_lower_words_dll_layout(host, sim):
    sim.memory[:2048] = np.arange(2048) | (np.arange(2048) << 16)
    host.command(block_command(0x40, 100, 300))
    raw = host.collect()
    assert len(raw) == 20 + 600
    words = np.frombuffer(raw[20:], dtype='<u2')
    assert list(words) == list(range(100, 400))


def test_block_read_upper_words(host, sim):
    sim.memory[5] = 0xABCD1234
    host.command(block_command(0x42, 5, 1))
    assert host.collect()[20:] == bytes([0xCD, 0xAB])


def test_block_read_max_start_and_count(host, sim):
    sim.flags = 0x00                                     # 16k
    sim.memory[16383] = 9
    host.command(block_command(0x40, 16383, 1))
    assert host.collect()[20:] == bytes([9, 0])
    host.command(block_command(0x40, 0, 1024))
    assert len(host.collect()) == 20 + 2048


def test_block_read_runs_into_next_group(host, sim):
    sim.memory[1024] = 42                                # group 1, channel 0
    host.command(block_command(0x40, 1023, 2))
    assert host.collect()[22:24] == bytes([42, 0])


def test_block_read_in_current_group(host, sim):
    sim.memory[2 * 1024 + 7] = 5
    sim.group = 2
    host.command(block_command(0x40, 7, 1))
    assert host.collect()[20:] == bytes([5, 0])


def test_data_checksum_arrives_with_next_data_command(host, sim):
    sim.memory[:4] = [0x0102, 0x0304, 0xFFFF, 0]
    host.command(block_command(0x40, 0, 4))
    first = host.collect()
    assert parse_status(first[:20])['field'] == 0       # no previous transfer
    expected = sum(first[20:])
    host.command(block_command(0x40, 0, 1))
    assert parse_status(host.collect()[:20])['field'] == expected


def test_prompt_clears_pending_data_checksum(host, sim):
    # observed 2026-10-19 (hwcheck/results/sim-contract-20261019)
    sim.memory[:4] = 1
    host.command(block_command(0x40, 0, 4))
    host.collect()
    status_of(host)
    host.command(block_command(0x40, 0, 1))
    assert parse_status(host.collect()[:20])['field'] == 0


def test_status_after_port_open_and_lines_low(sim):
    # observed 2026-10-19: opening raises RTS; dropping it gets a status
    from mcasim import FakeSerial
    port = FakeSerial(sim, '/dev/sim', baudrate=4800, parity='E')
    sim.clock.sleep(0.05)
    port.dtr = False
    port.rts = False
    sim.clock.sleep(0.2)
    assert port.in_waiting == 20


def test_short_rts_low_pulse_gets_no_answer(host, sim):
    # observed 2026-10-19: the library's prompt right after open gives one status.
    # Modelled as: the answer to RTS falling is dropped if RTS rises within 0.5 ms.
    host.port.rts = True
    host.wait(0.01)
    host.port.rts = False
    host.wait(0.0002)
    host.port.rts = True
    host.wait(0.01)
    assert host.port.in_waiting == 0
    host.port.rts = False
    assert len(host.collect()) == 20


def test_prompt_right_after_command(host, sim):
    # the library prompts ~0.2 ms after a command's RTS low; this works on hardware
    host.command(with_checksum(0x02, 4, 0, 0))
    host.port.rts = False
    host.wait(0.0002)
    ref = host.port.cts
    host.port.rts = True
    assert host.wait_flip(ref)
    host.port.rts = False
    raw = host.collect()
    assert len(raw) == 20 and parse_status(raw)['preset'] == 4


def test_group_data_commands_report_serial_number(host, sim):
    host.command(block_command(0x40, 0, 4))
    host.collect()
    host.command(block_command(0x50, 0, 4))
    assert parse_status(host.collect()[:20])['serial'] == 1224


@pytest.mark.parametrize('modulus', [2**16, 2**32])
def test_data_checksum_modulus_option(host, sim, modulus):
    sim.data_checksum_modulus = modulus
    sim.memory[:1024] = 0xFFFF
    host.command(block_command(0x40, 0, 1024))
    total = sum(host.collect()[20:])
    assert total > 2**16
    host.command(block_command(0x40, 0, 1))
    assert parse_status(host.collect()[:20])['field'] == total % modulus


# ---------------------------------------------------------------- baud rate (§5)

@pytest.mark.parametrize('baud', [9600, 19200, 38400, 57600, 115200])
def test_baud_change_from_4800(host, sim, baud):
    assert host.set_baud(baud) == (True, True)
    assert sim.baud == baud
    host.command(block_command(0x40, 0, 1))              # ends the stream (EndDataStream)
    host.wait(0.3)
    host.port.reset_input_buffer()
    assert status_of(host)['serial'] == 1224


def test_baud_command_only_works_at_4800(host, sim):
    host.set_baud(57600)
    assert host.set_baud(38400) == (True, False)        # sent at 57600: garbled at 4800
    assert sim.baud == 4800


def test_port_open_resets_to_4800(host, sim):
    host.set_baud(57600)
    host.command(block_command(0x40, 0, 1))              # end the stream
    host.collect()
    host.port.close()
    host.port.open()
    host.port.baudrate = 4800
    host.port.dtr = False
    host.port.rts = False
    host.collect()
    assert sim.baud == 4800                              # reset although RTS/DTR were low
    assert status_of(host)['serial'] == 1224


def test_rate_mismatch_garbles_bytes(host, sim):
    host.port.baudrate = 9600
    raw = host.status()
    assert raw == b'\xff' * 20


def test_baud_command_starts_data_stream(host, sim):
    host.set_baud(57600)
    raw = host.collect(idle=0.01, limit=1.0)
    assert len(raw) > 4000                               # ~5200 bytes/s at 57600
    assert parse_status(raw[:20])['checksum_ok']


def test_stream_stopped_by_prompt_at_57600_leaves_extra_bytes(host, sim):
    host.set_baud(57600)
    host.wait(0.2)
    host.port.reset_input_buffer()
    raw = host.status()
    assert len(raw) > 20                                 # tail of the stream + status
    assert parse_status(raw[-20:])['checksum_ok']
    assert len(host.status()) == 20                      # then normal


def test_stream_stopped_by_prompt_at_115200_mutes_until_a_byte(host, sim):
    host.set_baud(115200)
    host.wait(0.2)
    host.port.reset_input_buffer()
    assert host.status() == b''                          # muted
    assert host.status() == b''
    # any command, even a rejected one, releases the tail plus its response
    assert host.command(bytes([0x40, 0, 0, 0, 0]))[1] is False
    raw = host.collect()
    assert len(raw) > 20 and parse_status(raw[-20:])['checksum_ok']
    assert len(host.status()) == 20


def test_end_data_stream_sequence_at_115200(host, sim):
    host.set_baud(115200)
    host.wait(0.2)
    host.port.reset_input_buffer()
    assert host.command(block_command(0x40, 0, 1)) == (True, True)
    host.wait(0.3)
    host.port.reset_input_buffer()
    assert len(host.status()) == 20


# ---------------------------------------------------------------- macOS purge bug

def test_purge_right_after_write_loses_command(host, sim):
    assert host.command(with_checksum(0x02, 9, 0, 0), purge_after_write=0) == (True, False)
    assert sim.preset == 0


def test_purge_after_wire_time_is_safe(host, sim):
    wait = host.wire_time(5) + 0.002 + sim.usb_latency
    assert host.command(with_checksum(0x02, 9, 0, 0), purge_after_write=wait) == (True, True)


def test_purge_bug_can_be_switched_off(host, sim):
    sim.purge_drops_unsent = False
    assert host.command(with_checksum(0x02, 9, 0, 0), purge_after_write=0) == (True, True)


def test_purge_keeps_bytes_still_in_flight_from_mca(host, sim):
    host.command(block_command(0x40, 0, 1024))           # 2068 bytes, 4.7 s at 4800
    host.wait(0.5)
    host.port.reset_input_buffer()
    rest = host.collect()
    assert 0 < len(rest) < 2068


# ---------------------------------------------------------------- power (§4, §13)

def test_powered_down_mca_is_silent(host, sim):
    sim.power_down_now()
    assert host.prompt()                                 # one floating CTS change
    assert host.collect() == b''
    assert not host.prompt()
    assert host.command(with_checksum(0x02, 1, 0, 0)) == (False, False)


def test_idle_powerdown_on_battery_only(host, sim):
    sim.idle_powerdown = 180
    host.wait(200)
    assert sim.powered                                   # external power (battery byte 0)
    sim.battery = 80
    host.wait(200)
    assert not sim.powered


def test_no_powerdown_while_acquiring(host, sim):
    sim.idle_powerdown = 180
    sim.battery = 80
    host.command(with_checksum(0x01, 0x04 | 0x10, 0, 0))
    host.wait(300)
    assert sim.powered


def test_power_on_burst(host, sim):
    sim.real[0] = 12.0
    sim.power_down_now()
    host.port.rts = True
    host.wait(0.01)                                      # let the floating change pass
    ref_cts, ref_dsr = host.port.cts, host.port.dsr
    for _ in range(200):                                 # 1 kHz for 0.2 s
        host.port.dtr = True
        host.wait(0.0005)
        host.port.dtr = False
        host.wait(0.0005)
    host.wait(3.8)
    assert host.port.cts != ref_cts and host.port.dsr == ref_dsr   # USB MCA (§4)
    host.port.rts = False
    host.collect()
    st = status_of(host)
    assert st['real'] == pytest.approx(12.0)             # state survives


# ---------------------------------------------------------------- faults

def test_fault_drop_toggle1(host, sim):
    sim.faults.drop_toggle1 = 1
    assert host.command(with_checksum(0x02, 1, 0, 0)) == (False, False)
    assert host.command(with_checksum(0x02, 1, 0, 0)) == (True, True)


def test_fault_reject_command(host, sim):
    sim.faults.reject_commands = 1
    assert host.command(with_checksum(0x02, 1, 0, 0)) == (True, False)


def test_fault_corrupt_status(host, sim):
    sim.faults.corrupt_status = 1
    assert not parse_status(host.status())['checksum_ok']
    assert parse_status(host.status())['checksum_ok']


def test_faults_on_response_length(host, sim):
    sim.faults.truncate_response = 1
    assert len(host.status()) == 19
    sim.faults.extra_bytes = 1
    assert len(host.status()) == 21
    sim.faults.silent_responses = 1
    assert host.status() == b''


# ---------------------------------------------------------------- library smoke tests

@pytest.mark.parametrize('baud', [57600, 115200])
def test_library_initialize(mca, sim, baud, capsys):
    assert mca.Initialize(baudrate=baud) == 0
    assert sim.baud == baud
    assert mca.SerialNumber == 1224
    assert mca.ReceiveStatusFromPrompt() == 0


def test_library_without_end_data_stream_fails_at_115200(mca, sim, monkeypatch, capsys):
    # 115200 as it was before 51181ea: the MCA stays muted and no status arrives.
    monkeypatch.setattr(mca, 'EndDataStream', lambda: 0)
    mca.SetBaudRate(115200)
    assert any(kind == 'muted_no_answer' for _, kind, _ in sim.log)
    assert mca.SerialNumber == 0                          # never got a status


def test_library_is_fast_on_virtual_time(mca, sim, clock, capsys):
    import time
    sim.power_down_now()
    t0 = time.perf_counter()
    assert mca.SendCommand(with_checksum(0x02, 1, 0, 0)) == 1     # 10 retries
    assert clock.now > 1.5
    assert time.perf_counter() - t0 < 5
