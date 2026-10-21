"""Command encoders (module-level functions of MCA8000A.py). docs/protocol.md §5, §9, §10.

Expected bytes come from the protocol doc's tables, computed here independently
(mcasim.host.block_command is the DLL layout, confirmed on hardware).
"""

import pytest

import MCA8000A as lib
from mcasim.host import block_command, with_checksum

def test_add_checksum_appends_sum_mod_256():
    data = bytearray([0x75, 0xFF, 0xFF, 0x01])
    out = lib.addCheckSum(data)
    assert out is data
    assert out == bytearray([0x75, 0xFF, 0xFF, 0x01, (0x75 + 0xFF + 0xFF + 1) % 256])


@pytest.mark.parametrize('divisor', [1, 2, 12, 24, 255])
def test_set_baud_rate(divisor):
    assert lib.Command_SetBaudRate(divisor) == with_checksum(0, 0, 0, divisor)


# ---------------------------------------------------------------- block data commands

@pytest.mark.parametrize('upper, group, code', [
    (False, False, 0x40), (True, False, 0x42), (False, True, 0x50), (True, True, 0x52)])
def test_send_data_codes(upper, group, code):
    assert lib.Command_SendData(0, 1024, upper=upper, group=group) == \
        block_command(code, 0, 1024)


@pytest.mark.parametrize('nwords', [1, 2, 3, 5, 64, 255, 256, 257, 300, 513, 1023, 1024])
def test_send_data_word_count(nwords):
    # DLL layout (confirmed on hardware), W = N - 1, bits 9-8 in byte 1, 7-0 in byte 3
    assert lib.Command_SendData(0, nwords) == block_command(0x40, 0, nwords)


@pytest.mark.parametrize('start', [0, 1, 63, 64, 256, 1000, 1024, 16000, 16383])
def test_send_data_start_channel(start):
    # S bits 5-0 in byte 1 bits 7-2, S bits 13-6 in byte 2 (§9). The old layout
    # (S bits 13-8 in byte 1, 7-0 in byte 2) agreed only for 0 and 16383.
    assert lib.Command_SendData(start, 7) == block_command(0x40, start, 7)


@pytest.mark.parametrize('start, nwords, exc', [
    (-1, 1, ValueError), (16384, 1, ValueError), (0, 0, ValueError), (0, 1025, ValueError),
    (0.0, 1, TypeError), (0, 1.0, TypeError)])
def test_send_data_rejects_bad_arguments(start, nwords, exc):
    with pytest.raises(exc):
        lib.Command_SendData(start, nwords)


# ---------------------------------------------------------------- CONTROL

@pytest.mark.parametrize('flags, threshold', [(0, 0), (0x14, 64), (0x0B, 0x1234), (0xFF, 16383)])
def test_control(flags, threshold):
    assert lib.Command_Control(flags, threshold) == \
        with_checksum(0x01, flags, threshold & 0xFF, threshold >> 8)


@pytest.mark.parametrize('threshold', [0, 64, 16383])
def test_control_absolute_peak(threshold):
    # absolute peak mode: bit 7 of byte 3 (threshold word bit 15, §10)
    assert lib.Command_Control(0x13, threshold, absolute_peak=True) == \
        with_checksum(0x01, 0x13, threshold & 0xFF, 0x80 | threshold >> 8)


@pytest.mark.parametrize('threshold, exc', [(-1, ValueError), (16384, ValueError),
                                            (1.0, TypeError)])
def test_control_rejects_bad_threshold(threshold, exc):
    with pytest.raises(exc):
        lib.Command_Control(0, threshold)


# ---------------------------------------------------------------- PRESET TIME

@pytest.mark.parametrize('preset', [0, 1, 255, 256, 0x123456, 2**24 - 1])
def test_preset_time_little_endian(preset):
    assert lib.Command_PresetTime(preset) == \
        with_checksum(0x02, preset & 0xFF, (preset >> 8) & 0xFF, preset >> 16)


@pytest.mark.parametrize('preset, exc', [(-1, ValueError), (2**24, ValueError),
                                         (1.5, TypeError)])
def test_preset_time_rejects_bad_values(preset, exc):
    with pytest.raises(exc):
        lib.Command_PresetTime(preset)


# ---------------------------------------------------------------- DELETE

@pytest.mark.parametrize('data, time', [(True, True), (True, False), (False, True)])
def test_delete(data, time):
    assert lib.Command_DeleteDataAndTime(delete_data=data, delete_time=time) == \
        with_checksum(0x05, int(data), int(time), 1)


def test_delete_defaults_to_both():
    assert lib.Command_DeleteDataAndTime() == with_checksum(0x05, 1, 1, 1)


# ---------------------------------------------------------------- LOCK (encoding only)

@pytest.mark.parametrize('n', [0, 1, 0x1234, 0xFFFF])
def test_lock_encoding(n):
    # encoded here, never sent to hardware (no known way to undo a lock)
    assert lib.Command_MCALock(n) == with_checksum(0x75, n & 0xFF, n >> 8, 1)


@pytest.mark.parametrize('n, exc', [(-1, ValueError), (2**16, ValueError), ('1', TypeError)])
def test_lock_rejects_bad_numbers(n, exc):
    with pytest.raises(exc):
        lib.Command_MCALock(n)


# ---------------------------------------------------------------- through the simulator

def test_encoders_understood_by_simulator(host, sim):
    assert host.command(bytes(lib.Command_PresetTime(0x012345))) == (True, True)
    assert sim.preset == 0x012345
    assert host.command(bytes(lib.Command_Control(0x03 | 0x08, 100))) == (True, True)
    assert sim.resolution == 2048 and sim.flags & 0x08 and sim.threshold_word == 100
    sim.memory[:] = 1
    assert host.command(bytes(lib.Command_DeleteDataAndTime(delete_time=False))) == \
        (True, True)
    assert sim.memory[:2048].sum() == 0


@pytest.mark.parametrize('start', [0, 70])
def test_send_data_reads_the_right_channels(host, sim, start):
    sim.memory[:200] = range(200)
    host.command(bytes(lib.Command_SendData(start, 4)))
    data = host.collect()[20:]
    assert [int.from_bytes(data[i:i + 2], 'little') for i in range(0, 8, 2)] == \
        list(range(start, start + 4))
