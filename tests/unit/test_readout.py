"""ReceiveChannelData against the simulator. docs/protocol.md §8 (checksum), §9.

The library reads min(resolution, 1024) channels from start channel 0 (M-2 would read
the full spectrum), and sets ChannelData to RawChannelData with the top 1/32 of the
group zeroed. With numpy >= 2 every successful readout currently raises (I-3).
"""

import numpy as np
import pytest

from mcasim.recorded import captured_counts, configure_sim, load_capture

I3 = 'I-3: data decoded as int16, overflows on numpy >= 2'


def set_resolution(sim, mca, channels):
    sim.flags = (sim.flags & ~7) | (14 - channels.bit_length() + 1)
    assert sim.resolution == channels
    assert mca.ReceiveStatusFromPrompt() == 0


def check_readout(mca, counts, zero_from):
    """RawChannelData = counts; ChannelData = counts with channels >= zero_from zeroed."""
    assert mca.ReceiveChannelData() == 0
    assert list(mca.RawChannelData) == [int(c) for c in counts]
    expected = np.array(counts, dtype=np.int64)
    expected[zero_from:] = 0
    assert list(mca.ChannelData) == list(expected)


@pytest.mark.xfail(reason=I3, strict=True)
@pytest.mark.parametrize('channels', [256, 512, 1024])
def test_readout_small_resolutions(connected, sim, channels):
    set_resolution(sim, connected, channels)
    rng = np.random.default_rng(channels)
    # byte sums stay < 2^16 so that I-11 doesn't come into it
    sim.memory[:channels] = rng.integers(0, 60, channels)
    check_readout(connected, sim.memory[:channels], channels - channels // 32)


@pytest.mark.xfail(reason=I3, strict=True)
def test_readout_large_counts(connected, sim):
    set_resolution(sim, connected, 1024)
    values = [0x7FFF, 0x8000, 0xFFFF, 0x10000, 0x12345678, 0xFFFFFFFF, 0x8000FFFF]
    sim.memory[:len(values)] = values
    check_readout(connected, sim.memory[:1024], 992)


@pytest.mark.xfail(reason=I3, strict=True)
def test_readout_recorded_spectrum_at_2048(connected, sim):
    # (channels 992-1023 are empty in this spectrum, so I-26 doesn't show here)
    cap = load_capture(1224)
    configure_sim(sim, cap)
    assert connected.ReceiveStatusFromPrompt() == 0
    # only the first 1024 channels are read (M-2); none is in the top 1/32 of the group
    check_readout(connected, captured_counts(cap)[:1024], 1024)


@pytest.mark.xfail(reason=I3 + '; I-26: zeroes channels 992-1023 at resolution >= 2048',
                   strict=True)
def test_readout_does_not_zero_ordinary_channels_at_2048(connected, sim):
    set_resolution(sim, connected, 2048)
    sim.memory[:2048] = 5
    check_readout(connected, sim.memory[:1024], 1024)


@pytest.mark.parametrize('modulus', [
    pytest.param(2**16, marks=pytest.mark.xfail(reason=I3, strict=True)),
    pytest.param(2**32, marks=pytest.mark.xfail(
        reason=I3 + '; I-11: compares sum mod 2^16 with the full 32-bit field',
        strict=True))])
def test_readout_high_byte_sums(connected, sim, modulus):
    # I-11 is open: the MCA may report the checksum mod 2^16 (docs) or 32-bit (DLL).
    # The library should accept the correct value either way.
    sim.data_checksum_modulus = modulus
    set_resolution(sim, connected, 1024)
    sim.memory[:1024] = 0x00FF00FF                      # byte sums 2 x 261120 > 2^16
    check_readout(connected, sim.memory[:1024], 992)


def test_lower_word_checksum_failure(connected, sim, capsys):
    set_resolution(sim, connected, 1024)
    sim.memory[:1024] = 3
    sim.faults.corrupt_block = sim.block_transfers      # the lower-word transfer
    assert connected.ReceiveChannelData() == 1
    assert 'lower word checksum failed' in capsys.readouterr().out


def test_upper_word_checksum_failure(connected, sim):
    set_resolution(sim, connected, 1024)
    sim.memory[:1024] = 3 << 16
    sim.faults.corrupt_block = sim.block_transfers + 1  # the upper-word transfer
    assert connected.ReceiveChannelData() == 1


@pytest.mark.xfail(reason='I-6: upper-word failure reports "lower word"', strict=True)
def test_upper_word_checksum_failure_message(connected, sim, capsys):
    set_resolution(sim, connected, 1024)
    sim.faults.corrupt_block = sim.block_transfers + 1
    connected.ReceiveChannelData()
    assert 'upper word checksum failed' in capsys.readouterr().out


def test_readout_fails_cleanly_when_mca_off(connected, sim):
    sim.power_down_now()
    assert connected.ReceiveChannelData() != 0
