"""SetADCResolution: channels -> flag bits 2-0 (§8: resolution = 16384 >> code, code 0-6)."""

import pytest


@pytest.fixture
def ready(mca, capsys):
    assert mca.ReceiveStatusFromPrompt() == 0
    return mca


@pytest.mark.parametrize('channels', [256, 512, 1024, 2048, 4096, 8192, 16384])
def test_powers_of_two(ready, sim, channels):
    assert ready.SetADCResolution(channels) == 0
    assert sim.resolution == channels
    assert ready.ADCResolution == channels


@pytest.mark.parametrize('channels, expected', [
    (129, 256), (200, 256), (257, 512), (300, 512), (1000, 1024), (1025, 2048),
    (9000, 16384), (16383, 16384)])
def test_rounds_up(ready, sim, channels, expected):
    assert ready.SetADCResolution(channels) == 0
    assert sim.resolution == expected


def test_keeps_other_flag_bits(ready, sim):
    sim.flags = 0x08 | 0x04                 # live timer, 1024
    ready.ReceiveStatusFromPrompt()
    ready.SetADCResolution(4096)
    assert sim.flags == 0x08 | 0x02


@pytest.mark.xfail(reason='I-7: 128 gives the undefined code 111b', strict=True)
def test_rejects_128(ready, sim):
    with pytest.raises(ValueError):
        ready.SetADCResolution(128)


@pytest.mark.parametrize('channels, exc', [(127, ValueError), (0, ValueError),
                                           (16385, ValueError), (1024.0, TypeError)])
def test_rejects_bad_values(ready, sim, channels, exc):
    with pytest.raises(exc):
        ready.SetADCResolution(channels)
    assert sim.commands == []               # nothing sent
