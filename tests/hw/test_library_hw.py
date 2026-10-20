"""The library on the real MCA, checked against the hand-driven host. Read-only.

Sends only status prompts, block data requests (40h/42h) and the baud command (00h at
4800 right after opening the port). No test sends a baud command at another rate: the
MCA would see garbled bytes, which could decode as any command.
"""

import time

import numpy as np
import pytest

import MCA8000A
from mcasim.host import Host, parse_status

BAUDS = [4800, 9600, 19200, 38400, 57600, 115200]
I3 = 'I-3: data decoded as int16, overflows on numpy >= 2'


def oracle_status(host):
    raw = host.status()
    assert len(raw) == 20, raw.hex()
    st = parse_status(raw)
    assert st['checksum_ok'], raw.hex()
    return st


# ---------------------------------------------------------------- connecting

@pytest.mark.parametrize('baud', BAUDS)
def test_initialize(mca, port, hw_info, baud):
    assert mca.Initialize(baudrate=baud) == 0
    assert port.baudrate == baud
    time.sleep(0.5)
    assert port.in_waiting == 0                       # no stream leftovers (I-24)
    st = oracle_status(Host.on_port(port))            # I-22: don't trust the return value
    assert st['serial'] == hw_info['serial']


# ---------------------------------------------------------------- status

def test_status_matches_oracle(connected, oracle):
    # library first: its prompt fails after someone else's prompt (I-15, below)
    assert connected.ReceiveStatusFromPrompt() == 0
    st = oracle_status(oracle)
    m = connected
    assert m.SerialNumber == st['serial']
    assert m.PresetTime == st['preset']
    assert m.flags == st['flags']
    assert m.ADCResolution == st['resolution']
    assert m.isRunning == st['acquiring']
    assert m.isProtected == st['protected']
    assert m.Threshold & 0x7FFF == st['threshold']    # bit 15: I-13 (tier 1)
    assert abs(m.BatteryStatus - st['battery']) <= 2
    tol = 1.0 if st['acquiring'] else 0.0
    assert m.RealTime == pytest.approx(st['real'], abs=tol)
    assert m.LiveTime == pytest.approx(st['live'], abs=tol)
    assert m.StatusChecksumErrors == 0


def test_prompt_after_another_prompt(connected, oracle):
    # I-15: one prompt not made by the library toggles CTS once. The MCA toggles CTS
    # again within a fraction of a ms of RTS rising, back to the level the library
    # remembered, so before the fix the library saw no change and timed out
    # (2026-10-19, 3/3).
    oracle_status(oracle)
    assert connected.ReceiveStatusFromPrompt() == 0


@pytest.mark.parametrize('baud', [4800, 57600, 115200])
def test_many_commands_and_prompts(mca, port, baud):
    """Timing margin: 100 rounds of prompt + 1-word data request without a failure.
    The macOS purge race (timing.md) depends on the rate (D6)."""
    assert mca.Initialize(baudrate=baud) == 0
    failures = []
    for i in range(100):
        if mca.ReceiveStatusFromPrompt():
            failures.append((i, 'prompt'))
        if mca.SendCommand(MCA8000A.Command_SendData(0, 1)):
            failures.append((i, 'send'))
            continue
        if mca.ReceiveStatus():
            failures.append((i, 'status'))
        stat, data = mca.ReceiveData(2)
        if stat:
            failures.append((i, 'data'))
    time.sleep(0.3)
    assert failures == []
    assert mca.StatusChecksumErrors == 0
    assert port.in_waiting == 0


# ---------------------------------------------------------------- data

def library_block(mca, start, nwords):
    assert mca.SendCommand(MCA8000A.Command_SendData(start, nwords)) == 0
    assert mca.ReceiveStatus() == 0
    stat, data = mca.ReceiveData(2 * nwords)
    assert stat == 0
    return bytes(data)


def oracle_block(host, code, start, nwords):
    got = host.read_block(code, start, nwords)
    assert got is not None
    st, data = got
    assert st['checksum_ok'] and len(data) == 2 * nwords
    return data


def test_block_at_start_0_matches_oracle(connected, oracle):
    assert library_block(connected, 0, 64) == oracle_block(oracle, 0x40, 0, 64)


def test_block_at_other_start_matches_oracle(connected, oracle):
    # I-1: the old encoding made the MCA read from S' = (S >> 8) | (S & 0xFF) << 6.
    # Pick an S whose data differs from the data at S'; skip if the spectrum has none.
    n = 16
    for start in range(n, 1024, n):
        wrong = (start >> 8) | ((start & 0xFF) << 6)
        if wrong + n > 16384:
            continue
        right_data = oracle_block(oracle, 0x40, start, n)
        if right_data != oracle_block(oracle, 0x40, wrong, n):
            break
    else:
        pytest.skip('no start channel in 16..1008 tells the layouts apart')
    assert library_block(connected, start, n) == right_data


def oracle_spectrum(host, nwords):
    lower = oracle_block(host, 0x40, 0, nwords)
    upper = oracle_block(host, 0x42, 0, nwords)
    if sum(lower) >= 2**16 or sum(upper) >= 2**16:
        pytest.skip('byte sum >= 2^16: the checksum question I-11 is open')
    lo = np.frombuffer(lower, dtype='<u2').astype(np.int64)
    hi = np.frombuffer(upper, dtype='<u2').astype(np.int64)
    return lo + (hi << 16)


@pytest.fixture
def spectrum(connected, oracle):
    st = oracle_status(oracle)
    if st['acquiring']:
        pytest.skip('MCA is acquiring; the spectrum would change between reads')
    nwords = min(st['resolution'], 1024)              # what the library reads (M-2)
    return st['resolution'], oracle_spectrum(oracle, nwords)


@pytest.mark.xfail(reason=I3, strict=True)
def test_receive_channel_data_raw(connected, spectrum):
    _, expected = spectrum
    assert connected.ReceiveChannelData() == 0
    assert list(connected.RawChannelData) == list(expected)


@pytest.mark.xfail(reason=I3 + '; also I-26 if channels 992-1023 have counts at '
                   'resolution >= 2048', strict=True)
def test_receive_channel_data_zeroes_only_top_channels(connected, spectrum):
    resolution, expected = spectrum
    expected = expected.copy()
    expected[resolution - resolution // 32:] = 0     # beyond the read when R >= 2048
    assert connected.ReceiveChannelData() == 0
    assert list(connected.ChannelData) == list(expected)
