"""Checks against responses recorded from S/N 1224 (tests/data/capture-1224.json).

The capture is read-only data from tests/data/capture.py: status prompts and block
reads of group 0 at 2048 channels, 115200 baud.
"""

import pytest

from mcasim.host import parse_status
from mcasim.recorded import captured_counts, configure_sim, load_capture

CAP = load_capture(1224)
TRANSFERS = CAP['transfers']


def test_capture_is_consistent():
    assert CAP['serial'] == 1224 and CAP['resolution'] == 2048
    for raw in CAP['statuses']:
        assert parse_status(bytes.fromhex(raw))['checksum_ok']
    for t in TRANSFERS:
        raw = t.get('status') or t['prompt']
        assert parse_status(bytes.fromhex(raw))['checksum_ok']
        if 'data' in t:
            assert len(t['data']) == 4 * t['words']


def test_data_checksum_is_the_previous_byte_sum():
    checked = 0
    for t in TRANSFERS:
        if 'previous_sum' in t:
            assert parse_status(bytes.fromhex(t['status']))['field'] == t['previous_sum']
            checked += 1
    assert checked >= 5


def test_first_data_status_after_prompts_reports_zero():
    # the capture starts with prompts; a prompt clears the pending checksum
    assert parse_status(bytes.fromhex(TRANSFERS[0]['status']))['field'] == 0


def test_top_channels_hold_group_parameters():
    # I-19 table in docs/protocol.md §9 (unchanged since 2026-10-13)
    counts = captured_counts(CAP)
    assert [int(c) for c in counts[2042:]] == [
        0x20022420, 0x05001625, 0x004003B1, 0x0906, 0x0A4B, 0x0A00]
    assert counts[1984] == 2


@pytest.mark.parametrize('raw', CAP['statuses'])
def test_library_parses_recorded_prompt_status(mca, raw):
    assert mca.UpdateStatusFromData(bytearray.fromhex(raw), hasSerialNumber=True) == 0
    assert mca.SerialNumber == 1224
    assert (mca.PresetTime, mca.Threshold, mca.ADCResolution) == (10, 64, 2048)
    assert mca.RealTime == pytest.approx(10.0)
    assert mca.LiveTime == pytest.approx(9 + 69 / 75)


def test_library_parses_recorded_checksum_status(mca):
    t = TRANSFERS[1]
    assert mca.UpdateStatusFromData(bytearray.fromhex(t['status'])) == 0
    assert mca.LastDataCheckSum == t['previous_sum']


def test_simulator_reproduces_capture(host, sim):
    """Replay the capture's commands on the simulator set to the captured state:
    every status and every data byte must match."""
    configure_sim(sim, CAP)
    for raw in CAP['statuses']:
        assert host.status().hex() == raw
    for t in TRANSFERS:
        if 'prompt' in t:
            assert host.status().hex() == t['prompt']
            continue
        assert host.command(bytes.fromhex(t['command'])) == (True, True)
        got = host.collect()
        assert got[:20].hex() == t['status'], hex(t['code'])
        assert got[20:].hex() == t['data'], hex(t['code'])
