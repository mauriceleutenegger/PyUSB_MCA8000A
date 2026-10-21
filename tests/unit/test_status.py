"""Status parsing: MCA8000A.UpdateStatusFromData. docs/protocol.md §8."""

import json
import pathlib
import random

import pytest

DATA = json.loads((pathlib.Path(__file__).parents[1] / 'data' / 'statuses.json').read_text())
RECORDED = DATA['statuses']


def make_status(field=(1224 << 16), preset=0, battery=0, real=(0, 75), live=(0, 75),
                threshold_word=0, flags=0x04, good_checksum=True):
    """Build a status (§8). real/live are (whole seconds, 75ths byte)."""
    b = bytearray(field.to_bytes(4, 'big'))
    b += preset.to_bytes(3, 'big') + bytes([battery])
    b += real[0].to_bytes(3, 'big') + bytes([real[1]])
    b += live[0].to_bytes(3, 'big') + bytes([live[1]])
    b += threshold_word.to_bytes(2, 'big') + bytes([flags])
    b.append(sum(b) % 256 if good_checksum else (sum(b) + 1) % 256)
    return b


def parsed(mca):
    return {
        'serial': mca.SerialNumber, 'preset': mca.PresetTime,
        'battery': mca.BatteryStatus, 'real': mca.RealTime, 'live': mca.LiveTime,
        'threshold': mca.Threshold, 'flags': mca.flags, 'resolution': mca.ADCResolution,
        'live_timer': mca.isLive, 'acquiring': mca.isRunning,
        'protected': mca.isProtected, 'nicad': mca.isNiCad,
        'backup_bad': mca.isBackupBatteryBad,
    }


# ---------------------------------------------------------------- recorded statuses

@pytest.mark.parametrize('rec', RECORDED, ids=[r['raw'][-6:] for r in RECORDED])
def test_recorded_status(mca, rec):
    assert mca.UpdateStatusFromData(bytearray.fromhex(rec['raw']), hasSerialNumber=True) == 0
    got = parsed(mca)
    for key, value in rec['expected'].items():
        if key == 'group':
            continue                    # not parsed yet
        assert got[key] == pytest.approx(value), key


# ---------------------------------------------------------------- fields

def test_serial_number_variant(mca):
    mca.UpdateStatusFromData(make_status(field=(4321 << 16) | 5), hasSerialNumber=True)
    assert mca.SerialNumber == 4321
    assert mca.LastDataCheckSum == 0


def test_data_checksum_variant_keeps_full_32_bits(mca):
    mca.SerialNumber = 1224
    mca.UpdateStatusFromData(make_status(field=0x0007F800), hasSerialNumber=False)
    assert mca.LastDataCheckSum == 0x0007F800
    assert mca.SerialNumber == 1224


def test_preset_and_battery(mca):
    mca.UpdateStatusFromData(make_status(preset=0x123456, battery=77), True)
    assert mca.PresetTime == 0x123456
    assert mca.BatteryStatus == 77


@pytest.mark.parametrize('secs, f75, expected', [
    (0, 75, 0.0), (10, 75, 10.0), (9, 6, 9 + 69 / 75), (0, 1, 74 / 75),
    (2**24 - 1, 75, 2**24 - 1)])
def test_times(mca, secs, f75, expected):
    mca.UpdateStatusFromData(make_status(real=(secs, f75), live=(secs, f75)), True)
    assert mca.RealTime == pytest.approx(expected)
    assert mca.LiveTime == pytest.approx(expected)


def test_threshold(mca):
    mca.UpdateStatusFromData(make_status(threshold_word=0x1234), True)
    assert mca.Threshold == 0x1234


def test_threshold_excludes_absolute_peak_bit(mca):
    mca.UpdateStatusFromData(make_status(threshold_word=0x8000 | 64), True)
    assert mca.Threshold == 64


@pytest.mark.parametrize('code', range(7))
def test_resolution_codes(mca, code):
    mca.UpdateStatusFromData(make_status(flags=code), True)
    assert mca.ADCResolution == 16384 >> code


@pytest.mark.parametrize('bit, attr', [
    (3, 'isLive'), (4, 'isRunning'), (5, 'isProtected'), (6, 'isNiCad'),
    (7, 'isBackupBatteryBad')])
def test_flag_bits(mca, bit, attr):
    mca.UpdateStatusFromData(make_status(flags=0x04 | (1 << bit)), True)
    assert getattr(mca, attr) is True
    assert mca.flags == 0x04 | (1 << bit)
    mca.UpdateStatusFromData(make_status(flags=0x04), True)
    assert getattr(mca, attr) is False


# ---------------------------------------------------------------- checksum

def test_bad_checksum_rejected_and_counted(mca):
    mca.UpdateStatusFromData(make_status(preset=5), True)
    before = parsed(mca)
    assert mca.UpdateStatusFromData(make_status(preset=9, good_checksum=False), True) == 1
    assert mca.StatusChecksumErrors == 1
    assert parsed(mca) == before


# ---------------------------------------------------------------- against the simulator

def test_simulator_statuses_round_trip(mca, sim):
    rng = random.Random(1)
    for _ in range(200):
        sim.preset = rng.randrange(2**24)
        sim.battery = rng.randrange(256)
        sim.real[0] = rng.uniform(0, 1e5)
        sim.live[0] = rng.uniform(0, sim.real[0])
        sim.threshold_word = rng.randrange(2**15)
        sim.flags = rng.randrange(256) & ~0x07 | rng.randrange(7)
        assert mca.UpdateStatusFromData(bytearray(sim.status(sim._sn_field())), True) == 0
        assert mca.SerialNumber == 1224
        assert mca.PresetTime == sim.preset
        assert mca.BatteryStatus == sim.battery
        assert mca.RealTime == pytest.approx(sim.real[0], abs=1 / 75)
        assert mca.LiveTime == pytest.approx(sim.live[0], abs=1 / 75)
        assert mca.Threshold == sim.threshold_word
        assert mca.ADCResolution == sim.resolution
        assert mca.flags == sim.flags
