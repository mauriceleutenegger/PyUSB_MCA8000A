"""Control methods: start/stop, threshold, preset, delete. docs/protocol.md §10."""

import pytest


def test_set_threshold(connected, sim):
    assert connected.SetThreshold(123) == 0
    assert sim.threshold_word == 123
    assert connected.Threshold == 123


def test_set_preset_time(connected, sim):
    assert connected.SetPresetTime(3600) == 0
    assert sim.preset == 3600
    assert connected.PresetTime == 3600


def test_start_and_stop(connected, sim, clock):
    assert connected.StartAcquisition() == 0
    assert sim.acquiring and connected.isRunning
    clock.sleep(5.0)
    assert connected.StopAcquisition() == 0
    assert not sim.acquiring and not connected.isRunning
    assert 5.0 < connected.RealTime < 5.5          # 1 s wait + 5 s - 0.97 s start delay


def test_start_keeps_other_settings(connected, sim):
    sim.flags = 0x08 | 0x03                        # live timer, 2048
    sim.threshold_word = 77
    connected.ReceiveStatusFromPrompt()
    connected.StartAcquisition()
    assert sim.flags == 0x08 | 0x10 | 0x03
    assert sim.threshold_word == 77


def test_preset_stops_acquisition(connected, sim, clock):
    connected.SetPresetTime(3)
    connected.StartAcquisition()
    clock.sleep(5.0)
    assert connected.ReceiveStatusFromPrompt() == 0
    assert not connected.isRunning
    assert connected.RealTime == pytest.approx(3.0, abs=1 / 75)


def test_set_threshold_after_preset_stop_sends_stopped_flags(connected, sim, clock):
    connected.SetPresetTime(3)
    connected.StartAcquisition()                   # cached flags: acquiring
    clock.sleep(5.0)                               # preset stops it; no status read since
    connected.SetThreshold(50)
    # Check what was sent, not the MCA's reaction: whether a start with the preset
    # already reached restarts acquisition is unknown (the sim assumes it doesn't).
    control = [c for _, c, ok in sim.commands if c[0] == 0x01]
    assert not control[-1][1] & 0x10


def test_stop_keeps_settings_changed_elsewhere(connected, sim):
    connected.StartAcquisition()
    sim.flags |= 0x08                              # e.g. set from another program
    connected.StopAcquisition()
    assert sim.flags & 0x08


def test_stop_with_absolute_peak_mode(connected, sim):
    sim.threshold_word = 0x8000 | 64
    connected.ReceiveStatusFromPrompt()            # (without it, this would test I-14)
    connected.StartAcquisition()
    assert connected.StopAcquisition() == 0
    assert sim.threshold_word == 0x8000 | 64      # mode and threshold kept


@pytest.mark.parametrize('method, data_cleared, time_cleared', [
    ('DeleteDataAndTime', True, True), ('DeleteData', True, False),
    ('DeleteTime', False, True)])
def test_delete(connected, sim, method, data_cleared, time_cleared):
    sim.memory[:2048] = 9
    sim.real[0] = sim.live[0] = 4.0
    assert getattr(connected, method)() == 0
    assert (sim.memory[:1024].sum() == 0) == data_cleared
    assert sim.memory[1024] == 9                   # group 1 untouched (1024 channels)
    assert (sim.real[0] == 0) == time_cleared
    assert connected.RealTime == pytest.approx(0.0 if time_cleared else 4.0)


@pytest.mark.parametrize('busy', [0.1, 0.35])
def test_delete_while_mca_briefly_busy(connected, sim, busy):
    sim.delete_busy = lambda R: busy
    assert connected.DeleteDataAndTime() == 0


@pytest.mark.xfail(reason='I-18: one 0.2 s prompt after 0.2 s; DTP allows up to 2 s',
                   strict=True)
def test_delete_at_16k_with_2s_busy(connected, sim):
    sim.flags &= ~7                                # 16384 channels
    connected.ReceiveStatusFromPrompt()
    sim.delete_busy = lambda R: 2.0
    assert connected.DeleteDataAndTime() == 0


def test_commands_fail_cleanly_when_mca_off(connected, sim):
    sim.power_down_now()
    assert connected.SetThreshold(5) == 1
    assert connected.StartAcquisition() == 1
    assert connected.DeleteData() == 1
