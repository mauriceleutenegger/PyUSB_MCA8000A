"""Output and error paths (I-4, I-5)."""

import pytest

import MCA8000A as lib


@pytest.mark.xfail(reason='I-5: SendCommand prints on every command', strict=True)
def test_send_command_is_silent(connected, capsys):
    capsys.readouterr()
    assert connected.SendCommand(lib.Command_PresetTime(1)) == 0
    assert capsys.readouterr().out == ''


@pytest.mark.xfail(reason='I-5: SetADCResolution prints debug values', strict=True)
def test_set_resolution_is_silent(connected, capsys, monkeypatch):
    monkeypatch.setattr(connected, 'SendCommand', lambda command: 0)   # isolate its prints
    capsys.readouterr()
    connected.SetADCResolution(1024)
    assert capsys.readouterr().out == ''


@pytest.mark.xfail(reason='I-5: Initialize prints "Setting baud rate"', strict=True)
def test_initialize_is_silent(mca, capsys, monkeypatch):
    monkeypatch.setattr(mca, 'SendCommand', lambda command, n_retries=10: 0)
    monkeypatch.setattr(mca, 'SetBaudRate', lambda baudrate: 0)
    mca.Initialize(baudrate=115200)
    assert capsys.readouterr().out == ''


def test_status_reads_are_silent(connected, capsys):
    capsys.readouterr()
    assert connected.ReceiveStatusFromPrompt() == 0
    assert capsys.readouterr().out == ''


def test_receive_status_checksum(connected, sim):
    # unused by the library; returns the DataChkSum field of a status already in flight
    connected.PromptForStatus()
    stat, value = connected.ReceiveStatusCheckSum()
    assert stat == 0 and value == (1224 << 16)


@pytest.mark.xfail(reason='I-4: error path uses undefined NULL', strict=True)
def test_receive_status_checksum_error_path(connected, sim):
    assert connected.ReceiveStatusCheckSum() == (1, None)
