"""The contract scenarios (tests/contract.py) on the simulator."""

import pytest

import contract


@pytest.mark.parametrize('scenario, baud', contract.SCENARIOS, ids=contract.IDS)
def test_contract(host, scenario, baud):
    contract.run(scenario, baud, host)


def test_lock_guard_refuses_set_lock():
    from lockguard import GuardedSerial
    port = GuardedSerial()                     # not opened
    with pytest.raises(pytest.exit.Exception, match='SET LOCK'):
        port.write(bytes([0x75, 0, 0, 1, 0x76]))


def test_lock_guard_lets_other_commands_through():
    from lockguard import check_write
    import MCA8000A
    for cmd in (MCA8000A.Command_SendData(0, 1), MCA8000A.Command_Control(0, 0x75),
                MCA8000A.Command_PresetTime(0x75)):
        check_write(cmd)
    with pytest.raises(pytest.exit.Exception):
        check_write(MCA8000A.Command_MCALock(0))
