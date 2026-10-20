"""The state-changing scenarios (tests/state.py) on the real MCA.

--hw-state: settings only (threshold, preset), in the scratch group, restored.
--hw-delete: also acquisition and DELETE in the scratch group.
--hw-any-memory: also resolution changes, which write into other groups' parameter
  channels (I-27). Only on an MCA with nothing to keep.
--hw-slow: power-down on battery and recovery (~7 min; the MCA must be on battery).
"""

import pytest

import state


@pytest.mark.timeout(300)       # includes the session's memory snapshot (~1 min)
@pytest.mark.parametrize('scenario', state.SCENARIOS)
def test_state(connected, oracle, scratch, scenario):
    scenario(connected, oracle, scratch)


@pytest.mark.hw_slow
@pytest.mark.timeout(600)
def test_power_down_and_recover(connected, oracle):
    state.power_down_and_recover(connected, oracle, idle=360)
