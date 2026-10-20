"""The contract scenarios (tests/contract.py) on the real MCA. Read-only."""

import pytest

import contract
from mcasim.host import Host


@pytest.mark.parametrize('scenario, baud', contract.SCENARIOS, ids=contract.IDS)
def test_contract(port, scenario, baud):
    contract.run(scenario, baud, Host.on_port(port))
