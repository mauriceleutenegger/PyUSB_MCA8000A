"""Contract scenarios: the same checks run on the simulator (tier 1, unit/test_contract_sim.py)
and on the real MCA (tier 2, hw/test_contract_hw.py).

Each scenario drives the MCA with the hand-driven Host (mcasim/host.py), not the library,
and asserts what docs/protocol.md says. On hardware a failure means the documentation or
the simulator's model is wrong; on the sim it means the model drifted from what hardware
showed. All scenarios are read-only: status prompts, block data requests, the baud command
and port reopening.
"""

from mcasim.host import STATUS_SIZE, block_command, parse_status


def valid_status(raw):
    assert len(raw) == STATUS_SIZE, raw.hex()
    st = parse_status(raw)
    assert st['checksum_ok'], raw.hex()
    return st


def first_status_after_open(host):
    """§5: opening the port counts as a prompt; afterwards one prompt gives one status."""
    valid_status(host.status())
    valid_status(host.status())


def accepted_command_toggles_twice(host):
    """§6, I-12: a valid command gets CTS toggle #2, then status + data."""
    assert host.command(block_command(0x40, 0, 1)) == (True, True)
    raw = host.collect()
    assert len(raw) == STATUS_SIZE + 2
    valid_status(raw[:STATUS_SIZE])


def rejected_command_acts_as_prompt(host):
    """§6, I-12: a wrong checksum gets no toggle #2, and only a status comes back."""
    cmd = bytearray(block_command(0x40, 0, 1))
    cmd[4] -= 1
    assert host.command(bytes(cmd)) == (True, False)
    valid_status(host.collect())


def short_command_rejected(host):
    """§6: 4 bytes are not a command."""
    assert host.command(block_command(0x40, 0, 1)[:4]) == (True, False)
    valid_status(host.collect())


def checksum_above_sum_accepted(host):
    """§6, I-23: the firmware accepts a checksum byte >= the true sum (mod 256)."""
    cmd = bytearray(block_command(0x40, 0, 1))
    cmd[4] += 1
    assert host.command(bytes(cmd)) == (True, True)
    assert len(host.collect()) == STATUS_SIZE + 2


def block_sizes(host):
    """§9: n words requested -> status + 2n bytes (DLL layout), for small and full blocks."""
    for n in (1, 7, 64, 1024):
        st, data = host.read_block(0x40, 0, n)
        assert st['checksum_ok'] and len(data) == 2 * n, n


def data_checksum_in_next_status(host):
    """§8: a 40h status carries the byte sum of the previous transfer's data."""
    _, data = host.read_block(0x40, 0, 256)
    st, _ = host.read_block(0x40, 0, 1)
    assert st['field'] == sum(data)          # < 2^16 here; mod 2^16 vs 32 bit is I-11


def prompt_clears_data_checksum(host):
    """§8: a status prompt between two transfers clears the pending checksum."""
    _, data = host.read_block(0x40, 0, 256)
    valid_status(host.status())
    st, _ = host.read_block(0x40, 0, 1)
    assert st['field'] == 0, sum(data)


def reopen_resets_to_4800(host):
    """§5, I-25: after a switch to 115200, reopening the port puts the MCA back to 4800."""
    host.connect(115200)
    valid_status(host.status())
    host.port.close()
    host.port.open()
    host.port.baudrate = 4800
    host._settle()
    valid_status(host.status())


# (name, scenario, baud the host switches to first)
SCENARIOS = [
    (first_status_after_open, 4800),
    (first_status_after_open, 115200),
    (accepted_command_toggles_twice, 115200),
    (rejected_command_acts_as_prompt, 115200),
    (short_command_rejected, 115200),
    (checksum_above_sum_accepted, 115200),
    (block_sizes, 115200),
    (block_sizes, 4800),
    (data_checksum_in_next_status, 115200),
    (prompt_clears_data_checksum, 115200),
    (reopen_resets_to_4800, 4800),
]
IDS = ['{}@{}'.format(f.__name__, b) for f, b in SCENARIOS]


def run(scenario, baud, host):
    assert host.connect(baud) == (True, True)
    scenario(host)
