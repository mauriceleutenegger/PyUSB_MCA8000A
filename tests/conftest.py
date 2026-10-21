"""Test options and the tier switch.

`pytest` alone runs tier 1 only (no serial port is touched). Hardware tests (tier 2,
everything under tests/hw/ or marked `hw`) run only with --hw. The riskier groups need
their own flag on top of --hw. See tests/README.md.
"""

import os
import pathlib

import pytest

HW_DIR = pathlib.Path(__file__).parent / 'hw'


def pytest_addoption(parser):
    g = parser.getgroup('mca', 'MCA8000A hardware tests (tier 2)')
    g.addoption('--hw', action='store_true',
                help='run hardware tests (read-only unless further flags are given)')
    g.addoption('--mca-port', default=os.environ.get('MCA_PORT'),
                help='serial device of the MCA, as --mca-port=/dev/cu.usbserial-XXX or '
                     'just the name cu.usbserial-XXX (default: $MCA_PORT). With a space '
                     'and a full path, pytest takes /dev as its root and fails.')
    g.addoption('--mca-serial', type=int, default=None,
                help='expected MCA serial number; abort if another unit answers')
    g.addoption('--power-on', action='store_true',
                help='run the remote power-on sequence before connecting')
    g.addoption('--hw-state', action='store_true',
                help='also run tests that change MCA settings (restored afterwards)')
    g.addoption('--hw-delete', action='store_true',
                help='also run tests that acquire in a scratch group and delete its data '
                     'and time (implies --hw-state)')
    g.addoption('--hw-any-memory', action='store_true',
                help='also run tests that may overwrite memory outside the scratch group, '
                     'e.g. the parameter channels of other groups. Only on an MCA '
                     'with nothing to keep. Implies --hw-state')
    g.addoption('--hw-slow', action='store_true',
                help='also run slow tests (minutes; e.g. power-down on battery)')


def pytest_configure(config):
    port = config.getoption('--mca-port')
    if config.getoption('--hw') and not port:
        raise pytest.UsageError('--hw needs --mca-port or $MCA_PORT')
    if port and '/' not in port:
        config.option.mca_port = '/dev/' + port


def pytest_collection_modifyitems(config, items):
    hw = config.getoption('--hw')
    allowed = {
        'hw_state': (config.getoption('--hw-state') or config.getoption('--hw-delete')
                     or config.getoption('--hw-any-memory')),
        'hw_any_memory': config.getoption('--hw-any-memory'),
        'hw_delete': config.getoption('--hw-delete'),
        'hw_slow': config.getoption('--hw-slow'),
    }
    for item in items:
        if HW_DIR in pathlib.Path(item.fspath).parents:
            item.add_marker(pytest.mark.hw)
        if 'hw' not in item.keywords:
            continue
        if not hw:
            item.add_marker(pytest.mark.skip(reason='hardware test; enable with --hw'))
            continue
        if item.get_closest_marker('timeout') is None:
            # the first test also pays for the session setup (a memory snapshot, ~1 min,
            # in a state session)
            item.add_marker(pytest.mark.timeout(300))
        for marker, ok in allowed.items():
            if marker in item.keywords and not ok:
                flag = '--' + marker.replace('_', '-')
                item.add_marker(pytest.mark.skip(reason='needs ' + flag))
