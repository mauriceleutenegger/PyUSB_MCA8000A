"""Capture test fixtures from a real MCA8000A. Read-only.

Sends only status prompts, block data requests (40h/42h/50h) and the baud command.
Saves the raw bytes, with provenance, to tests/data/capture-<serial>.json. Unit tests use
them as real responses for parsing and decoding (plan D4).

    python3 tests/data/capture.py [--port /dev/cu.usbserial-XXX] [--power-on]

Uses hwcheck/common.py for the handshakes and its own parsing of the bytes.
"""

import datetime
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'hwcheck'))
import common  # noqa: E402
from common import block_command, block_read, parse_status, prompt_status  # noqa: E402

BAUD = 115200

# (code, start channel, words). Each transfer's status carries the data checksum of the
# transfer before it (40h/42h), or S/N + group (50h, and after a prompt).
TRANSFERS = [
    (0x40, 0, 1024), (0x40, 1024, 1024), (0x42, 0, 1024), (0x42, 1024, 1024),
    (0x40, 0, 1), 'prompt', (0x50, 0, 4), (0x40, 2040, 16), (0x42, 2040, 16), (0x40, 0, 1),
]


def main():
    p = common.make_parser(__doc__)
    args = p.parse_args()
    args.baud = BAUD
    mca, st = common.connect(args)
    out = {
        'about': 'Raw responses from a real MCA8000A (USB), captured by tests/data/capture.py. '
                 'Read-only capture; hex strings.',
        'captured': datetime.datetime.now().isoformat(timespec='seconds'),
        'port': args.port, 'baud': BAUD, 'serial': st['serial'],
        'resolution': st['resolution'], 'group': st['group'],
        'statuses': [], 'transfers': [],
    }
    for _ in range(3):
        s = prompt_status(mca)
        if s is None:
            raise common.HWError('status prompt failed')
        out['statuses'].append(s['raw'])
        time.sleep(0.3)
    prev = None
    for t in TRANSFERS:
        if t == 'prompt':
            s = prompt_status(mca)
            out['transfers'].append({'prompt': s['raw']})
            prev = None
            continue
        code, start, n = t
        status, data = block_read(mca, block_command(code, start, n))
        if len(data) != 2 * n:
            raise common.HWError('{:02x} {} {}: got {} data bytes'.format(code, start, n,
                                                                          len(data)))
        entry = {'command': block_command(code, start, n).hex(), 'code': code,
                 'start': start, 'words': n, 'status': status['raw'], 'data': data.hex()}
        if prev is not None and code in (0x40, 0x42):
            entry['previous_sum'] = prev
            print('{:02x} start {:5} n {:4}: checksum field {} vs previous sum {}'.format(
                code, start, n, status['datachk'], prev))
        out['transfers'].append(entry)
        prev = sum(data)
    mca.serial_connection.close()
    path = os.path.join(HERE, 'capture-{}.json'.format(out['serial']))
    with open(path, 'w') as f:
        json.dump(out, f, indent=1)
    print('Saved', os.path.relpath(path))


if __name__ == '__main__':
    main()
