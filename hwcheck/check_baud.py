"""Baud rate setting (the DTR delay and related observations).

Checks, for each rate:
  1. Reopening the port puts the MCA back to 4800 baud.
  2. SetBaudRate (as in MCA8000A.py: no DTR->RTS delay) switches it, repeatedly.
And separately:
  3. A baud change sent while the MCA is *not* at 4800 leaves it at 4800.

Observed 2026-10-13 (S/N 1224, FTDI adapter, macOS): 9600-57600 work; 115200 did
not until SetBaudRate ended the data stream that the baud command starts (fixed
2026-10-15, see docs/protocol.md section 5). Read-only.
"""

import time

from common import (make_parser, quiet, prompt_status, reopen, Report, HWError, lib,
                    DEFAULT_PORT)

ALL_RATES = (4800, 9600, 19200, 38400, 57600, 115200)


def where(mca):
    """Return the host baud rate at which the MCA answers a status prompt, or None."""
    for b in ALL_RATES:
        mca.serial_connection.baudrate = b
        if prompt_status(mca) is not None:
            return b
    return None


def main():
    p = make_parser(__doc__)
    p.add_argument('--rates', default='9600,19200,38400,57600,115200')
    p.add_argument('--repeat', type=int, default=3)
    args = p.parse_args()
    rep = Report('baud', args)
    try:
        with quiet():
            mca = lib.MCA8000A(args.port)
        if args.power_on:
            with quiet():
                mca.PowerOn()
        rates = [int(x) for x in args.rates.split(',')]
        for rate in rates:
            ok_count = 0
            for i in range(args.repeat):
                reopen(mca)
                at = where(mca)
                if at != 4800:
                    rep.result('reopen -> 4800 (before {} #{})'.format(rate, i), 'FAIL',
                               'MCA at {}'.format(at))
                    continue
                mca.serial_connection.baudrate = 4800
                t0 = time.monotonic()
                with quiet():
                    mca.SetBaudRate(rate)
                mca.serial_connection.baudrate = rate
                ok = prompt_status(mca) is not None
                ok_count += ok
                rep.result('4800 -> {} #{}'.format(rate, i), 'PASS' if ok else 'FAIL',
                           '{:.2f} s'.format(time.monotonic() - t0) if ok
                           else 'afterwards answering at {}'.format(where(mca)))
            rep.result('SetBaudRate({}) with no DTR delay: {}/{}'.format(rate, ok_count, args.repeat),
                       'PASS' if ok_count == args.repeat else 'FAIL')

        # 3. baud change requested from a rate other than 4800
        working = [r for r in rates if r != 4800]
        if len(working) >= 2:
            a, b = working[0], working[-1]
            reopen(mca)
            with quiet():
                mca.SetBaudRate(a)
            mca.serial_connection.baudrate = a
            if prompt_status(mca) is None:
                rep.result('setup for non-4800 change', 'INCONCLUSIVE', 'could not reach {}'.format(a))
            else:
                with quiet():
                    mca.SetBaudRate(b)
                at = where(mca)
                rep.result('change {} -> {} requested at {} baud'.format(a, b, a), 'INFO',
                           'MCA ends up at {}'.format(at))
                rep.result('baud change from non-4800 rate falls back to 4800',
                           'PASS' if at == 4800 else 'FAIL')
        reopen(mca)
    except HWError as e:
        rep.result('error', 'FAIL', str(e))
    rep.save()


if __name__ == '__main__':
    main()
