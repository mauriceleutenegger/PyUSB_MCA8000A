"""I-21 steps 1-2: try to unlock an ALREADY-LOCKED MCA with lock number 0.

DANGER: sends SET LOCK (75h). Believed (unconfirmed) to wipe the MCA's data.
Only for the unit that is already locked. Refuses to run unless:
  --serial matches the connected MCA's serial number, AND
  the MCA reports the protected flag (status flags bit 5), AND
  --i-understand-this-may-wipe-data is given.
It never sets a non-zero lock.

Records: status before/after, whether the protected flag clears, and whether data
(lower words of the first 1024 channels + top channels) changed.
"""

import time

from common import (make_parser, connect, send, with_checksum, block_command, block_read,
                    words, prompt_status_retry, Report, HWError)


def snapshot(mca, R):
    lo = words(block_read(mca, block_command(0x40, 0, min(R, 1024)))[1])
    top = words(block_read(mca, block_command(0x40, R - 8, 8))[1])
    return {'sum_first_1024_lower': sum(lo), 'top8_lower': top}


def main():
    p = make_parser(__doc__)
    p.add_argument('--serial', type=int, required=True, help='serial number of the locked unit')
    p.add_argument('--i-understand-this-may-wipe-data', dest='confirm', action='store_true')
    p.add_argument('--wait', type=float, default=4.0, help='seconds to wait after the command')
    args = p.parse_args()
    if not args.confirm:
        raise SystemExit('Refusing: pass --i-understand-this-may-wipe-data')
    rep = Report('lock_zero', args)
    try:
        mca, st = connect(args)
        if st['serial'] != args.serial:
            raise HWError('connected MCA is S/N {}, not {}'.format(st['serial'], args.serial))
        if not st['protected']:
            raise HWError('MCA is not locked (protected flag clear); refusing to send SET LOCK')
        rep.info('status_before', st)
        try:
            rep.info('data_before', snapshot(mca, st['resolution']))
        except HWError as e:
            rep.result('read data while locked', 'INFO', 'failed: {}'.format(e))
        send(mca, with_checksum(0x75, 0, 0, 1))              # SET LOCK 0
        time.sleep(args.wait)
        st2 = prompt_status_retry(mca, tries=10)
        if st2 is None:
            raise HWError('no status after SET LOCK 0')
        rep.info('status_after', st2)
        rep.result('protected flag cleared by lock 0', 'PASS' if not st2['protected'] else 'FAIL')
        try:
            after = snapshot(mca, st2['resolution'])
            rep.info('data_after', after)
            rep.result('data after lock 0', 'INFO', str(after))
        except HWError as e:
            rep.result('read data after lock 0', 'INFO', 'failed: {}'.format(e))
    except HWError as e:
        rep.result('error', 'FAIL', str(e))
    rep.save()


if __name__ == '__main__':
    main()
