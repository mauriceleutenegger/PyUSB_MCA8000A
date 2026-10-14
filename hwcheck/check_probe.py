"""Connect and print the MCA status. Read-only (apart from setting the baud rate).

Run this first: if it fails, the other checks will too.
"""

from common import make_parser, connect, Report, HWError


def main():
    args = make_parser(__doc__).parse_args()
    rep = Report('probe', args)
    try:
        mca, st = connect(args)
    except HWError as e:
        rep.result('connect', 'FAIL', str(e))
        rep.save()
        return 1
    rep.info('status', st)
    for k in ('serial', 'group', 'resolution', 'acquiring', 'live_timer', 'preset',
              'real', 'live', 'threshold', 'absolute_peak', 'protected', 'battery'):
        print('  {:14s} {}'.format(k, st[k]))
    rep.result('connect + status', 'PASS')
    rep.save()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
