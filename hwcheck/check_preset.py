"""Preset time semantics (protocol.md §16 Q9) and the stale-flags issue (I-14).

CHANGES MCA STATE. Requires --yes. Works in a scratch memory group (default: the
highest group at the current resolution) whose time it clears; the original group
and preset time are restored at the end. Data in the scratch group is not deleted,
but acquisition runs in it for a few seconds (with no detector input, no counts).

  1. preset 0, start, wait --run s: still acquiring? (expect yes: "run until stopped")
  2. preset 3 s, clear time, start, wait 3 s + margin: stopped by itself, real ~3 s?
     Also reports whether the library's cached flags still say "acquiring" (I-14).

Commands are built here with fresh read-modify-write of the flags, not with the
library's cached flags.
"""

import time

from common import (make_parser, connect, send, with_checksum, prompt_status_retry,
                    Report, HWError)


def control(mca, st, acquire):
    flags = (st['flags'] | 0x10) if acquire else (st['flags'] & ~0x10 & 0xFF)
    word = st['threshold'] | (st['absolute_peak'] << 15)
    send(mca, with_checksum(0x01, flags, word & 0xFF, word >> 8))


def fresh(mca):
    st = prompt_status_retry(mca)
    if st is None:
        raise HWError('lost status')
    return st


def main():
    p = make_parser(__doc__)
    p.add_argument('--yes', action='store_true', help='confirm: this changes MCA state')
    p.add_argument('--group', type=int, help='scratch group (default: highest group)')
    p.add_argument('--run', type=float, default=5.0, help='seconds to run with preset 0')
    args = p.parse_args()
    if not args.yes:
        raise SystemExit('This check changes MCA state (preset, group, clears time in a scratch '
                         'group, starts/stops acquisition). Re-run with --yes.')
    rep = Report('preset', args)
    mca = None
    orig = None
    try:
        mca, orig = connect(args)
        if orig['acquiring']:
            raise HWError('MCA is acquiring; stop it first')
        R = orig['resolution']
        group = args.group if args.group is not None else 32768 // R - 1
        if not 0 <= group < 32768 // R:
            raise HWError('group {} out of range at resolution {}'.format(group, R))
        rep.info('original', orig)
        rep.info('scratch_group', group)
        clear_wait = R / 8 / 1000 + 0.256   # DLL ClearDataAndTime

        send(mca, with_checksum(0x11, 0, group, 1))         # SET GROUP
        st = fresh(mca)
        rep.result('set scratch group {}'.format(group), 'PASS' if st['group'] == group else 'FAIL',
                   'status group {}'.format(st['group']))

        # 1. preset 0
        send(mca, with_checksum(0x05, 0, 1, 1))              # DELETE time only
        time.sleep(clear_wait)
        send(mca, with_checksum(0x02, 0, 0, 0))              # PRESET 0
        st = fresh(mca)
        control(mca, st, True)
        time.sleep(args.run)
        st = fresh(mca)
        rep.result('preset 0: still acquiring after {} s'.format(args.run),
                   'PASS' if st['acquiring'] else 'FAIL',
                   'acquiring={} real={:.2f} live={:.2f}'.format(st['acquiring'], st['real'], st['live']))
        control(mca, st, False)
        st = fresh(mca)
        rep.result('stop', 'PASS' if not st['acquiring'] else 'FAIL')

        # 2. preset 3 s
        send(mca, with_checksum(0x05, 0, 1, 1))
        time.sleep(clear_wait)
        send(mca, with_checksum(0x02, 3, 0, 0))              # PRESET 3
        st = fresh(mca)
        control(mca, st, True)
        st = fresh(mca)                                      # library cache now says acquiring
        time.sleep(3 + 2)
        cached = bool(mca.flags & 0x10)
        st = fresh(mca)
        timer = 'live' if st['live_timer'] else 'real'
        rep.result('preset 3 s ({} timer): stopped by itself'.format(timer),
                   'PASS' if not st['acquiring'] else 'FAIL',
                   'real={:.2f} live={:.2f}'.format(st['real'], st['live']))
        rep.result('I-14: library cached flags said acquiring after the preset stop', 'INFO',
                   'cached acquiring={} actual={}'.format(cached, st['acquiring']))
    except HWError as e:
        rep.result('error', 'FAIL', str(e))
    finally:
        if mca is not None and orig is not None:
            try:
                st = fresh(mca)
                if st['acquiring']:
                    control(mca, st, False)
                p = orig['preset']
                send(mca, with_checksum(0x02, p & 0xFF, (p >> 8) & 0xFF, p >> 16))
                send(mca, with_checksum(0x11, 0, orig['group'], 1))
                st = fresh(mca)
                ok = st['group'] == orig['group'] and st['preset'] == orig['preset'] and not st['acquiring']
                rep.result('restore group {} and preset {}'.format(orig['group'], orig['preset']),
                           'PASS' if ok else 'FAIL')
            except HWError as e:
                rep.result('restore', 'FAIL', str(e))
    rep.save()


if __name__ == '__main__':
    main()
