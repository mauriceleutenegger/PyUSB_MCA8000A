"""I-20: what happens when the MCA powers itself down, and does it recover?

Battery operation only. Connects, then leaves the MCA completely idle for --idle
seconds (default 300; the DTP says it powers down after ~3 minutes). Then:
  a) prompts for status by hand (does CTS still toggle? any bytes?)
  b) calls the library's ReceiveStatusFromPrompt and SendCommand: return values,
     durations, exceptions (looking for hangs, crashes, or false success)
  c) reopens the port and looks for it at 4800
  d) runs PowerOn + connect and checks the status
Does not change MCA settings or data. Takes ~6 minutes.
"""

import time

from common import (make_parser, connect, quiet, prompt_status, reopen, block_command,
                    Report, HWError)


def timed(fn):
    t0 = time.monotonic()
    try:
        with quiet():
            r = fn()
        return {'return': r, 'seconds': round(time.monotonic() - t0, 2)}
    except Exception as e:  # report, don't crash: crashes are a result here
        return {'exception': repr(e), 'seconds': round(time.monotonic() - t0, 2)}


def main():
    p = make_parser(__doc__)
    p.add_argument('--idle', type=int, default=300, help='idle seconds (default %(default)s)')
    args = p.parse_args()
    rep = Report('powerdown', args)
    try:
        mca, st = connect(args)
        if st['battery'] == 0:
            rep.result('battery check', 'INCONCLUSIVE', 'on external power; auto power-down only happens on battery')
            rep.save()
            return
        rep.info('status_before', st)
        print('Idle for {} s; do not touch the MCA...'.format(args.idle))
        t_end = time.monotonic() + args.idle
        while time.monotonic() < t_end:
            time.sleep(min(30, max(0, t_end - time.monotonic())))
            print('  {:.0f} s left'.format(max(0, t_end - time.monotonic())), flush=True)

        port = mca.serial_connection
        rep.result('lines after idle', 'INFO', 'cts={} dsr={}'.format(port.cts, port.dsr))
        # a) hand prompt
        t0 = time.monotonic()
        with quiet():
            pstat = mca.PromptForStatus()
        st_a = prompt_status(mca)
        rep.result('a) status prompt after idle', 'INFO',
                   'CTS toggled={} status={} ({:.2f} s)'.format(
                       pstat == 0, 'received' if st_a else 'none', time.monotonic() - t0))
        powered_down = st_a is None
        rep.result('MCA powered down after {} s idle'.format(args.idle),
                   'PASS' if powered_down else 'INFO',
                   '' if powered_down else 'still answering (did not power down)')
        # b) library calls
        before = (mca.RealTime, mca.LiveTime, mca.SerialNumber)
        r = timed(mca.ReceiveStatusFromPrompt)
        after = (mca.RealTime, mca.LiveTime, mca.SerialNumber)
        rep.result('b) ReceiveStatusFromPrompt', 'INFO', '{} cached status changed={}'.format(r, before != after))
        if powered_down and r.get('return') == 0:
            rep.result('ReceiveStatusFromPrompt reports success with a powered-down MCA', 'FAIL',
                       'false success (see issues I-22)')
        r = timed(lambda: mca.SendCommand(block_command(0x40, 0, 1)))
        rep.result('b) SendCommand', 'INFO', str(r))
        # c) reopen at 4800
        reopen(mca)
        st_c = prompt_status(mca)
        rep.result('c) after port reopen, status at 4800', 'INFO', 'received' if st_c else 'none')
        # d) power on + connect
        args.power_on = True
        mca.serial_connection.close()
        try:
            mca2, st_d = connect(args)
            rep.result('d) PowerOn + connect recovers', 'PASS',
                       'real time {} -> {}'.format(st['real'], st_d['real']))
            rep.info('status_after', st_d)
        except HWError as e:
            rep.result('d) PowerOn + connect recovers', 'FAIL', str(e))
    except HWError as e:
        rep.result('error', 'FAIL', str(e))
    rep.save()


if __name__ == '__main__':
    main()
