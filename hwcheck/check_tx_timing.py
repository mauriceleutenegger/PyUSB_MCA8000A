"""Retest the macOS FTDI driver bugs described in timing.md (docs/protocol.md §3).

Claimed bugs:
  A. out_waiting always returns 0, even before the bytes have been sent.
  B. reset_input_buffer() (purge RX) also discards output that hasn't been sent yet.

Run it once per driver: FTDI's VCP dext (/dev/cu.usbserial-A10KR3LP) and Apple's
AppleUSBFTDI (/dev/cu.usbserial-5). Never open both ports at once.

Part 1, handshake (default; MCA on; read-only). Sends a 1-word data request (40h) with the
USB handshake. CTS toggle #2 comes only when the MCA has received the complete, valid
command (issues I-12), so it shows whether the bytes really went out. Right after write():
  none        nothing (control)
  purge_rx    reset_input_buffer() at once                 -> tests bug B
  wait_purge  library's fixed wait, then reset_input_buffer()  (the current workaround)
  purge_tx    reset_output_buffer() at once  (discarding unsent output is legitimate here;
              shows whether a purge can catch the bytes at all)
  flush_purge flush() (tcdrain), then reset_input_buffer()  (a possible alternative)
  delay<ms>   wait <ms>, then reset_input_buffer()  (--modes; to measure the race window)
out_waiting is sampled right after write() and until toggle #2 -> bug A, as the library
would see it.

Part 2, bulk (--bulk; MCA unplugged from the cable, so the bytes go nowhere). Writes 5 s of
data, more than the driver buffers, and uses the blocking write() as the gauge: records
out_waiting and flush() while the buffer is full, and whether reset_input_buffer() /
reset_output_buffer() discard the pending output (see run_bulk). 57600 and 115200 only;
at 4800 the driver buffer holds more than 30 s. The script refuses to run --bulk if the MCA
answers a status prompt.
"""

import os
import statistics
import time

from common import make_parser, connect, Report, HWError, block_command, collect, \
    prompt_status, quiet, lib, STATUS_SIZE

MODES = ['none', 'purge_rx', 'wait_purge', 'purge_tx', 'flush_purge']


def handshake_trial(mca, mode, cmd):
    port = mca.serial_connection
    r = {'mode': mode}
    port.reset_input_buffer()
    mca.ResetRTS()
    lib.wait(0.001)
    mca.RememberCTS()
    mca.SetRTS()
    if mca.WaitForCTSFlip():
        mca.ResetRTS()
        r['toggle1'] = False
        return r
    r['toggle1'] = True
    t0 = time.perf_counter()
    port.write(cmd)
    t_w = time.perf_counter()
    ow = [(0.0, port.out_waiting)]
    if mode == 'purge_rx':
        port.reset_input_buffer()
    elif mode == 'wait_purge':
        lib.wait(len(cmd) * mca.bits_per_byte / port.baudrate + mca.send_padding)
        port.reset_input_buffer()
    elif mode == 'purge_tx':
        port.reset_output_buffer()
    elif mode.startswith('delay'):
        lib.wait(float(mode[5:]) / 1e3)
        port.reset_input_buffer()
    elif mode == 'flush_purge':
        port.flush()
        r['flush_ms'] = (time.perf_counter() - t_w) * 1e3
        port.reset_input_buffer()
    # wait for toggle #2, sampling out_waiting
    t_end = time.perf_counter() + 0.2
    toggle2 = None
    while time.perf_counter() < t_end:
        if mca.IsCTSFlipped():
            toggle2 = (time.perf_counter() - t0) * 1e3
            mca.RememberCTS()
            break
        n = port.out_waiting
        if n != ow[-1][1]:
            ow.append(((time.perf_counter() - t0) * 1e3, n))
    mca.ResetRTS()
    r['write_ms'] = (t_w - t0) * 1e3
    r['toggle2_ms'] = toggle2
    r['out_waiting'] = ow
    time.sleep(0.03)
    raw = collect(mca, idle=0.15, limit=2.0)
    r['rx_bytes'] = len(raw)
    r['ok'] = toggle2 is not None and len(raw) == STATUS_SIZE + 2
    return r


def rx_purge_trial(mca, which):
    """Get a status into the RX buffer, then purge with `which`; return in_waiting before/after
    and whether the bytes can still be read."""
    port = mca.serial_connection
    port.reset_input_buffer()
    with quiet():
        stat = mca.PromptForStatus()
    if stat:
        return {'which': which, 'prompt': False}
    t_end = time.perf_counter() + 1.0
    while port.in_waiting < STATUS_SIZE and time.perf_counter() < t_end:
        time.sleep(0.005)
    time.sleep(0.02)
    before = port.in_waiting
    if which == 'reset_input_buffer':
        port.reset_input_buffer()
    elif which == 'reset_output_buffer':
        port.reset_output_buffer()
    after = port.in_waiting
    time.sleep(0.05)
    left = len(collect(mca, idle=0.1, limit=1.0))
    return {'which': which, 'prompt': True, 'before': before, 'after': after, 'readable': left}


def run_rx_purge(mca, rep, trials):
    baud = mca.serial_connection.baudrate
    out = {}
    for which in ('none', 'reset_input_buffer', 'reset_output_buffer'):
        res = [rx_purge_trial(mca, which) for _ in range(trials)]
        out[which] = res
        ok = [r for r in res if r['prompt']]
        rep.result('{} baud RX purge with {}'.format(baud, which), 'INFO',
                   '{}/{} prompts; in_waiting before {}, after {}; bytes still readable {}'
                   .format(len(ok), trials, sorted({r['before'] for r in ok}),
                           sorted({r['after'] for r in ok}),
                           sorted({r['readable'] for r in ok})))
    rep.data.setdefault('rx_purge', {})[str(baud)] = out


def run_handshake(mca, rep, trials, modes=MODES):
    cmd = block_command(0x40, 0, 1)
    baud = mca.serial_connection.baudrate
    tx_ms = len(cmd) * mca.bits_per_byte / baud * 1e3
    print('\nHandshake test at {} baud: 5 bytes take {:.2f} ms on the wire.'.format(baud, tx_ms))
    summary = {}
    for mode in modes:
        res = []
        for i in range(trials):
            res.append(handshake_trial(mca, mode, cmd))
            time.sleep(0.02)
        t2 = [r['toggle2_ms'] for r in res if r.get('toggle2_ms') is not None]
        ow_max = max((n for r in res for _, n in r.get('out_waiting', [])), default=0)
        ow0 = [r['out_waiting'][0][1] for r in res if r.get('out_waiting')]
        s = {'trials': trials,
             'toggle1': sum(r['toggle1'] for r in res),
             'toggle2': len(t2),
             'ok': sum(r.get('ok', False) for r in res),
             'toggle2_ms_median': round(statistics.median(t2), 2) if t2 else None,
             'toggle2_ms_min': round(min(t2), 2) if t2 else None,
             'out_waiting_after_write': sorted(set(ow0)),
             'out_waiting_max': ow_max,
             'rx_bytes': sorted({r.get('rx_bytes') for r in res if 'rx_bytes' in r})}
        if mode == 'flush_purge':
            f = [r['flush_ms'] for r in res if 'flush_ms' in r]
            s['flush_ms_median'] = round(statistics.median(f), 2) if f else None
        summary[mode] = s
        rep.result('{} baud {}'.format(baud, mode), 'INFO',
                   'toggle2 {}/{}, ok {}/{}, toggle2 median {} ms, out_waiting after '
                   'write {}, max {}, rx {}{}'.format(
                       s['toggle2'], trials, s['ok'], trials, s['toggle2_ms_median'],
                       s['out_waiting_after_write'], ow_max, s['rx_bytes'],
                       ', flush median {} ms'.format(s['flush_ms_median'])
                       if 'flush_ms_median' in s else ''))
        rep.data.setdefault('raw', {})['{}-{}'.format(baud, mode)] = res
    rep.data.setdefault('handshake', {})[str(baud)] = summary
    # verdicts
    if 'none' not in summary or 'purge_rx' not in summary:
        return
    if summary['none']['ok'] < trials:
        rep.result('{} baud control'.format(baud), 'INCONCLUSIVE',
                   'control trials failed; MCA link unreliable')
        return
    b = summary['purge_rx']
    rep.result('{} baud bug B (purge RX discards unsent TX)'.format(baud),
               'FAIL' if b['toggle2'] < trials else 'PASS',
               'present: command lost in {}/{} trials'.format(trials - b['toggle2'], trials)
               if b['toggle2'] < trials else 'absent: command arrived in all {} trials'.format(trials))
    a_ow = max(summary[m]['out_waiting_max'] for m in modes)
    rep.result('{} baud bug A (out_waiting, 5-byte command)'.format(baud),
               'INFO', 'out_waiting max {} over all trials; toggle #2 median {} ms after '
               'write'.format(a_ow, summary['none']['toggle2_ms_median']))


def run_bulk(mca, rep, seconds=5.0):
    """Blocking write() is the gauge: it returns once the rest fits in the driver's buffer.

    (a) write `seconds` of data; record when write() returns, out_waiting meanwhile, and how
        long a following flush() takes.
    (b) for op in none / reset_input_buffer / reset_output_buffer: write the payload (returns
        with the buffer full), apply op, write the payload again and time it. If op discarded
        the pending output, the second write returns as early as the first.
    Between parts, wait long enough for everything to drain (flush() can't be trusted).
    """
    port = mca.serial_connection
    baud = port.baudrate
    port.write_timeout = None
    rate = baud / mca.bits_per_byte  # bytes/s
    n = int(seconds * rate)
    payload = bytes([0x55]) * n
    print('\nBulk test at {} baud: {} bytes, {:.2f} s on the wire.'.format(baud, n, seconds))
    out = {'bytes': n, 'wire_s': seconds}
    mca.ResetRTS()
    mca.ResetDTR()
    drain = seconds + 1.0

    # (a)
    t0 = time.perf_counter()
    port.write(payload)
    t_ret = time.perf_counter() - t0
    ow = port.out_waiting
    port.flush()
    t_flush = time.perf_counter() - t0
    samples = []
    while time.perf_counter() - t0 < seconds + 0.5:
        samples.append((round(time.perf_counter() - t0, 3), port.out_waiting))
        time.sleep(0.02)
    out['a_write_returned_s'] = t_ret
    out['a_out_waiting_after_write'] = ow
    out['a_flush_done_s'] = t_flush
    out['a_out_waiting_max'] = max(x[1] for x in samples)
    out['a_samples'] = samples[::5]
    buffered_s = seconds - t_ret
    out['driver_buffer_bytes_est'] = int(buffered_s * rate)
    rep.result('{} baud bulk write/out_waiting'.format(baud), 'INFO',
               'write() of {:.1f} s returned at {:.2f} s (driver buffer ~{} bytes = {:.2f} s); '
               'out_waiting right after {}, max {} while draining; flush() done at {:.3f} s'
               .format(seconds, t_ret, out['driver_buffer_bytes_est'], buffered_s, ow,
                       out['a_out_waiting_max'], t_flush))
    time.sleep(max(0.0, drain - (time.perf_counter() - t0)))

    # (b)
    for op in ('none', 'reset_input_buffer', 'reset_output_buffer'):
        t0 = time.perf_counter()
        port.write(payload)
        t1 = time.perf_counter() - t0
        if op == 'reset_input_buffer':
            port.reset_input_buffer()
        elif op == 'reset_output_buffer':
            port.reset_output_buffer()
        t_op = time.perf_counter()
        port.write(payload)
        t2 = time.perf_counter() - t_op
        out['b_' + op] = {'first_write_s': t1, 'second_write_s': t2}
        rep.result('{} baud bulk {}'.format(baud, op), 'INFO',
                   'first write returned at {:.2f} s; second write took {:.2f} s '
                   '({})'.format(t1, t2, 'pending output discarded' if t2 < t1 + 0.5 * buffered_s
                                 else 'pending output kept'))
        time.sleep(drain)

    rep.data.setdefault('bulk', {})[str(baud)] = out


def main():
    p = make_parser(__doc__)
    p.add_argument('--trials', type=int, default=20)
    p.add_argument('--bauds', default='4800,57600,115200',
                   help='comma-separated host rates to test (default: %(default)s)')
    p.add_argument('--modes', default=','.join(MODES),
                   help='comma-separated handshake modes; delay<ms> = purge RX after <ms> '
                        '(default: %(default)s)')
    p.add_argument('--bulk', action='store_true',
                   help='bulk test only; the MCA must be switched off')
    args = p.parse_args()
    if args.bulk and args.bauds == p.get_default('bauds'):
        args.bauds = '57600,115200'
    bauds = [int(b) for b in args.bauds.split(',')]
    name = 'tx_timing_bulk' if args.bulk else 'tx_timing'
    rep = Report(name, args)
    rep.info('device', os.path.realpath(args.port))
    try:
        if args.bulk:
            with quiet():
                mca = lib.MCA8000A(args.port)
            mca.serial_connection.baudrate = 4800
            for _ in range(3):
                if prompt_status(mca) is not None:
                    raise HWError('MCA answered a status prompt; switch it off for --bulk')
                time.sleep(0.2)
            for baud in bauds:
                mca.serial_connection.baudrate = baud
                run_bulk(mca, rep)
            mca.close()
        else:
            for baud in bauds:
                args.baud = baud
                mca, st = connect(args)
                rep.info('serial', st['serial'])
                run_handshake(mca, rep, args.trials, args.modes.split(','))
                run_rx_purge(mca, rep, min(args.trials, 10))
                mca.ResetRTS()
                mca.close()
                args.power_on = False
    except HWError as e:
        rep.result('setup', 'FAIL', str(e))
        rep.save()
        return 1
    rep.save()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
