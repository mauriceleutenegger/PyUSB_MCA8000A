"""I-12: what does the second CTS toggle after a command mean?

Hand-drives the SEND handshake (RTS high, wait for CTS toggle #1, write 5 bytes,
wait for CTS toggle #2, RTS low) and records the toggle timings and how many bytes
the MCA sends afterwards. Sends a 1-word data request (40h) with the correct
checksum, several wrong checksums, and truncated to 4 bytes. Read-only: the only
command that can be accepted is a data request.

Each case is classified "accepted" if 2 + 20 bytes come back (status + the data
word) and "rejected" if only 20 come back (a rejected command acts like a status
prompt). The check passes if toggle #2 happens exactly for accepted commands.

Observed 2026-10-13 (S/N 1224): the MCA accepts a command when the checksum byte is
>= the true sum mod 256, not only when it's equal. The 'ck' cases test that rule.
"""

import time

from common import (make_parser, connect, block_command, collect, prompt_status_retry,
                    Report, HWError)


def wait_toggle(port, level, timeout):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if port.cts != level:
            return time.monotonic() - t0
    return None


def hand_send(mca, cmd, timeout=0.5):
    port = mca.serial_connection
    mca.ResetRTS()
    time.sleep(0.02)
    port.reset_input_buffer()
    level = port.cts
    mca.SetRTS()
    t1 = wait_toggle(port, level, 0.2)
    if t1 is None:
        mca.ResetRTS()
        return {'toggle1_ms': None, 'toggle2_ms': None, 'bytes': None}
    level = port.cts
    port.write(bytes(cmd))
    t_write = time.monotonic()
    t2 = wait_toggle(port, level, timeout)
    write_time = len(cmd) * mca.bits_per_byte / port.baudrate
    mca.ResetRTS()
    data = collect(mca, idle=0.5)
    return {'toggle1_ms': round(t1 * 1e3, 2),
            'toggle2_ms': None if t2 is None else round(t2 * 1e3, 2),
            'nominal_write_ms': round(write_time * 1e3, 2),
            'bytes': len(data)}


def main():
    p = make_parser(__doc__)
    p.add_argument('--repeat', type=int, default=2)
    args = p.parse_args()
    rep = Report('cts_ack', args)
    try:
        mca, st = connect(args)
        cases = []
        for start in (0, 1):                        # sums 40h and 41h
            cmd = block_command(0x40, start, 1)
            ck = cmd[4]
            for delta, rule in ((0, True), (-1, False), (1, True), (0x7F, True),
                                (-ck, False)):      # -ck: checksum byte 00h
                c = bytearray(cmd)
                c[4] = (ck + delta) % 256
                expect = rule if 0 <= ck + delta <= 255 else None
                cases.append(('sum {:02x} ck {:02x}'.format(ck, c[4]), c, expect))
        cases.append(('4 bytes only', bytearray(block_command(0x40, 0, 1)[:4]), False))
        consistent_ack = consistent_rule = True
        for _ in range(args.repeat):
            for label, cmd, expect in cases:
                r = hand_send(mca, cmd)
                accepted = r['bytes'] == 22
                ack = r['toggle2_ms'] is not None
                consistent_ack &= (ack == accepted) and r['bytes'] in (20, 22)
                if expect is not None:
                    consistent_rule &= (accepted == expect)
                rep.result('{} ({})'.format(label, bytes(cmd).hex()),
                           'INFO', 'accepted={} expected={} {}'.format(accepted, expect, r))
                if prompt_status_retry(mca) is None:  # resynchronize
                    raise HWError('lost the MCA after "{}"'.format(label))
        rep.result('I-12: toggle #2 happens exactly when the command is accepted',
                   'PASS' if consistent_ack else 'FAIL')
        rep.result('checksum rule: accepted iff checksum byte >= sum mod 256',
                   'PASS' if consistent_rule else 'FAIL')
    except HWError as e:
        rep.result('error', 'FAIL', str(e))
    rep.save()


if __name__ == '__main__':
    main()
