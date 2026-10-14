"""I-11: is the data checksum in the status a full 32-bit byte sum, or mod 2^16?

Reads memory in 1024-word blocks (lower and upper words) from the current group
onwards (reading past the end of a group continues into the next groups). Each
data command's status carries the checksum of the *previous* transfer, so the
reads are chained and one extra command at the end fetches the last checksum.
Read-only.

The two hypotheses only differ for transfers whose byte sum exceeds 65535, so the
result is INCONCLUSIVE unless such a block is found (needs a spectrum with lots of
counts somewhere in memory).
"""

from common import make_parser, connect, block_command, block_read, Report, HWError


def main():
    p = make_parser(__doc__)
    p.add_argument('--blocks', type=int, default=17,
                   help='number of 1024-channel blocks to scan (default: %(default)s; '
                        'start channel is limited to 16383)')
    args = p.parse_args()
    rep = Report('checksum', args)
    try:
        mca, st = connect(args)
        transfers = []
        for blk in range(args.blocks):
            start = blk * 1024
            if start > 16383:
                break
            for code in (0x40, 0x42):
                transfers.append((code, start))
        prev = None
        n_full = n_mod = n_big = n_bad = 0
        for code, start in transfers + [(0x40, 0)]:
            status, data = block_read(mca, block_command(code, start, 1024))
            if not status['checksum_ok']:
                rep.result('status checksum after {:02x} @{}'.format(code, start), 'FAIL')
            if prev is not None:
                pcode, pstart, psum = prev
                field = status['datachk']
                full = field == psum
                mod = field == psum % 65536
                big = psum > 65535
                n_big += big
                n_full += full and big
                n_mod += mod and big
                n_bad += not (full or mod)
                if big or not (full or mod):
                    rep.result('{:02x} @{}'.format(pcode, pstart),
                               'INFO' if (full or mod) else 'FAIL',
                               'byte sum {} field {} -> {}'.format(
                                   psum, field, 'full sum' if full else 'mod 2^16' if mod else 'NEITHER'))
            prev = (code, start, sum(data))
        rep.info('counts', {'big_blocks': n_big, 'full_match': n_full, 'mod_match': n_mod,
                            'mismatch': n_bad})
        print('{} transfers checked, {} with byte sum > 65535, {} mismatches'.format(
            len(transfers), n_big, n_bad))
        if n_bad:
            rep.result('checksums', 'FAIL', '{} transfers matched neither hypothesis'.format(n_bad))
        if n_big == 0:
            rep.result('I-11: checksum modulus', 'INCONCLUSIVE',
                       'no block with byte sum > 65535 found; need a high-count spectrum')
        elif n_full == n_big:
            rep.result('I-11: checksum is the full 32-bit sum', 'PASS')
        elif n_mod == n_big:
            rep.result('I-11: checksum is mod 2^16', 'PASS')
        else:
            rep.result('I-11: checksum modulus', 'INCONCLUSIVE', 'mixed results')
    except HWError as e:
        rep.result('error', 'FAIL', str(e))
    rep.save()


if __name__ == '__main__':
    main()
