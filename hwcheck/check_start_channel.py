"""I-1: how is the start channel encoded in block data commands?

Reads a reference window of the top 64 channels of the spectrum (they are never
zero, so they make a recognizable pattern), then reads shorter windows that start
part-way into it. If the start channel is encoded correctly, each short window is
the matching slice of the reference. Also reads the same windows encoded the way
MCA8000A.Command_SendData currently does it ('py' layout). Read-only.

The reference window itself relies on the DLL encoding; the check is that every
offset lines up, which a wrong encoding would not do.
"""

from common import (make_parser, connect, block_command, block_read, words,
                    Report, HWError)


def read_words(mca, code, start, n, layout):
    status, data = block_read(mca, block_command(code, start, n, layout))
    return words(data)


def main():
    p = make_parser(__doc__)
    p.add_argument('--window', type=int, default=64, help='reference window size')
    args = p.parse_args()
    rep = Report('start_channel', args)
    try:
        mca, st = connect(args)
        R = st['resolution']
        base = R - args.window
        rep.info('resolution', R)
        all_ok = {'dll': True, 'py': True}
        for code, name in ((0x40, 'lower'), (0x42, 'upper')):
            ref = read_words(mca, code, base, args.window, 'dll')
            rep.result('{} words, channels {}-{} (reference)'.format(name, base, R - 1), 'INFO',
                       ' '.join('{:x}'.format(w) for w in ref))
            if len(ref) != args.window or len(set(ref)) < 3:
                rep.result('{} reference window has too little structure'.format(name),
                           'INCONCLUSIVE')
                all_ok['dll'] = all_ok['py'] = None
                continue
            # a window starting at 0 checks the encoding against an unambiguous start
            if R <= 1024:
                full = read_words(mca, code, 0, R, 'dll')
                ok = full[base:] == ref
                rep.result('{}: reference == tail of a read from channel 0'.format(name),
                           'PASS' if ok else 'FAIL')
                all_ok['dll'] &= ok
            # windows that all end at the top channel, so each overlaps the non-zero tail
            for k in (1, 7, 29, 45, 50, 55, 58, 60, 61, 62, 63):
                n = args.window - k
                exp = ref[k:k + n]
                if len(set(exp)) < 2:
                    continue  # all-equal window can't tell positions apart
                for layout in ('dll', 'py'):
                    got = read_words(mca, code, base + k, n, layout)
                    ok = got == exp
                    all_ok[layout] = all_ok[layout] and ok
                    rep.result('{}: start {} N={} {} layout'.format(name, base + k, n, layout),
                               'PASS' if ok else 'FAIL',
                               '' if ok else 'expected {} got {}'.format(exp, got))
        rep.info('all_ok', all_ok)
        if all_ok['dll'] is None:
            rep.result('I-1', 'INCONCLUSIVE', 'no usable pattern')
        else:
            rep.result('I-1: DLL start-channel encoding correct', 'PASS' if all_ok['dll'] else 'FAIL')
            rep.result('I-1: MCA8000A.py start-channel encoding correct (expected FAIL)',
                       'PASS' if all_ok['py'] else 'FAIL')
    except HWError as e:
        rep.result('error', 'FAIL', str(e))
    rep.save()


if __name__ == '__main__':
    main()
