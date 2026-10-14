"""I-2: where do the words-requested bits go in block data commands (40h)?

Sends Send Data Lower Word (40h) from start channel 0 for several word counts N,
encoded with the DLL layout and the USB-doc bit-table layout, and counts how many
words actually come back. Read-only.

  DLL layout: byte1 bits 1-0 = W[9:8], byte3 = W[7:0]   (W = N - 1)
  doc layout: byte1 bits 1-0 = W[1:0], byte3 = W[9:2]
"""

from common import make_parser, connect, block_command, block_read, Report, HWError


def decoded_count(cmd, layout):
    """Word count the MCA would see if it decodes `cmd` with `layout`."""
    b1, b3 = cmd[1], cmd[3]
    if layout == 'dll':
        return ((b1 & 3) << 8 | b3) + 1
    return (b3 << 2 | (b1 & 3)) + 1


def main():
    p = make_parser(__doc__)
    p.add_argument('--counts', default='1,2,5,17,64,100,300,1024',
                   help='comma-separated word counts to try (default: %(default)s)')
    args = p.parse_args()
    rep = Report('block_count', args)
    try:
        mca, st = connect(args)
        counts = [int(x) for x in args.counts.split(',')]
        votes = {'dll': 0, 'doc': 0}
        for n in counts:
            for layout in ('dll', 'doc'):
                cmd = block_command(0x40, 0, n, layout)
                status, data = block_read(mca, cmd)
                got = len(data) // 2
                if_dll = decoded_count(cmd, 'dll')
                if_doc = decoded_count(cmd, 'doc')
                if if_dll == if_doc:
                    verdict, why = 'INFO', 'both layouts give {} (not discriminating)'.format(if_dll)
                elif got == if_dll:
                    verdict, why = 'INFO', 'matches DLL decoding'
                    votes['dll'] += 1
                elif got == if_doc:
                    verdict, why = 'INFO', 'matches doc decoding'
                    votes['doc'] += 1
                else:
                    verdict, why = 'INCONCLUSIVE', 'matches neither'
                rep.result('N={} sent as {} layout ({})'.format(n, layout, bytes(cmd).hex()),
                           verdict, 'got {} words{}; DLL decoding -> {}, doc decoding -> {}; {}'.format(
                               got, ' + {} odd byte'.format(len(data) % 2) if len(data) % 2 else '',
                               if_dll, if_doc, why))
                if not status['checksum_ok']:
                    rep.result('status checksum', 'FAIL', 'bad status checksum')
        rep.info('votes', votes)
        if votes['dll'] and not votes['doc']:
            rep.result('I-2: MCA decodes the count with the DLL layout', 'PASS', str(votes))
        elif votes['doc'] and not votes['dll']:
            rep.result('I-2: MCA decodes the count with the USB-doc bit-table layout', 'PASS', str(votes))
        else:
            rep.result('I-2: count layout', 'INCONCLUSIVE', str(votes))
    except HWError as e:
        rep.result('error', 'FAIL', str(e))
    rep.save()


if __name__ == '__main__':
    main()
