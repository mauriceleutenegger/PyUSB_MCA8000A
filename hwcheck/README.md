# Hardware checks

Scripts that answer protocol questions on a real MCA8000A (see `docs/protocol.md` §16).
They are the seed of the hardware tier of the test suite.

They build commands and parse responses themselves, so known bugs in `MCA8000A.py` don't
affect the results. The library is used only for the handshakes (`SendCommand`, status
prompts, `SetBaudRate`, `PowerOn`).

Run from this directory. Each script saves its results as JSON in `results/`.
`I-n` labels refer to the maintainer's issue list; results are summarized in
`docs/protocol.md`.

```
python3 check_probe.py [--port /dev/cu.usbserial-XXXX] [--baud 57600] [--power-on]
```

The port defaults to `$MCA_PORT`. Use `--power-on` if the MCA is off.

| Script | Question | Changes MCA state? | Time |
|---|---|---|---|
| `check_probe.py` | Connect and print status | No | seconds |
| `check_block_count.py` | Count-bit layout of 40h–52h (I-2) | No | ~30 s |
| `check_start_channel.py` | Start-channel encoding (I-1) | No | ~30 s |
| `check_checksum.py` | Data checksum: 32-bit or mod 2¹⁶ (I-11). Needs a spectrum with many counts somewhere in memory. | No | ~40 s |
| `check_cts_ack.py` | Meaning of the 2nd CTS toggle; how the checksum is checked (I-12) | No | ~30 s |
| `check_baud.py` | Baud-rate setting, DTR delay (I-17) | No (baud only) | ~2 min |
| `check_powerdown.py` | Auto power-down and recovery (I-20). Battery only. | No | ~6 min |
| `check_preset.py --yes` | Preset 0 = run until stopped? Preset stop; stale flags (I-14) | **Yes**: scratch group, clears its time; restores group and preset | ~30 s |
| `check_lock_zero.py` | Lock 0 on an already-locked unit (I-21) | **Yes: may wipe data** | ~15 s |

`check_lock_zero.py` refuses to run unless the serial number matches, the unit reports
itself locked, and `--i-understand-this-may-wipe-data` is given. Nothing here ever sets a
non-zero lock.

## Notes for running

- Opening the port resets the MCA to 4800 baud. Every script starts from there and
  switches to `--baud` (default 57600).
- 115200 baud works only with the `SetBaudRate` fix of 2026-10-15 (see `docs/protocol.md` §5).
- On battery, the MCA powers down after a few idle minutes. Run with `--power-on` if a
  script can't find it.
