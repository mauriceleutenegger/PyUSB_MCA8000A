# Tests

Two tiers, both run with pytest from the repository root:

- **Tier 1 (no hardware)**, `tests/unit/`: the library against a simulated MCA behind a
  fake serial port, on virtual time. Fast (a few seconds), deterministic, runs anywhere.
  This is what `pytest` alone runs, and what GitHub Actions runs on each push.
- **Tier 2 (hardware)**, `tests/hw/`: the library against a real MCA8000A (USB serial
  variant). Skipped unless enabled with `--hw`. Read-only unless further flags are given.

The protocol facts the tests rely on are in `docs/protocol.md`.

## Requirements

Python ≥ 3.10, `numpy`, `pyserial`, `pytest`, `pytest-timeout`. From the repository root,
install the library (editable) with the test tools:

```
python3 -m pip install -e ".[test]"
```

The tests also run from a checkout without installing the library, as long as the
dependencies are installed (`pyproject.toml` puts the root on pytest's path). CI tests
with numpy 2.

## Tier 1

```
python3 -m pytest                 # everything that needs no hardware
python3 -m pytest tests/unit/test_readout.py -v
```

Known, unfixed bugs have tests that check the *correct* behaviour, marked
`xfail(strict=True, reason='I-n: ...')`. They show as `x`. When a bug is fixed, its tests
pass, strict mode turns that into a failure, and the marker is removed with the fix.

What's simulated (`tests/mcasim/`):
- `SimMCA`: the MCA side of the protocol as documented in `docs/protocol.md`, including
  observed firmware quirks (lenient command checksum, reset to 4800 on port open, the data
  stream after a baud command, the busy second after START, threshold clamping, power-down
  on battery). Each modelled behaviour cites its source; assumptions are marked as such.
- `FakeSerial`: the part of pyserial the library uses, with the macOS FTDI driver's
  behaviour (`timing.md`). `VirtualClock` replaces the library's `timeit`, so time-outs and
  retries cost no real time.
- `Host`: a minimal hand-driven host, independent of `MCA8000A.py`. It tests the simulator,
  and on hardware it is the oracle the library is checked against.

`tests/contract.py` (read-only) and `tests/state.py` (state-changing) hold scenarios that
run unchanged on the simulator (tier 1) and on the real MCA (tier 2), so a difference between
the model and the hardware shows up as a tier-2 failure.

`tests/data/` holds responses recorded from S/N 1224 (`capture-1224.json`, `statuses.json`).
`tests/data/capture.py` (read-only) re-records them from an attached MCA.

## Tier 2

```
python3 -m pytest tests/hw --hw --mca-port=/dev/cu.usbserial-XXXX [--mca-serial N] [flags]
```

The port can also come from `$MCA_PORT` (setting it alone enables nothing). Write
`--mca-port=/dev/...` with `=`, or just the name (`--mca-port cu.usbserial-XXXX`): with a
space and a full path, pytest takes `/dev` as its root directory and fails.

| Flag | Adds | Time |
|---|---|---|
| `--hw` | Read-only tests: contract scenarios, `Initialize` at each baud rate, status and data against the oracle, 100-round robustness at 4800/57600/115200 | ~2 min |
| `--hw-state` | Threshold and preset set and read back, in the scratch group; settings restored | +1–2 min |
| `--hw-delete` | (implies `--hw-state`) start/stop, preset stop, stale cached flags, DELETE data/time/both, all in the scratch group | +3 min |
| `--hw-any-memory` | (implies `--hw-state`) resolution changes; they write into other groups' parameter channels (below). **Only on an MCA with nothing to keep.** | +1 min |
| `--hw-slow` | Power-down after 6 min idle and recovery. MCA on battery (skips otherwise); don't touch it meanwhile | ~7 min |
| `--mca-serial N` | Abort unless the MCA with S/N N answers | |
| `--power-on` | Run the remote power-on first (the MCA powers down after a few idle minutes on battery) | |

Use `-k` to pick tests, e.g. `-k "start_stop or delete"`.

### Safety

- **SET LOCK (75h) is never sent.** Every port the tests open is a `GuardedSerial`
  (`tests/lockguard.py`), which ends the session before a write starting with 75h goes out.
  There's no known way to undo a lock.
- **Read-only sessions** (`--hw` alone) send only status prompts, block data requests and
  the baud command right after opening the port. At the end, the session checks that the
  settings and the current group's times are as they were.
- **State-changing tests** work in a **scratch group**: the highest group at the current
  resolution (group 15 at 2048 channels). They skip if the MCA is acquiring, locked, in
  absolute-peak mode, or already in that group. After each test the original flags
  (resolution, timer), threshold, group and preset are restored on a freshly opened port;
  if that fails, the session stops.
- A state session reads **all 32768 channels** at its start and end (~30 s each) and fails if
  anything changed outside the scratch group. Allowed: the last 4 channels of each group the
  session visits, where the MCA rewrites the group's parameters when it switches group. The
  reads are saved as `hwcheck/results/memory-*.npy` (if that directory exists).
- Changing the resolution from the scratch group briefly puts the MCA on other memory, and
  the next group switch writes the parameters into that group's last 4 channels (e.g. group 1
  at 2048 channels). That's why the resolution test needs `--hw-any-memory`.
- `--hw-delete` clears the scratch group's data and time, and acquires there (no detector
  input needed).

Not in the suite: the lock investigation and the other one-off investigations stay
scripts in `hwcheck/` (see its README).

## Layout

```
conftest.py        options, markers, tier/opt-in skipping
lockguard.py       GuardedSerial (no SET LOCK)
contract.py        read-only scenarios, sim + hardware
state.py           state-changing scenarios, scratch group, memory snapshot, sim + hardware
mcasim/            SimMCA, FakeSerial, VirtualClock/RealClock, Host, recorded-data loader
data/              recorded responses from S/N 1224, and the script that captures them
unit/              tier 1
hw/                tier 2 (conftest: session checks, guarded port, library, oracle, scratch)
```
