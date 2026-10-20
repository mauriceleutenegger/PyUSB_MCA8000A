# MCA8000A communication protocol (USB serial adapter variant)

This document describes how a host talks to an Amptek MCA8000A "pocket MCA" that implements
the **USB Serial Adapter Protocol** (the variant used by `MCA8000A.py`). It consolidates
Amptek's documents and the Amptek PMCA DLL source code. Where they disagree, or where they
disagree with what this library does, that's stated explicitly.

The older (non-USB) MCA8000A protocol is covered only where the USB protocol builds on it.

## Contents

1. [Sources and conventions](#1-sources-and-conventions)
2. [Overview](#2-overview)
3. [Serial port settings and control lines](#3-serial-port-settings-and-control-lines)
4. [Remote power-on and protocol detection](#4-remote-power-on-and-protocol-detection)
5. [Setting the baud rate (connecting)](#5-setting-the-baud-rate-connecting)
6. [Sending a command (SEND mode)](#6-sending-a-command-send-mode)
7. [Receiving from the MCA (RECEIVE mode)](#7-receiving-from-the-mca-receive-mode)
8. [Status structure](#8-status-structure)
9. [Spectral data and readout](#9-spectral-data-and-readout)
10. [Command reference](#10-command-reference)
11. [Start stamp](#11-start-stamp)
12. [Security lock](#12-security-lock)
13. [Automatic power-down](#13-automatic-power-down)
14. [Timing summary](#14-timing-summary)
15. [Implementation status in `MCA8000A.py`](#15-implementation-status-in-mca8000apy)
16. [Open questions](#16-open-questions)

---

## 1. Sources and conventions

### Sources

| Tag | Document | Version |
|---|---|---|
| **[DTP]** | *MCA8000A Data Transfer Protocol* (`dataprot.pdf`) | undated; referred to by [USB] as Rev 1.4.1 |
| **[USB]** | *MCA8000A USB Serial Adapter Protocol Developer Guide* | Rev 1.42, Oct 2008 |
| **[API]** | *MCA8000A Programmer's Guide (API)* (PMCA COM DLL) | Rev 3, 1/29/01 |
| **[API-U]** | *Update to the MCA8000A Programmer's Guide (API) for the USB Serial Adapter Protocol* | Rev 1.42, Oct 2008 |
| **[DLL]** | PMCA DLL C++ source, `Request.cpp`, `PmcaCom.h` | version with 2009-04-02 "slow USB" fixes |
| **[PY]** | `MCA8000A.py` in this repository | current |

All except [PY] are available from Amptek; [PY] is this library. Two copies of the DLL source are distributed; the copy under
`MCA8000A API/PmcaDLL/DLL Source Code/` is the newer one and is the one cited here. DLL
references use function names.

[USB] is a set of amendments to [DTP]. Anything [USB] doesn't change still holds ([USB] §4
lists what is unchanged). The DLL is Amptek's working implementation, so it's the best evidence
of what the hardware actually does when the documents are unclear. Behavior observed on real
hardware takes precedence over all of them.

### Conventions

- "PC" means the host. Signal names are as seen at the **PC** serial port: the PC drives
  **RTS** and **DTR** and reads **CTS** and **DSR**.
- Bytes are numbered in transmission order, starting at 0.
- `xxh` is hexadecimal. Decimal command codes from [DTP] are given alongside.
- **🔬 Observed** marks behavior measured on hardware (MCA8000A S/N 1224, FTDI USB-serial
  adapter, macOS, 2026-10-13) with the scripts in `hwcheck/`. One unit only.
- **⚠ Conflict** marks a disagreement between sources. **❓** marks something not yet
  verified.

---

## 2. Overview

- The PC is always the master and the MCA always the slave [DTP p.9].
- **RTS sets the direction**: RTS high is SEND mode (PC → MCA, a command is being sent); RTS
  low is RECEIVE mode (MCA → PC) [DTP p.9; USB §2.1.1].
- Every PC → MCA transfer is a **5-byte command** (§10).
- Every MCA → PC transfer starts with a **20-byte status structure** (§8). Data commands are
  followed by channel data (§9), and the GET START STAMP command by an 8-byte start stamp (§11).
- **USB protocol vs. old protocol**: the old protocol handshook every byte by toggling
  DSR/DTR. The USB protocol drops per-byte handshaking because USB adapters have more than
  1 ms of control-line latency. Instead the MCA toggles **CTS** at a few defined points, and
  the host says how many words it wants with four new **block data commands** (40h/42h/50h/52h)
  [USB §2.1].
- **DTR** is still used in the USB protocol, but only for remote power-on (§4) and baud rate
  setting (§5) [USB §2.1].

A typical session:

```
power on (optional) → set baud rate → prompt for status
→ { configure / start / stop / delete / read data, each followed by a status read }
```

---

## 3. Serial port settings and control lines

### Line settings

| Setting | Value | Source |
|---|---|---|
| Data bits | 8 | [DTP p.8] |
| Stop bits | 1 | [DTP p.8] |
| Parity | MCA sends a parity bit of **0**. Space parity recommended; even/odd usable if host parity errors are ignored. | [DTP p.8] |
| Baud rate at power-on | 4800 | [DTP p.12] |
| Baud rates available | 115200 / *n* for integer divisor *n* (1–255); > 19200 recommended | [DTP p.12] |
| Flow control | None in the UART sense; RTS/CTS/DTR are driven and read manually | [DLL `OpenComPort`] |

- **[DLL]** opens the port with `SPACEPARITY`, parity checking off (`fParity = FALSE`), and RTS
  and DTR under manual control.
- **[PY]** uses `parity='E'` (even). pyserial on macOS has no space parity (it needs
  `CMSPAR`, which is Linux-only). With even parity, any incoming byte with an odd number of 1
  bits arrives with a "wrong" parity bit. This works as long as the host doesn't drop or mark
  bytes with parity errors. 🔬 On macOS every status and data transfer arrived complete with
  valid checksums, so bytes with "wrong" parity are not dropped. ❓ Linux unchecked.
- The PC transmits with *even* parity rather than space. The MCA evidently ignores the parity
  of received bytes, since commands get through.

### Control line roles (USB protocol)

| Line | Direction | Role |
|---|---|---|
| RTS | PC → MCA | Direction: high = SEND (PC → MCA), low = RECEIVE (MCA → PC). Also held high during power-on. |
| CTS | MCA → PC | MCA handshake: toggles (changes state) when the MCA is ready for a command, and again when it has accepted one (§6). Toggles at power-on on USB MCAs. |
| DTR | PC → MCA | Square wave = power-on (§4). High during the command = baud rate setting (§5). Otherwise low. |
| DSR | MCA → PC | Not used in the USB protocol, except that it does *not* toggle at power-on on USB MCAs (§4). |

All handshakes are **edge-based**: the host remembers the last CTS level and waits for it to
change. The absolute level means nothing.

[DTP p.8] notes that the RxD/TxD lines are inverted on both PC and MCA. That's a wiring matter
handled by the cable/adapter and invisible to software.

### Host-side notes: FTDI on macOS

See `timing.md` in this repository. On macOS, with **both** FTDI's VCP driver and Apple's
AppleUSBFTDI (🔬 `hwcheck/check_tx_timing.py`, 2026-10-19):

- `out_waiting` returns 0 while bytes are still in flight (almost always after a 5-byte
  write; always with kB pending), so it can't be used to wait for a write to finish.
- `flush()` (tcdrain) returns at once, so it can't be used either.
- `reset_input_buffer()` (purging the RX buffer) also discards **output that hasn't been sent
  yet**, apparently including bytes already in the chip's FIFO. A purge right after `write()`
  lost the command every time at 4800 and 0.5–10 % of the time at 57600–115200.

Together these mean a command can be purged before it's sent. [PY] therefore waits a computed
fixed time after each write before doing anything else:
`len(command) × bits_per_byte / baud + 20 ms`, where `bits_per_byte` = 1 start + 8 data +
1 parity + 1 stop = 11. This matters wherever the protocol calls for an RX purge right after
sending (§6, §7). 🔬 With this wait, 1200/1200 commands arrived (both drivers, 4800–115200).
The DLL purges RX *before* writing, which avoids the race altogether.

---

## 4. Remote power-on and protocol detection

### Power-on [DTP p.13–14]

1. Hold RTS high.
2. While RTS is high, drive DTR with a square wave: 1–200 kHz, lasting > 50 ms, > 4 Vpp.

| | Frequency | Duration | Wait afterwards |
|---|---|---|---|
| [DTP] | 1–200 kHz | > 50 ms | — |
| [DLL] `OP_POWER_UP` | ~1 kHz (0.5 ms high / 0.5 ms low, 199 cycles) | ~200 ms | 3.8 s |
| [PY] `PowerOn` | 2 kHz (default) | 100 ms (default) | until 4.0 s total (default) |

### Detecting the USB protocol [USB §3.12; DLL `OP_POWER_UP`]

USB-protocol MCAs toggle **CTS** during power-up; old-protocol MCAs toggle **DSR**. The DLL:

1. Opens the port and samples CTS and DSR (initial values).
2. Sets RTS high, sends the DTR square wave, then waits 3.8 s.
3. Samples CTS and DSR again.
4. CTS changed → "USB by CTS". DSR unchanged → "USB by DSR".
5. If the two verdicts agree, that's the answer; otherwise "can't identify" (treated as not
   USB).
6. Sets RTS low and closes the port.

Notes:

- If the MCA was **already on**, neither line toggles, so the result is "can't identify".
  ❓ What the power-on sequence does to an MCA that's already on hasn't been documented.
- **[PY]** samples the initial CTS/DSR in `__init__`, not right before the DTR toggling, and
  closes and reopens the port in between. The result is stored in `is_USB_MCA` but nothing
  else uses it.

---

## 5. Setting the baud rate (connecting)

The MCA always starts at 4800 baud. The baud rate is changed by sending a **SEND DATA command
(00h) with DTR high**, with the divisor in byte 3 [DTP p.12–13]. [USB] leaves this unchanged.

**[DTP] sequence** (old protocol, adapted):

1. Set the UART to 4800. Put the divisor (115200 / desired rate) in byte 3.
2. Remember the handshake line state.
3. Set RTS and DTR high together.
4–8. Send the command as usual (§6).
9. On success: dummy-read RX, **switch the UART to the new rate**, clear RTS, set DTR high.
   Wait ≥ 100–200 µs.
10. On timeout: clear RTS and DTR, wait 100–200 µs, retry.

**What the DLL does** (`ExecConnect`):

1. Open the port at 4800.
2. `SetDtr()`.
3. `SendCommandData(0)`: command 00h, address 0, divisor in byte 3, sent with the normal USB
   send sequence (§6), which sets and clears RTS.
4. `ResetDtr()`.
5. Switch the UART to the new rate.
6. Prompt for status (§7) **twice**. The comment says "Repeat to clear CheckSum". The serial
   number is read from the second status. Serial number 0 means an MCA8000 (not "A").

**[PY]** (`SetBaudRate`, called from `Initialize`) matches the DLL: DTR low and RTS low, wait
0.1 s, DTR high, send `[00h, 0, 0, divisor, checksum]`, DTR low, switch the UART, wait 0.2 s,
purge RX, then prompt for status once. `Initialize` also prompts for status before (at 4800)
and after the baud change.

⚠ **Conflict, minor.** [DTP] says byte 3 of command 00h "must be zero" (p.4), but byte 3 also
carries the baud divisor (p.12). The DLL **always** puts the current divisor in byte 3 of
00h/10h, including when DTR is low. So the MCA presumably only acts on byte 3 when DTR is high.
This doesn't matter for [PY], which only uses 00h for setting the baud rate.

🔬 **Observed** (`hwcheck/check_baud.py`):

- **Opening the port resets the MCA to 4800 baud**, every time. pyserial raises DTR and RTS
  when it opens the port; DTR + RTS high is how baud-setting mode starts, which presumably
  resets the rate. Opening also counts as a status prompt: when RTS then goes low, the MCA
  sends a 20-byte status.
- **A baud change only works when sent at 4800.** Sent at any other rate, it leaves the MCA
  at 4800. This matches [DTP]'s "the serial port must be configured for 4800".
- With [PY]'s sequence (no delay between DTR and RTS), switching to 9600, 19200, 38400 and
  57600 worked every time.
- **The baud command starts a data stream.** Command 00h is the old SEND DATA command, and
  the MCA answers it. After the switch, while RTS is low, it streams the old-protocol
  response (status, then channel data) at the new rate. It was still streaming after 3 s at
  4800, 57600 and 115200.
  - At ≤ 57600 the next status prompt stops it. The rest of the stream plus the status comes
    back, so expect more than 20 bytes.
  - At 115200 the prompt stops it, but the MCA then answers no plain prompt until it has
    received at least one byte in SEND mode. Any command (even a rejected one) releases the
    rest of the stream plus its normal response; after that, everything is normal.
  - Left alone, the stream is the status plus the lower words of **all 32768 channels** (all
    groups): 65556 bytes, 17 s at 57600. At 115200 the MCA stalled after 11897 words (4 s),
    without losing bytes.
  - At both rates, more leftover bytes can be prepended to the first large data transfer
    unless a small data request comes first.
- **Reliable sequence** ([PY] `SetBaudRate` → `EndDataStream`, since 2026-10-15): after
  switching the host rate, wait 0.2 s and purge. Then send a 1-word data request (40h, start
  0) with CTS sampled after RTS has been low ~1 ms, wait for toggle #2, drop RTS, wait 0.3 s,
  purge. Then prompt for status. 🔬 With this, `Initialize()` worked 40/40 (57600 and 115200,
  both macOS drivers), each followed by exact 1024-word reads. Sending the 1-word request
  through the library's ordinary `SendCommand` was not reliable at 115200 (once: success
  reported, nothing received). An earlier prototype that sent a rejected command
  (`40 00 00 00 00`) first also worked (45/45), but isn't needed.

---

## 6. Sending a command (SEND mode)

### As documented [USB §3.9]

1. Remember CTS (OLDCTS).
2. Set RTS high.
3. Start a timeout of 110–165 ms per byte.
4. Wait for the transmit register to be empty.
5. Wait for CTS to differ from OLDCTS, then send the 5 command bytes.
6. Dummy-read the RX buffer, then clear RTS. Wait ≥ 100–200 µs before the next command.
7. If the command was one of 00h, 10h, 30h, 40h, 42h, 50h or 52h, the PC is now in RECEIVE
   mode (§7).
8. On timeout: set RTS low, wait 100–200 µs.

[DTP p.10] adds: retry **at least 10 times** before concluding that something is wrong.
After a DELETE command in particular, the MCA may need up to **2 s**. RTS must be low for
≥ 100 µs between attempts.

### What the DLL actually does (`SendCommand`, USB branch)

```
read modem status (CTS reference)
repeat up to 10 times:
    RTS high
    wait ≤ 200 ms for CTS toggle #1        -- MCA ready
    purge RX
    write the 5 bytes
    wait ≤ 200 ms for CTS toggle #2        -- MCA accepted the command
    RTS low
    if success: break
    sleep 20 ms
wait 200 µs
```

⚠ **Conflict: second CTS toggle.** [USB §3.9] describes only one CTS toggle (before the
bytes are sent). The DLL, and [PY], also wait for a **second toggle after the last byte** and
treat its absence as a failure. This mirrors the old protocol's step 8 [DTP p.9]: "wait until
DSR changes … indicating that the MCA has received the last byte, the check sum … is OK, and
the command code is recognizable".

🔬 **Observed** (`hwcheck/check_cts_ack.py`):

- **Toggle #2 means "command accepted".** It comes about 1–3 ms after the last byte. If it
  doesn't come, the command was not executed, and lowering RTS just gets a status, the same as
  a prompt. A truncated (4-byte) command gets no toggle #2.
- **The checksum test is lenient.** A command was accepted when its checksum byte was
  **greater than or equal to** the true sum (mod 256), not only when equal. For a 40h command
  with sum 40h, checksums 40h–FFh were accepted and 00h–3Fh rejected. With sum 41h, 40h was
  rejected and 41h accepted. So a corrupted command is caught only if the corruption makes the
  checksum byte smaller than the sum. Only tested with command 40h.
- An extra 6th byte after a valid command was ignored.

DLL quirk: if CTS toggle #1 fails, the DLL `break`s out of the retry loop *without* clearing
RTS and without retrying. That looks like a DLL bug. Don't copy it.

### [PY] (`SendCommand`)

```
repeat up to 10 times:
    purge TX; remember CTS
    RTS high
    wait ≤ 200 ms for CTS toggle #1      (on failure: RTS low, retry)
    write the 5 bytes; wait fixed time (§3)
    purge RX
    wait ≤ 200 ms for CTS toggle #2      (on failure: RTS low, retry)
    purge RX
    RTS low
```

Differences from the DLL:

- [PY] purges RX **after** writing, not before. It waits a fixed time first because of the
  macOS purge bug (§3). The DLL originally purged after writing too; on 2009-04-02 Amptek
  moved the purge in front of the write "to insure purge before slow prolific usb adapter
  data" and commented out the one after (`Request.cpp:888-915`). Since then the DLL never
  purges between the write and toggle #2. [PY]'s second purge, after toggle #2, is safe: the
  MCA has the whole command by then and doesn't send until RTS goes low.
- [PY] retries immediately. RTS is low only for as long as the next few calls take, versus
  ≥ 20 ms in the DLL and ≥ 100 µs in [DTP]. USB control-line latency probably stretches this
  in practice. ❓
- [PY] doesn't add the 200 µs wait after RTS goes low. Callers usually add their own delay.

---

## 7. Receiving from the MCA (RECEIVE mode)

When RTS is low the MCA may transmit. In the USB protocol the MCA sends without per-byte
handshaking, so the host must know how many bytes to expect [USB §2.1, §3.10]:

| After … | MCA sends |
|---|---|
| a block data command (40h/42h/50h/52h) for *N* words | 20-byte status, then 2·*N* data bytes |
| GET START STAMP (30h) | 8-byte start stamp (the DLL reads no status first) |
| a status prompt (below) | 20-byte status |
| any other command | nothing until prompted |

[USB §3.10]: flush the RX buffer as early as possible, allow time for the data to arrive, and
verify the checksum for block data commands.

### Prompting for status (`PromptPmcaForStatus`)

The host gets a status by pulsing RTS **without sending a command**. The MCA acknowledges the
RTS rise with a CTS toggle. When RTS falls it transmits the 20-byte status.

| Step | [DLL] / [USB §3.13] | [PY] `PromptForStatus` |
|---|---|---|
| 1 | RTS low, wait 200 µs | RTS low, wait 200 µs |
| 2 | read modem status (CTS reference) | — (uses the CTS remembered earlier) ❓ |
| 3 | RTS high | RTS high |
| 4 | wait ≤ 200 ms for CTS toggle | same |
| 5 | purge RX, wait 200 µs | purge RX |
| 6 | RTS low, wait 200 µs | RTS low, wait 200 µs |
| 7 | read 20 bytes (`ReceiveStatusWithRetry`) | same |

`ReceiveStatusWithRetry` [DLL and PY]: up to 10 times, read the status; if the read or the
checksum fails, prompt again. The [DLL] returns the result of the last *prompt*, so it reports
success when every status read failed but the prompts' CTS toggles still came (e.g. host and
MCA at different baud rates); `Request.cpp:1110-1124`. [PY] had the same flaw until
2026-10-19 and now returns the last status read's result.

### Read timing

- **[DLL] `ReceivePacket`**: first waits the nominal transfer time (12 bit times per byte),
  then reads until it has the expected count, with a hard timeout of 0.75 s. Reads go in
  packets of at most 1024 bytes.
- **[PY] `ReceiveData`**: polls `in_waiting` and reads up to the expected count. Fails if
  0.4 s passes **with no new bytes** (the timeout restarts whenever data arrives), or if more
  bytes arrive than expected.

---

## 8. Status structure

20 bytes. **Multi-byte fields are sent most significant byte first** [DTP p.1–3]. (The DLL
reverses the whole buffer into a little-endian C struct, `PmcaStatusType`.)

| Byte | Name | Meaning |
|---|---|---|
| 0–3 | DataChkSum_3 … _0 | 32-bit value: **either** the data checksum **or** S/N + group (see below) |
| 4–6 | PresetTime_2 … _0 | Preset time, seconds (24-bit) |
| 7 | Battery | Battery voltage code; 0 = external power |
| 8–10 | RealTime_2 … _0 | Elapsed real time, whole seconds (24-bit) |
| 11 | RealTime_75 | Fractional real time (below) |
| 12–14 | LiveTime_2 … _0 | Elapsed live time, whole seconds (24-bit) |
| 15 | LiveTime_75 | Fractional live time |
| 16–17 | Threshold_1, _0 | Bit 15: absolute peak mode; bits 14–0: threshold (channel) |
| 18 | Flags | See below |
| 19 | CheckSum | Sum of bytes 0–18, mod 256 |

### DataChkSum field

[DTP p.2, p.5], [DLL `PmcaStatusType`, `ReceiveDataWords`, `ExecConnect`]:

- In the **first status after a data transfer** (that is, the status sent in response to the
  *next* data command), the field is the **sum of all channel data bytes sent in the previous
  transfer**. The DLL comment: "This is the only opportunity to get status with checksum.
  After status is sent PMCA clears the checksum."
  🔬 Confirmed for a status prompt: after 40h for 64 words and then a prompt, the status of the
  next 40h had 0 in this field, not the sum (3/3; without the prompt, the sum).
- Otherwise (status prompts, after S/N-type commands 10h/50h/52h) it holds the **serial
  number and group**:
  - bytes 0–1: serial number (16-bit, MSB first)
  - bytes 2–3: group number. [DTP] says byte 3 (DataChkSum_0) is the group; the DLL reads the
    low 10 bits of bytes 2–3. Groups only go up to 127, so these agree in practice.
- Serial number 0 identifies an MCA8000 (not "A") [DLL `ExecConnect`].

⚠ **Conflict: checksum modulus.** [DTP] and [USB §3.7] say the data checksum is the byte sum
"MOD 2¹⁶". The DLL compares the full 32-bit sum with the full 32-bit field, with no modulus.
[PY] reduces its own sum mod 2¹⁶ and compares that with the full 32-bit field. A transfer of
1024 words can have a byte sum up to 522,240, so these give different results once the sum
exceeds 65,535. [PY] has only been used with low-count spectra so far (byte sums < 500), so
the question has never come up. ❓ Hardware check: read a spectrum with a large byte sum.

### Times

```
T = 65536·T_2 + 256·T_1 + T_0 + (1 − T_75 / 75)        [seconds]
```

[DTP p.2]. The 75ths field evidently counts *down* within the current second. The DLL computes
the same value in ms: `(sec + 1)·1000 − T_75 / 0.075`. Real and live time can't be set by the
host, only cleared (DELETE). Time is kept per group [API, "Clear time"].

🔬 **Observed** (S/N 1224, 2026-10-19, raw commands, status polled every 0.25 s):
- After a START (CONTROL with bit 4 set), the MCA answers no status prompts for about 1 s,
  and real/live time start counting **0.97 s after the command** (2 runs, ±10 ms). STOP takes
  effect within the poll interval. The clock rate is right (fit slope 0.999).
  [PY]'s fixed 1 s wait in `StartAcquisition` matches the busy time; the [DLL] doesn't wait.
- One status read while acquiring had a checksum off by one (sum 219, byte 218): a time byte
  probably changed while the status was being sent. A checksum error while acquiring is
  worth one retry.

### Threshold word

- Bits 14–0: low-level threshold as a channel number. It "should not exceed half of the
  currently set resolution" [DTP p.3].
  🔬 **Observed** (S/N 1224 at 2048 channels, 2026-10-19): the MCA **clamps** larger values
  to R/2 − 1. CONTROL with 1024 … 16383 is accepted (toggle #2) and the status then reports
  1023; 0 … 1023 are taken as sent. Not checked at other resolutions.
- Bit 15: **absolute peak mode** (1) vs. first peak mode (0). This comes from the [DLL]/[API]
  `PmcaControlType` (`threshold:15; absolutePeak:1`). It isn't in [DTP]. The MCA powers up in
  first peak mode, so absolute peak mode must be set again after each power-on [API].

### Flags byte [DTP p.3; API]

| Bit | Meaning |
|---|---|
| 2–0 | ADC resolution ("gain") code: 000 = 16384, 001 = 8192, 010 = 4096, 011 = 2048, 100 = 1024, 101 = 512, 110 = 256. Resolution = 16384 >> code. 111 is undefined. |
| 3 | Timer: 1 = live, 0 = real (which time the preset applies to) |
| 4 | 1 = acquiring (start), 0 = stopped |
| 5 | Security: 1 = protected (locked), 0 = public |
| 6 | Main battery type: 1 = NiCd, 0 = alkaline |
| 7 | Backup battery: 1 = bad, 0 = OK |

The same byte is written back with the CONTROL command (§10). The normal pattern is
read–modify–write: get the status, change bits, and send flags and threshold back together.

### Battery byte [DTP p.2; DLL `GetBatteryCapacity`]

0 means external power. Otherwise:

| Remaining capacity | Alkaline (3 V) | NiCd (2.5 V) |
|---|---|---|
| < 5 % | < 50 | < 68 |
| 12.5 % | 63 | 73 |
| 25 % | 68 | 74 |
| 37.5 % | 71 | 75 |
| 50 % | 74 | 76 |
| 62.5 % | 77 | 77 |
| 75 % | 80 | 78 |
| > 87.5 % | > 88 | > 80 |

The DLL interpolates linearly in this table, using 5 % for the "< 5 %" row's upper value. A
DLL comment says the NiCd values "do not match admca", so treat them as approximate.

---

## 9. Spectral data and readout

### Memory organization

- Each channel is a 32-bit unsigned count, made of a **lower word** (bits 15–0) and an **upper
  word** (bits 31–16) [DTP p.1].
- Total memory is 32768 channels, divided into **groups** of size *R* (the resolution):
  2 groups at 16k, 4 at 8k, … 128 at 256 [DTP p.6; DLL `TOTAL_MEMORY_SIZE`].
- Data commands address channels **within the current group**. Reading past the end of the
  current group continues into the **following** groups, so data in higher groups can be read
  without changing group; data in lower groups can't [API, "Get Data" note].
- **Top 1/32 of each spectrum**: Amptek's example code reads only *R* − *R*/32 channels
  [API-U, VB example]. [PY] zeroes those channels in `ChannelData` (keeping them in
  `RawChannelData`), with the comment that they're used for sliding-scale linearization.
  Observed: the top few channels are **never zero**, even with no input counts, so they
  don't hold ordinary spectrum data. ❓ None of the documents in the reference set say what
  they contain. Some Amptek documentation is believed to explain it, but it hasn't been
  located.

  🔬 **Observed**: at resolution 2048 (group 0), the top channels hold values that match the
  group's status and start stamp:

  | Channel | Upper word | Lower word | Matches |
  |---|---|---|---|
  | 2042 | 2002h | 2420h | start date 2024-02-20 in BCD? |
  | 2043 | 0500h | 1625h | start time 16:25 in BCD? |
  | 2044 | 0040h | 03B1h | threshold 64 (upper); lower unknown |
  | 2045 | 0 | 0906h | live time 9 s + 6/75 |
  | 2046 | 0 | 0A4Bh | real time 10 s + 75/75 |
  | 2047 | 0 | 0A00h | preset 10 s |

  So the MCA seems to keep each group's times, preset, threshold and start stamp in the top
  channels of that group's memory. Channel 1984 (= R − R/32) had lower word 2.

  🔬 **Observed** (2026-10-19, memory read before and after the state tests): the MCA
  **rewrites a group's last 4 channels when it switches group** (SET GROUP), with that
  group's current values. Group 0's channel 2047 then read 0A58h (preset 10, and 58h = 88 =
  the battery byte at the time) and 2044 read 0040_0309h (threshold 64; flags 03h in the
  lower word, plus one unknown byte). Since the group number is kept when the resolution
  changes, changing the resolution and then the group writes the parameters into the last
  4 channels of whatever group that number means at the new resolution, i.e. into other
  memory. At 2048 → 256 from group 15, that was channels 4092–4095 (the top of group 1 at
  2048).

### Block data commands (USB protocol) [USB §3.2–3.8]

| Code | Name | Words sent | DataChkSum field of the status |
|---|---|---|---|
| 40h | Send Data Lower Word | lower words | data checksum of the previous transfer |
| 42h | Send Data Upper Word | upper words | data checksum of the previous transfer |
| 50h | Send Data Group Lower Word | lower words | S/N + group |
| 52h | Send Data Group Upper Word | upper words | S/N + group |

The code is the old command (00h or 10h) + 2 for upper words + 40h.

Parameters:

- *start channel* S: 0–16383 (14 bits)
- *count* N: 1–1024 words, sent as W = N − 1 (10 bits)

**Byte layout (as implemented by the DLL, `SendCommandDataEx`):**

| Byte | Bits 7–2 | Bits 1–0 |
|---|---|---|
| 0 | command code | |
| 1 | S bits 5–0 | W bits 9–8 |
| 2 | S bits 13–6 (all 8 bits) | |
| 3 | W bits 7–0 (all 8 bits) | |
| 4 | checksum = (byte0 + byte1 + byte2 + byte3) mod 256 | |

Equivalently: bytes 1–2, read as a little-endian 16-bit value with the low 2 bits masked off,
equal 4·S. That's the same "channel × 4" address the old 00h command used. The low 2 bits of
byte 1 then carry the top of W.

⚠ **Conflict (resolved by hardware): where the count bits go.** The bit table in [USB §3.4/§3.6] puts W bits 1–0 in
byte 1 and W bits 9–2 in byte 3. The C example in the same document [USB §3.11] and the DLL put
W bits 9–8 in byte 1 and W bits 7–0 in byte 3, as in the table above. The DLL is Amptek's
working code, so its layout is taken as correct. Usage of [PY] doesn't settle it: W = 1023
encodes the same under both schemes. W = 63 (the 64-word dummy read) encodes differently, but
if the MCA sent extra words, [PY] would silently leave them in the buffer.

🔬 **Observed** (`hwcheck/check_block_count.py`): the MCA decodes the count with the **DLL
layout** in 12 of 12 discriminating cases (N = 2 … 300), and sends exactly N words. The bit
table in [USB] is wrong.

🔬 **Observed** (`hwcheck/check_start_channel.py`): the start-channel encoding above is
correct at every offset tested, and [PY]'s current encoding reads the wrong channels.

**Start channel in [PY]:** `Command_SendData` puts S bits 13–8 in byte 1 and S bits 7–0 in
byte 2. That disagrees with both [USB] and the DLL; it only gives the right result for S = 0,
the only value [PY] uses. (Known bug.)

### Byte order of data

Within each 2-byte word the **low byte is sent first** [DTP p.12]. The 2·N bytes are N
little-endian unsigned 16-bit values.

### Readout sequence

The checksum for a transfer arrives only in the status of the **next** data command, so a
readout needs one extra command at the end.

**[DLL] `ReceiveDataWords`**, for the lower words and then again for the upper words:

```
repeat up to 10 times:
    send 40h (or 42h) for start S, N words
    sleep 5 ms
    read status (with retry), discarding it
    read 2·N bytes, summing them
    send 40h/42h with W = 0 (1 word)       -- just to get the checksum
    read status; compare DataChkSum with the sum
    sleep 20 ms
    if equal: prompt for status (also flushes the 1 leftover word); done
```

After both halves, `OP_GET_DATA` sleeps 100 ms and prompts for status once more. Requests over
1024 words are split by the caller ([API-U] example).

**[PY] `ReceiveChannelData`** chains the commands so that each data command's status carries
the checksum of the previous transfer:

```
send 40h (S=0, N)  → wait 0.1 s → read status → read 2N bytes (lower)
send 42h (S=0, N)  → wait 0.1 s → read status (checksum of lower) → read 2N bytes (upper)
send 40h (S=0, 64) → wait 0.1 s → read status (checksum of upper) → read 128 bytes (discarded)
```

Here N = min(resolution, 1024). The 64-word dummy read was chosen because asking for 1 word
"seemed to be a problem" (comment in [PY]). The DLL handles the leftover word by purging RX
during the next prompt. On macOS the purge-discards-output bug (§3) may be why the 1-word
version failed. ❓

---

## 10. Command reference

Every command is 5 bytes: code, three parameter bytes, then a checksum = (sum of bytes 0–3)
mod 256 [DTP p.4]. Multi-byte parameters are **little-endian** (low byte in the lower-numbered
byte), the opposite of the status structure. "Non-zero" bytes are ignored but must not be 0;
the DLL uses 1.

| Code | Name | Byte 1 | Byte 2 | Byte 3 | MCA then sends | Notes |
|---|---|---|---|---|---|---|
| 00h (0) | SEND DATA AND CHECKSUM | addr low | addr high | 0, or baud divisor | status + data (old protocol) | addr = 4·channel + (0 lower / 2 upper). With DTR high: sets the baud rate (§5). |
| 10h (16) | SEND DATA, GROUP AND S/N | addr low | addr high | non-zero | status + data (old protocol) | Status has S/N + group |
| 40h/42h/50h/52h | Block data commands | see §9 | | | status + 2N bytes | USB protocol only |
| 01h (1) | CONTROL | Flags | threshold low | threshold high (bit 7 = absolute peak) | — | Read–modify–write of flags and threshold |
| 02h (2) | PRESET TIME | bits 7–0 | bits 15–8 | bits 23–16 | — | Seconds; applies to the selected timer (live or real). 0 means "no preset" (run until stopped) 🔬 |
| 05h (5) | DELETE | 1 = delete data | 1 = delete time | non-zero | — | Current group. MCA may be slow to respond afterwards (§14). |
| 11h (17) | SET GROUP | 0 | group | non-zero | — | Ignored while acquiring. 0 ≤ group < 32768 / R. |
| 19h / 20h | SET START DATE | year (BCD) | month (BCD) | day (BCD) | — | The code itself is the BCD century (19 or 20). Ignored while acquiring. |
| 25h (37) | SET START TIME | hours (BCD) | minutes (BCD) | seconds (BCD) | — | Ignored while acquiring |
| 30h (48) | GET START STAMP | non-zero | non-zero | non-zero | 8-byte start stamp (§11) | |
| 75h (117) | SET MCA LOCK | lock low | lock high | non-zero | — | §12 |

Notes:

- **Old data commands (00h/10h) on a USB MCA.** [USB §3.13] says they still work but with
  restrictions: the status must be requested with control-signal toggles (a prompt), which
  stops the MCA's transmitter after the data, and the data checksum should be ignored. Use the
  block commands for data.
- **Byte order of lock, preset time, group.** [DTP] gives the preset time byte order explicitly.
  For the lock number it says only "Byte 1,2 Lock number"; the DLL sends it little-endian
  (C `WORD`), as does [PY].
- **Commands that start, stop or change the resolution, timer, battery type or peak mode** are
  all CONTROL with modified flags and threshold. The DLL always reads the status first and
  sends back the fresh values. [PY] sends its last-known `self.flags` / `self.Threshold`.
- **Start time stamp on start.** When asked to, the DLL's start-acquisition sends SET START
  TIME, then SET START DATE (1 ms apart), then the CONTROL command. [DTP p.3] says the computer
  "must" set the start stamp every time acquisition is started, but nothing breaks if it
  doesn't.

---

## 11. Start stamp

8 bytes of packed BCD, sent **seconds first, century last** [DTP p.3–4; DLL
`ReceiveDateTime`]:

| Byte | Field |
|---|---|
| 0 | seconds (0–59) |
| 1 | minutes (0–59) |
| 2 | hours (0–23) |
| 3 | not used |
| 4 | day (1–31) |
| 5 | month (1–12) |
| 6 | year (0–99) |
| 7 | century (19 or 20) |

The stamp belongs to the current group. It is set by 25h (time) and 19h/20h (date), and read
by 30h.

---

## 12. Security lock

[DTP p.7]: SET MCA LOCK (75h) sets a 16-bit lock number "used to unlock the MCA after power up
or after long periods of taking data in local mode". It refers to the MCA user's manual, which
isn't in the reference set. When the MCA is locked, status flag bit 5 is 1.

[PY] notes from use (comments in `Command_MCALock`, `SetLockToZero`): setting lock number 0
unlocks and **wipes** the MCA, and needs about 4 s before the next status. ❓ This is
unconfirmed, and the procedure for unlocking with a known non-zero lock number isn't known.
Because a lock that can't be undone would make a unit unusable, lock commands are only ever
sent to a unit that is already locked (to try lock 0), and a new lock is set only once the
unlock procedure has been shown to work on that unit.

---

## 13. Automatic power-down

[DTP p.14]: on battery, the MCA powers itself down after about **3 minutes** if it isn't
acquiring and hasn't communicated with the host. Software should communicate at least every
**2 minutes**.

Observed: USB-protocol MCAs on battery **do** power down when idle.

The DLL sends a keep-alive (command 00h) about once a second, but **only for old-protocol
MCAs** (`MonitorThreadProc`: `!isUSBMCA`). [PY] has no keep-alive.

🔬 **Observed** (`hwcheck/check_powerdown.py`, 300 s idle on battery):

- The MCA was powered down. Status prompts got no data. [PY] calls failed cleanly:
  `ReceiveStatusFromPrompt` returned an error in 0.2 s, and `SendCommand` gave up after
  10 retries in 2.1 s.
- A CTS change was still seen on the first prompt, so a CTS toggle alone doesn't prove the
  MCA is on. Only a valid status does.
- Reopening the port doesn't wake it. Remote power-on (§4) plus the baud setting does, and
  the stored state (times, settings) survives power-down.
- ❓ The exact idle time before power-down hasn't been measured.

---

## 14. Timing summary

| What | [DTP]/[USB] | [DLL] | [PY] |
|---|---|---|---|
| Timeout per CTS/DSR handshake | 110–165 ms per byte | 200 ms | 200 ms (`WaitForCTSFlip`) |
| Command retries | ≥ 10 | 10 | 10 |
| Pause between retries (RTS low) | ≥ 100 µs | 20 ms | ~0 (immediate) |
| After a command, before the next | 100–200 µs | 200 µs | caller-specific delays (0.1–1 s) |
| Write completion | wait for THR empty | synchronous byte-by-byte `WriteFile` | fixed: 11 bits/byte ÷ baud + 20 ms |
| Prompt: RTS low before / after | — / — | 200 µs / 200 µs (+200 µs after purge) | 200 µs / 200 µs |
| Data command → read status | "allow enough time" | 5 ms | 0.1 s |
| Receive timeout | — | nominal time + 0.75 s total | 0.4 s with no new bytes |
| After DELETE | MCA may need up to 2 s | sleep R/8 + 256 ms (e.g. 384 ms at 1k, 2304 ms at 16k) | 0.2 s, then prompt with retries |
| After lock 0 | — | — | 4 s |
| After start acquisition | — | — | 1.0 s before status (🔬 MCA busy, not counting, ~0.97 s) |
| Baud change → first status | ≥ 100–200 µs | prompt immediately | 0.2 s |
| Power-on DTR burst / settle | > 50 ms at 1–200 kHz / — | ~200 ms at 1 kHz / 3.8 s | 100 ms at 2 kHz / 3.9 s |
| Auto power-down | ~3 min idle (keep-alive < 2 min) | keep-alive ~1 s (old MCA only) | none |

---

## 15. Implementation status in `MCA8000A.py`

| Feature | Status |
|---|---|
| Remote power-on + USB detection | Implemented (`PowerOn`) |
| Baud rate setting | Implemented (`SetBaudRate`, `Initialize`) |
| Status prompt / parse | Implemented (`PromptForStatus`, `ReceiveStatus*`, `UpdateStatusFromData`). Group number and absolute-peak bit not parsed. |
| Data readout | Partial (`ReceiveChannelData`): first 1024 channels only, start channel 0 only. Data decoding has known problems (signed 16-bit; fails with numpy ≥ 2). |
| Start / stop acquisition | Implemented |
| ADC resolution | Implemented (`SetADCResolution`) |
| Threshold | Implemented (`SetThreshold`) |
| Preset time | Implemented (`SetPresetTime`) |
| Live / real timer select | Not implemented |
| Absolute / first peak mode | Not implemented |
| Battery type | Not implemented (read only) |
| Delete data / time / both | Implemented |
| Set group | Not implemented |
| Start stamp (set time / date, get stamp) | Not implemented |
| Lock | Implemented (`SetLock`, `SetLockToZero`) |
| Keep-alive | Not implemented |
| Old (non-USB) protocol | Not supported, by design |

The test suite (`tests/`, see `tests/README.md`) checks the library against a simulated MCA
and, optionally, a real one. Known bugs have strict-xfail tests.

---

## 16. Open questions

To be resolved by hardware tests or further reading; ❓ marks them in the text.

1. ~~Byte layout of the block data commands (§9).~~ Resolved: DLL layout (observed).
2. Is the data checksum mod 2¹⁶ or a full 32-bit sum (§8)? The checksums of all 32 low-count
   transfers matched; a spectrum with many counts is needed to decide.
3. ~~Does the second CTS toggle after a command mean "accepted"?~~ Yes (observed, §6). Also
   found: a wrong checksum is only rejected if it's *less than* the true sum.
4. What do the top 1/32 channels contain (§9)? The last 4 hold the group's parameters and are
   rewritten on a group switch (observed); the start stamp is just below; the rest is still
   unknown.
5. ~~Does a USB MCA on battery power down when idle?~~ Yes, and it recovers with power-on +
   baud setting; state is kept (observed, §13). Exact idle time not measured.
6. What does the power-on sequence do to an MCA that's already on (§4)?
7. How do you unlock an MCA with a known non-zero lock number? Does lock 0 really unlock and
   wipe (§12)? Not to be tested by locking a working unit.
8. Does pyserial with `parity='E'` ever discard or flag received bytes (§3)? Not on macOS
   (observed); Linux unchecked.
9. ~~Does a preset time of 0 mean "run until stopped"?~~ Yes 🔬 (still acquiring after 5 s with
   preset 0; a 3 s real-time preset stopped at real 3.00 s, live 2.99 s).
10. ~~Is a DTR→RTS delay needed when setting the baud rate?~~ No (observed, §5).
11. ~~Why doesn't 115200 baud work on the test unit (§5)?~~ Resolved: the baud command starts a
    data stream that must be ended (fixed 2026-10-15).
12. ~~What happens to a threshold above R/2 (§8)?~~ Clamped to R/2 − 1 (observed at 2048).
13. Why does counting start 0.97 s after START (§8)? Does it depend on resolution or on a
    preceding DELETE?
