There are bugs in the MacOS FTDI library. The command out_waiting fromt the serial object always returns 0 even before the command is sent. The command reset_input_buffer purges the input buffer even from unsent data that are still pending. Together these cause commands to be purged before they are sent when using a write strategy that monitors out_waiting. Therefore the present implementation uses a fixed delay based on the command length, baud rate, and padding to ensure margin.

## Retest, 2026-10-19

Tested with `hwcheck/check_tx_timing.py` on macOS 15 (Darwin 24.6), pyserial 3.5, one FT232R
cable, through **both** drivers that bind to it: FTDI's VCP dext (`com.ftdi.vcp.dext` 1.1,
`/dev/cu.usbserial-A10KR3LP`) and Apple's AppleUSBFTDI (`/dev/cu.usbserial-5`). **Both drivers
behave the same**, so this is not specific to FTDI's driver. Results: `hwcheck/results/tx_timing*`.

**Method.** The MCA's handshake detects whether a command really went out: after a 5-byte
command, CTS toggles a second time only when the MCA has received the complete, valid command.
The script sends a harmless 1-word data request and does something right after `write()`.

**`reset_input_buffer()` discards unsent output: confirmed.** Purge RX right after `write()`:

| Purge at | 4800 (5 bytes = 11.5 ms) | 57600 (0.95 ms) | 115200 (0.48 ms) |
|---|---|---|---|
| immediately | lost 20/20 (FTDI), 49/50 (Apple) | lost 2–8 % | lost 0.5–1 % |
| after 5 ms | lost 50/50 | | |
| after 8–12 ms | 0 lost | | |
| after 0.25–0.5 ms | | lost 2–10 % | lost 0–2.5 % |
| after 1 ms | | 0/100 | 0/200 (FTDI), 2/200 (Apple) |
| after wire time + 20 ms (library) | 0/200 | 0/200 | 0/200 |

At 4800 a command that is only partly on the wire is lost, so the purge seems to reset the
chip's TX FIFO too, not just the host buffer. At high rates the window is set by USB
scheduling, not by the wire time. The purge does clear received bytes correctly, and
`reset_output_buffer()` leaves received bytes alone (the two are not just swapped).

**`out_waiting` is useless for waiting on a write: confirmed.** After a 5-byte write it reads 0
about 95 % of the time while the bytes are still in flight (it sometimes reads 5 for 2–3 ms).
With several kB pending in the driver (MCA unplugged), it read 0 throughout.

**Bulk writes (`--bulk`, MCA unplugged; 5 s of data at 57600 and 115200).** A blocking
`write()` returned before the data could have been sent, implying a driver buffer of roughly
3–15 kB; the estimate varied between runs and rates, so the buffering isn't simple. A
non-blocking write accepted 800 bytes at 4800 and 1024 at 57600/115200. Using the blocking
write as a gauge for "did the purge discard the buffered output?" didn't work: even
`reset_output_buffer()` appeared to keep it. Inconclusive without a receiver.

**New: `flush()` (tcdrain) doesn't wait either.** It returns in ~0.02 ms after a 5-byte write,
and right after `write()` returns when kB are still buffered. Flush-then-purge loses commands
just like a plain purge.

**Conclusion.** The fixed delay (`len × 11 / baud + 20 ms`) is the right workaround and has
margin (1200/1200 on both drivers, 4800–115200). The race could be avoided altogether by
not purging between the write and the MCA's CTS toggle #2: Amptek's DLL purges RX only
*before* writing (since 2009-04-02), and the purge after toggle #2 is safe.

Not tested: Linux; the 4800 delay window on the FTDI driver between 5 and 10 ms; whether a
purge discards data still held in the driver's buffer (needs a loopback to measure).
