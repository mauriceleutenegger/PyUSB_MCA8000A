"""Lock guard for the hardware tests (tier 2): never send SET LOCK (75h).

There is no known way to undo a lock, so every port the hardware tests open
is a GuardedSerial. Its `write` ends the whole pytest session, before anything goes out,
if the data starts with the SET LOCK command byte. The library writes each 5-byte command
in one call, so the first byte is the command byte.
"""

import pytest
import serial

SET_LOCK = 0x75


def check_write(data):
    if len(data) and bytes(data)[0] == SET_LOCK:
        pytest.exit('lock guard: refused to send SET LOCK (75h): {}'.format(
            bytes(data).hex()), returncode=3)


class GuardedSerial(serial.Serial):
    def write(self, data):
        check_write(data)
        return super().write(data)
