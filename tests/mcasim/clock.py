"""Virtual time for the simulator.

The library busy-waits on `timeit.default_timer` (in `wait`, `WaitForCTSFlip` and
`ReceiveData`). Tests replace the library's `timeit` module with a VirtualClock, whose
`default_timer` advances time by a small tick per call. So every busy-wait loop moves
virtual time forward, the simulated MCA sees time pass, and no real time is spent: a
failure path with 10 retries x 0.2 s costs milliseconds.
"""

import time


class VirtualClock:
    def __init__(self, tick=20e-6):
        self.now = 0.0
        self.tick = tick

    def default_timer(self):
        """Stands in for timeit.default_timer: returns the time, then ticks."""
        t = self.now
        self.now += self.tick
        return t

    def sleep(self, dt):
        """For test code (host helpers); the library itself never sleeps."""
        if dt > 0:
            self.now += dt


class RealClock:
    """Wall time with the VirtualClock interface, for driving real hardware (tier 2)."""

    @property
    def now(self):
        return time.monotonic()

    def sleep(self, dt):
        if dt > 0:
            time.sleep(dt)
