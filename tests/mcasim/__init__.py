"""Simulated MCA8000A for the no-hardware test tier. See mca.py for the model."""

from .clock import RealClock, VirtualClock
from .mca import SimMCA, Faults, STATUS_SIZE, TOTAL_CHANNELS
from .port import FakeSerial, serial_module

__all__ = ['VirtualClock', 'RealClock', 'SimMCA', 'Faults', 'FakeSerial', 'serial_module',
           'STATUS_SIZE', 'TOTAL_CHANNELS']
