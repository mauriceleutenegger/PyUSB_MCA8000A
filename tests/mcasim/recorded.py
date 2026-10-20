"""Recorded responses from real hardware (tests/data/capture-<serial>.json)."""

import json
import pathlib

import numpy as np

DATA_DIR = pathlib.Path(__file__).parents[1] / 'data'


def load_capture(serial=1224):
    return json.loads((DATA_DIR / 'capture-{}.json'.format(serial)).read_text())


def words(hexdata):
    return np.frombuffer(bytes.fromhex(hexdata), dtype='<u2').astype(np.uint32)


def captured_memory(cap):
    """(counts, known): 32-bit counts by absolute channel (32768), and a mask of the
    channels the capture covers. Reads past the end of the group run into the next
    group (§9), so the capture can cover a few channels beyond it."""
    from .mca import TOTAL_CHANNELS
    base = cap['group'] * cap['resolution']
    lower = np.zeros(TOTAL_CHANNELS, dtype=np.uint32)
    upper = np.zeros(TOTAL_CHANNELS, dtype=np.uint32)
    known = np.zeros(TOTAL_CHANNELS, dtype=bool)
    for t in cap['transfers']:
        if t.get('code') not in (0x40, 0x42):
            continue
        sl = slice(base + t['start'], base + t['start'] + t['words'])
        (upper if t['code'] == 0x42 else lower)[sl] = words(t['data'])
        known[sl] = True
    return lower | (upper << 16), known


def captured_counts(cap):
    """32-bit counts of the captured group."""
    counts, _ = captured_memory(cap)
    base = cap['group'] * cap['resolution']
    return counts[base:base + cap['resolution']]


def configure_sim(sim, cap):
    """Put the simulator in the captured MCA's state (status fields and memory)."""
    st = bytes.fromhex(cap['statuses'][0])
    sim.serial_number = int.from_bytes(st[0:2], 'big')
    sim.group = cap['group']
    sim.preset = int.from_bytes(st[4:7], 'big')
    sim.battery = st[7]
    sim.flags = st[18]
    sim.threshold_word = int.from_bytes(st[16:18], 'big')
    g = sim.group
    # inverse of §8: T = s + 1 - f/75, nudged up to survive float rounding
    sim.real[g] = int.from_bytes(st[8:11], 'big') + 1 - st[11] / 75 + 1e-9
    sim.live[g] = int.from_bytes(st[12:15], 'big') + 1 - st[15] / 75 + 1e-9
    counts, known = captured_memory(cap)
    sim.memory[known] = counts[known]
