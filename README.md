# PyUSB_MCA8000A
Control an Amptek MCA8000A over a USB serial connection using python

The USB to serial cable is controlled by the PySerial library. The algorthims used to communicate with the MCA are based on those documented in "MCA8000A data transfer protocol" as amended by "MCA8000A USB Serial Adapter Protocol Developer Guide". 

This library is not intended to support "non-USB" MCA8000A devices or more modern Amptek MCAs/DPPs. The primary intention is to support Mac OS and Linux. Feature implementation may not be complete, at the discretion of contributing authors. When documentation eventually exists, it should indicate which features are implemented.

## Installation

Python ≥ 3.10 with `pyserial` and `numpy`. The library is the single module
`MCA8000A.py`. From a checkout:

```
python3 -m pip install .              # or -e ".[test]" to develop and run the tests
```

## Usage

Methods return 0 on success and a nonzero code on failure. Status values (times, flags,
resolution) are attributes, updated after each command.

```python
import MCA8000A

with MCA8000A.MCA8000A('/dev/cu.usbserial-XXXX') as mca:   # the port closes on exit
    if mca.Initialize(baudrate=115200) == 0:
        mca.ReceiveChannelData()        # channels 0 to 1023 at most, in mca.ChannelData
        print(mca.RealTime, mca.LiveTime, mca.ChannelData.sum())
```

On battery the MCA powers down after a few idle minutes; `mca.PowerOn()` wakes it.
Protocol details: `docs/protocol.md`. Tests: `tests/README.md`.
