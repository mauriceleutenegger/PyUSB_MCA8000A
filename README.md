# PyUSB_MCA8000A
Control an Amptek MCA8000A over a USB serial connection using python

The USB to serial cable is controlled by the PySerial library. The algorthims used to communicate with the MCA are based on those documented in "MCA8000A data transfer protocol" as amended by "MCA8000A USB Serial Adapter Protocol Developer Guide". 

This library is not intended to support "non-USB" MCA8000A devices or more modern Amptek MCAs/DPPs. The primary intention is to support Mac OS and Linux. Feature implementation may not be complete, at the discretion of contributing authors. When documentation eventually exists, it should indicate which features are implemented.

As of the current version, this library does not yet work in any useful sense.