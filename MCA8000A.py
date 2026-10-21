import serial
import timeit
import numpy as np

# time.sleep is unreliable
def wait (delay=0.2) :
    timeout = timeit.default_timer () + delay
    while (True) :
        if timeit.default_timer () > timeout :
            return

def addCheckSum (data) : # data is a byte array
    checksum = sum (data)
    data.append (checksum % 256)
    return data

def Command_SetBaudRate (divisor) :
    command = bytearray ([0,0,0,divisor])
    return addCheckSum (command)

def Command_SendData (start_channel, words_requested,
                      upper=False, group=False):
    # should only call with int
    if not isinstance (start_channel, int) :
        raise TypeError ('StartChannelNotInt')
    if not isinstance (words_requested, int) :
        raise TypeError ('WordsRequestedNotInt')
    # need to verify that the arguments are in bounds
    if start_channel > 16383:
        raise ValueError ('StartChannelHigh')
    if start_channel < 0 :
        raise ValueError ('StartChannelLow')
    if words_requested > 1024 :
        raise ValueError ('WordsRequestedHigh')
    if words_requested < 1 :
        raise ValueError ('WordsRequestedLow')
    words_arg = words_requested - 1 # the MCA wants a number from 0-1023
    command_byte = 64 # lower
    if upper : command_byte += 2
    if group : command_byte += 16
    # lower command byte is 64 or 0x40 or 1000000
    # upper command byte is 66 or 0x42 or 1000010
    # lower group command byte is 80 or 0x50 or 1010000
    # upper group command byte is 82 or 0x52 or 1010010
    # start channel bits 5-0 go in byte 1 bits 7-2, bits 13-6 in byte 2
    # (USB doc 3.6, DLL; confirmed on hardware, hwcheck/check_start_channel.py)
    byte1 = (start_channel % 64) << 2
    byte2 = start_channel // 64
    byte3 = words_arg % 256
    byte1 += words_arg // 256
    command = bytearray ([command_byte, byte1, byte2, byte3])
    return addCheckSum (command)
    
def Command_Control (flags, threshold, absolute_peak=False) :
    # should check type of flags (needs to be 1 byte)
    if not isinstance (threshold, int) :
        raise TypeError ('ThresholdNotInt')
    if threshold < 0 :
        raise ValueError ('ThresholdLow')
    if threshold > 16383 :
        raise ValueError ('ThresholdHigh')
    command_byte = 1
    byte2 = threshold % 256 # lower byte
    byte3 = threshold // 256 # upper byte
    if absolute_peak : byte3 += 128 # bit 15 of the threshold word
    command = bytearray ([command_byte, flags, byte2, byte3])
    return addCheckSum (command)

# pick lock number zero to unlock and wipe (e.g. in case you don't know lock#)
# not sure how to unlock if you set a non-zero lock number
# and know what it is
def Command_MCALock (lock_number) :
    if not isinstance (lock_number, int) :
        raise TypeError ('LockNumberNotInt')
    if lock_number < 0 :
        raise ValueError ('LockNumberLow')
    if lock_number >= 2**16 :
        raise ValueError ('LockNumberHigh')
    command_byte = 117
    # note that the lower and upper byte positions are actually not defined
    # in the protocol manual
    byte1 = lock_number % 256 # lower byte
    byte2 = lock_number // 256 # upper byte
    byte3 = 1 # doesn't matter, but can't be zero
    command = bytearray ([command_byte, byte1, byte2, byte3])
    return addCheckSum (command)
    
def Command_DeleteDataAndTime (delete_data=True, delete_time=True) :
    command_byte = 5
    byte1 = 1 if delete_data else 0
    byte2 = 1 if delete_time else 0
    byte3 = 1
    command = bytearray ([command_byte, byte1, byte2, byte3])
    return addCheckSum (command)

def Command_PresetTime (preset) :
    if not isinstance (preset, int) :
        raise TypeError ('PresetNotInt')
    if preset < 0 :
        raise ValueError ('PresetLow')
    if preset >= 2**24 :
        raise ValueError ('PresetHigh')
    command_byte = 2
    byte3 = preset // 2**16
    remainder = preset % 2**16
    byte2 = remainder // 256
    byte1 = remainder % 256
    command = bytearray ([command_byte, byte1, byte2, byte3])
    return addCheckSum (command)
    
class MCA8000A :
    # serial_connection: an already open port object to use instead of opening
    # device_path (e.g. a simulated port for testing). It is set to the same
    # line settings as a port opened here.
    def __init__ (self, device_path, serial_connection=None) :
        self.device_path = device_path
        if serial_connection is None :
            self.serial_connection = serial.Serial \
                (self.device_path, baudrate=4800, parity='E',
                 timeout=0.2, write_timeout=0.2)
        else :
            self.serial_connection = serial_connection
            self.serial_connection.baudrate = 4800
            self.serial_connection.parity = 'E'
            self.serial_connection.timeout = 0.2
            self.serial_connection.write_timeout = 0.2
        c = self.serial_connection
        self.bits_per_byte = 1 + c.bytesize + (c.parity !='N') + c.stopbits
        self.send_padding = 0.02
        # 20 ms padding after sending bytes 
        self.oldcts = self.serial_connection.cts
        self.olddsr = self.serial_connection.dsr
        self.debug = False
        self.is_USB_MCA = False # default
        # status variables
        self.SerialNumber = 0 # dummy
        self.Group = 0
        self.LastDataCheckSum = 0 # dummy
        self.PresetTime = 0.0
        self.RealTime = 0.0
        self.LiveTime = 0.0
        self.BatteryStatus = 0
        self.Threshold = 0
        self.isAbsolutePeak = False # absolute (True) or first peak mode
        # status from flags
        self.flags = 0 # store flags byte as well
        self.ADCResolution = 2**14
        self.isLive = False
        self.isRunning = False
        self.isProtected = False
        self.isNiCad = False
        self.isBackupBatteryBad = False
        # data
        self.ChannelData = None
        self.RawChannelData = None
        # recoverable error logging
        self.StatusChecksumErrors = 0
        
    def ResetRTS (self) :
        self.serial_connection.rts = False

    def SetRTS (self) :
        self.serial_connection.rts = True

    def ResetDTR (self) :
        self.serial_connection.dtr = False

    def SetDTR (self) :
        self.serial_connection.dtr = True

    def PurgeRX (self) :
        self.serial_connection.reset_input_buffer ()

    def PurgeTX (self):
        self.serial_connection.reset_output_buffer ()
        
    def RememberCTS (self) :
        self.oldcts = self.serial_connection.cts

    def IsCTSFlipped (self) :
        return self.oldcts != self.serial_connection.cts

    def PrintStatus (self) :
        print ("Serial Number:\t {}".format (self.SerialNumber))
        print ("Group:\t {}".format (self.Group))
        print ("Preset Time:\t {}".format (self.PresetTime))
        print ("Real Time:\t {}".format (self.RealTime))
        print ("Live Time:\t {}".format (self.LiveTime))
        print ("BatteryStatus:\t {}".format (self.BatteryStatus))
        print ("Threshold:\t {}".format (self.Threshold))
        print ("ADC Resolution:\t {}".format (self.ADCResolution))
        if self.isLive :
            print ("Using live timer")
        else :
            print ("Using real timer")
        if self.isRunning :
            print ("Acquiring data")
        else :
            print ("Not acquiring data")
        MCAsecurity = "Protected" if self.isProtected else "Public"
        print ("MCA security:\t {}".format (MCAsecurity))
        BatteryType = 'NiCad' if self.isNiCad else 'Alkaline'
        print ("Battery type:\t {}".format (BatteryType))
        BackupBatteryStatus = 'bad' if self.isBackupBatteryBad else 'good'
        print ("Backup battery is {}".format (BackupBatteryStatus))
        return
        
    def PowerOn (self, freq=2000, duration=0.1, power_on_time=4.0) : # Hz, s, s
        # spec is 1-200 kHz for > 50 ms 
        # the 4 s is from the code, probably could shorten
        delay = 0.5 / freq # delay for half a cycle
        ncycles = int (duration * freq)
        # close and open port in case something is messed up
        self.serial_connection.close ()
        self.serial_connection.open ()
        self.SetRTS ()
        for i in range (ncycles) :
            self.SetDTR ()
            wait (delay)
            self.ResetDTR ()
            wait (delay)
        # give it plenty of time to flip bits
        wait (power_on_time - duration)
        # verify correct USB behavior; old-style protocol not supported
        is_USB_MCA_CTS = (self.serial_connection.cts != self.oldcts)
        is_USB_MCA_DSR = (self.serial_connection.dsr == self.olddsr)
        if (is_USB_MCA_CTS == is_USB_MCA_DSR) :
            self.is_USB_MCA = is_USB_MCA_CTS
            if self.debug :
                print ("USBMCA == {}".format (self.is_USB_MCA))
            if not self.is_USB_MCA :
                print ("This is not a USB MCA. It is not supported.")
        else :
            self.is_USB_MCA = False
            print ("Error identifying MCA type.")
        self.ResetRTS ()
        self.RememberCTS ()
        # close port? doesn't seem necessary
        return

    # close the port (also one passed in); nothing is sent to the MCA.
    # Safe to call twice.
    def close (self) :
        self.serial_connection.close ()

    def __enter__ (self) :
        return self

    def __exit__ (self, exc_type, exc_value, traceback) :
        self.close ()

    def WaitForCTSFlip (self, delay=0.2) :
        timeout = timeit.default_timer () + delay
        while (True) :
            if self.IsCTSFlipped () :
                self.RememberCTS ()
                return 0 # success
            if timeit.default_timer () > timeout :
                if self.debug : print ("CTS did not flip")
                return 1 # failure

    # this function only does the actual write command on the serial port;
    # it then waits for the bytes to go out (their time on the wire plus
    # send_padding), because on macOS a purge right after a write can
    # discard bytes not yet sent (timing.md)
    def SendCommandBytes (self, commandbytes) :
        self.serial_connection.write (commandbytes)
        wait (len (commandbytes) * self.bits_per_byte
              / self.serial_connection.baudrate + self.send_padding)
        return 0
        
    def SendCommand (self, command, n_retries=10) :
        for i in range (n_retries) :
            if self.debug : print ("Retry number {}".format (i))
            if i : wait (0.02) # RTS low between retries (DLL: 20 ms)

            # get ready
            self.PurgeTX ()
            self.RememberCTS ()

            # set RTS and check for CTS flip
            self.SetRTS ()
            if self.WaitForCTSFlip () :
                if self.debug : print ("First CTS flip failed")
                self.ResetRTS ()
                continue # retry

            # send data
            self.SendCommandBytes (command)

            # make sure the buffer for receiving MCA data is cleared
            if self.debug :
                print ("CTS before purge:", self.serial_connection.cts,
                       "old:", self.oldcts)
            self.PurgeRX ()
            if self.debug :
                print ("CTS after purge:", self.serial_connection.cts)
            # check for 2nd CTS flip
            if self.WaitForCTSFlip () :
                if self.debug : print ("Second CTS flip failed")
                self.ResetRTS ()
                continue # retry

            self.PurgeRX () # apparently sometimes an extra byte shows up early?
            # end transmission
            self.ResetRTS ()
            
            return 0 # sending command succeeded

        print ("Sending command failed")
        return 1 # failure

    def SetBaudRate (self, baudrate) :
        if 115200 % baudrate > 0 :
            print ("SetBaudRate: Baudrate must be integer divisor of 115200")
            return 1 # failure
        divisor = 115200 // baudrate
        comm = Command_SetBaudRate (divisor)
        # clear
        self.ResetDTR ()
        self.ResetRTS ()
        wait (0.1)
        # send command
        self.SetDTR ()
        # DTR and RTS are supposed to be set simultaneously. An earlier
        # version waited 10 ms in between on M1 Macs; since the macOS FTDI
        # fix no delay is needed (hwcheck/check_baud.py, 0-30 ms tested).
        stat = self.SendCommand (comm)
        self.ResetDTR ()
        if stat :
            print ("SetBaudRate: couldn't send command")
            return 1 # failure (SendCommand leaves RTS low)
        # set baudrate on comm port, clear
        self.serial_connection.baudrate = baudrate
        wait (0.2)

        self.PurgeRX () # throw out whatever's in there

        stat = self.EndDataStream ()
        if stat :
            print ("SetBaudRate: couldn't end data stream")
            return 1 # failure
        # get status to confirm it's OK
        return self.ReceiveStatusFromPrompt ()

    # The baud rate command is the old SEND DATA command (00h), and the MCA
    # answers it: while RTS is low it streams the status and then all of
    # memory (17 s at 57600). A status prompt stops the stream at 57600,
    # but at 115200 the MCA then ignores status prompts until it receives
    # a byte, and leftover bytes can end up in front of the next data
    # transfer at either rate. A 1-word data request, sent with CTS
    # sampled after RTS has been low, ends the stream cleanly.
    # See docs/protocol.md section 5.
    def EndDataStream (self, n_retries=3) :
        command = Command_SendData (0, 1)
        for i in range (n_retries) :
            self.ResetRTS ()
            wait (0.001)
            self.RememberCTS ()
            self.SetRTS ()
            stat = self.WaitForCTSFlip ()
            if not stat :
                self.SendCommandBytes (command)
                stat = self.WaitForCTSFlip ()
            self.ResetRTS ()
            wait (0.3) # let the rest of the stream and the reply arrive
            self.PurgeRX ()
            if not stat :
                return 0 # success
        return 1 # failure


    def PromptForStatus (self) :
        self.ResetRTS ()
        wait (0.0002)
        self.RememberCTS () # sample CTS right before raising RTS, as the DLL does
        # could add a check for serial connection status
        self.SetRTS ()
        stat = self.WaitForCTSFlip ()
        self.PurgeRX ()
        self.ResetRTS ()
        wait (0.0002)
        return stat

    # a prompted status always has the serial number and group, not the
    # data checksum (docs/protocol.md section 8)
    def ReceiveStatusFromPrompt (self) :
        stat = self.PromptForStatus ()
        if not stat :
            stat = self.ReceiveStatusWithRetry ()
        return stat

    def ReceiveStatusWithRetry (self, nretries=10) :
        for i in range (nretries) :
            stat = self.ReceiveStatus (hasSerialNumber=True)
            if not stat : # success
                return stat
            if self.PromptForStatus () : # failure
                break
        return stat # the last ReceiveStatus result (nonzero here)

    def ReceiveStatus (self, hasSerialNumber=False) :
        stat, StatusData = self.ReceiveData (20)
        if stat:
            print ("ReceiveStatus: error getting status bytes")
            return 1
        stat = self.UpdateStatusFromData (StatusData, hasSerialNumber)
        if stat :
            if self.debug :
                print ("ReceiveStatus: failed in UpdateStatusFromData")
            return 1
        return 0 # success

    def ReceiveStatusCheckSum (self) :
        stat, StatusData = self.ReceiveData (20)
        if stat:
            return stat, None
        return stat, int.from_bytes (StatusData[:4], "big")
    
    def UpdateStatusFromData (self, data, hasSerialNumber=False) :
        checksum = data[-1]
        datasum = sum (data[:-1]) % 256
        if checksum != datasum :
            self.StatusChecksumErrors += 1
            if self.debug :
                print ("UpdateStatusFromData: checksum error")
                for i in range (20) :
                    print (data[i])
            return 1 # failure
        # bytes 0-3: serial number and group, or the data checksum of the
        # previous transfer (docs/protocol.md section 8)
        if hasSerialNumber :
            self.SerialNumber = int.from_bytes (data[0:2], "big")
            # bytes 2-3 are the group number (not parsed yet; the DLL
            # uses the low 10 bits)
        else :
            self.LastDataCheckSum = int.from_bytes (data[0:4], "big")
        self.PresetTime = int.from_bytes (data[4:7], "big")
        self.BatteryStatus = data[7] # leave as int
        self.RealTime = int.from_bytes (data[8:11], "big")
        RealTime75 = data[11]
        self.RealTime += (1.0 - RealTime75 / 75)
        self.LiveTime = int.from_bytes (data[12:15], "big")
        LiveTime75 = data[15]
        self.LiveTime += (1.0 - LiveTime75 / 75)
        ThresholdWord = int.from_bytes (data[16:18], "big")
        self.Threshold = ThresholdWord & 0x7FFF # threshold is bits 0-14
        self.isAbsolutePeak = bool (ThresholdWord >> 15)
        # absolute peak mode is bit 15
        self.flags = data[18]
        # parse flags
        ADCRes_bits = self.flags & 0b111 # resolution info is in bits 0-2
        self.ADCResolution = 2**(14-ADCRes_bits)
        self.isLive = bool ((self.flags >> 3) & 1)
        # live/real timer flag is bit 3
        self.isRunning = bool ((self.flags >> 4 ) & 1)
        # start/stop is bit 4
        self.isProtected = bool ((self.flags >> 5) & 1)
        # protected/public is bit 5
        self.isNiCad = bool ((self.flags >> 6) & 1)
        # NiCad/Alkaline is bit 6
        self.isBackupBatteryBad = bool ((self.flags >> 7) & 1)
        # backup battery bad/OK is bit 7
        return 0

    # returns status and data
    # if status is bad don't use data
    def ReceiveData (self, nbytes, delay=0.4) :
        # I found that sometimes a delay of 0.2 was not enough,
        # so I increased the timeout to 0.4.
        # I don't think it breaks anything.
        timeout = timeit.default_timer () + delay
        outdata = bytearray ()
        while (True) :
            available = self.serial_connection.in_waiting
            if (available) :
                timeout = timeit.default_timer () + delay # reset timer
                # read whatever you can get, but don't go over nbytes
                nbytes_to_get = min (available, nbytes - len (outdata))
                outdata += self.serial_connection.read (nbytes_to_get)
            if timeit.default_timer () > timeout :
                print ("ReceiveData: read timeout")
                return 1, None  # failure
            if len (outdata) >= nbytes : # done getting data
                if len (outdata) > nbytes :
                    return 1, outdata # got more than we expected
                else :
                    return 0, outdata

    def ReceiveChannelData (self, delay=0.1) :
        # reads channels 0 to min (resolution, 1024) - 1 of the current
        # group; above 1024 channels the rest needs more data requests
        words_requested = min (self.ADCResolution, 1024)
        start_channel = 0
        # first get the lower words
        comm = Command_SendData (start_channel, words_requested)
        stat = self.SendCommand (comm)
        if stat :
            print ("ReceiveChannelData: error sending command")
            return stat
        wait (delay)
        stat = self.ReceiveStatus () # no S/N
        if stat :
            print ("ReceiveChannelData: failed getting status")
        stat, lowerdata = self.ReceiveData (words_requested*2)
        if stat :
            print ("ReceiveChannelData: error receiving data")
            return stat
        # The docs give the checksum as the byte sum mod 2^16, the DLL compares
        # the full sum with the full 32-bit field; comparing both mod 2^16
        # accepts either (issues I-11)
        lowerdatachecksum = sum (lowerdata) % (2**16)
        # now get the upper words
        comm = Command_SendData (start_channel, words_requested, upper=True)
        stat = self.SendCommand (comm)
        if stat :
            print ("ReceiveChannelData: error sending command")
            return stat
        wait (delay)
        stat = self.ReceiveStatus () # no S/N
        if stat :
            print ("ReceiveChannelData: failed getting status")
        # ReceiveStatus updated the checksum, now check it
        if lowerdatachecksum != self.LastDataCheckSum % (2**16) :
            print ("ReceiveChannelData: lower word checksum failed")
            return 1
        # now get the upper word data
        stat, upperdata = self.ReceiveData (words_requested*2)
        if stat :
            print ("ReceiveChannelData: error receiving data")
            return stat
        upperdatachecksum = sum (upperdata)  % (2**16)
        # ask for 64 words of data to prompt a status with checksum
        dummy_words_requested = 64
        comm = Command_SendData (0,dummy_words_requested)
        # tried asking for one but for some reason that seemed to be a problem
        stat = self.SendCommand (comm)
        if stat :
            print ("ReceiveChannelData: error sending command")
            return stat
        wait (delay)
        stat = self.ReceiveStatus () # no S/N
        if stat :
            print ("ReceiveChannelData: failed getting status")
        # ReceiveStatus updated the checksum, now check it
        if upperdatachecksum != self.LastDataCheckSum % (2**16) :
            print ("ReceiveChannelData: upper word checksum failed")
            return 1
        # get the words; purging seems to not work reliably, so just read all
        stat, dummydata = self.ReceiveData (dummy_words_requested*2)
        if stat :
            print ("ReceiveChannelData: error receiving data")
            return stat
        # throw away the dummy data

        # words are unsigned, low byte first; widen before combining
        lower = np.frombuffer (lowerdata, dtype='<u2').astype (np.int64)
        upper = np.frombuffer (upperdata, dtype='<u2').astype (np.int64)
        self.RawChannelData = lower + upper * 2**16
        # zero out the top 1/32 channels of the group, which are used
        # for sliding scale linearization (none of the channels read
        # here if the resolution is 2048 or more)
        MaxChannels = min (self.RawChannelData.size,
                           self.ADCResolution - self.ADCResolution // 32)
        if self.debug :
            print ("MaxChannels = {}".format (MaxChannels))
        self.ChannelData = np.zeros (self.RawChannelData.size, dtype=int)
        self.ChannelData[0:MaxChannels] = self.RawChannelData[0:MaxChannels]
        return 0
        
    # Send a command, wait, and read the status that shows its effect.
    # name is used in error messages; receive reads the status
    # (default ReceiveStatusFromPrompt).
    def SendCommandAndUpdate (self, name, command, delay=0.0, receive=None) :
        stat = self.SendCommand (command)
        if stat :
            print ("{}: error sending command".format (name))
            return stat
        if delay : wait (delay)
        if receive is None : receive = self.ReceiveStatusFromPrompt
        stat = receive ()
        if stat :
            print ("{}: error updating status".format (name))
            return stat
        return 0

    # CONTROL command built from the MCA's current flags and threshold:
    # read them first (as the DLL does), so that stale cached values aren't
    # written back; then clear clear_flags and set set_flags.
    def SendControl (self, name, set_flags=0, clear_flags=0, threshold=None,
                     delay=0.0) :
        stat = self.ReceiveStatusFromPrompt ()
        if stat :
            print ("{}: error reading status".format (name))
            return stat
        newflags = (self.flags & ~clear_flags & 0xFF) | set_flags
        if self.debug : print ("{}: flags {}".format (name, newflags))
        if threshold is None : threshold = self.Threshold
        command = Command_Control (newflags, threshold, self.isAbsolutePeak)
        return self.SendCommandAndUpdate (name, command, delay)

    # The threshold should not exceed half the resolution. Larger values are
    # left to the MCA, which clamps them to resolution/2 - 1 (observed at
    # 2048); the status read afterwards shows the value actually set.
    def SetThreshold (self, threshold) :
        return self.SendControl ("SetThreshold", threshold=threshold)

    # Clearing the protected flag with CONTROL probably doesn't unlock;
    # the unlock procedure is unknown (docs/protocol.md section 12).
    def SetLock (self, lock_number) :
        command = Command_MCALock (lock_number)
        return self.SendCommandAndUpdate ("SetLock", command)

    def SetLockToZero (self) :
        command = Command_MCALock (0)
        return self.SendCommandAndUpdate ("SetLockToZero", command, 4.0)

    def SetADCResolution (self, channels) :
        # rounds channels up to nearest power of two
        # here is the code that sets resolution from bits:
        # self.ADCResolution = 2**(14-ADCRes_bits)
        if not isinstance (channels, int) :
            raise TypeError ('ChannelsNotInt')
        if channels <= 128 : # 256 is the lowest resolution
            raise ValueError ('ChannelsLow')
        if channels > 2**14 :
            raise ValueError ('ChannelsHigh')
        # there's probably a more elegant way to do all this
        # set a flag to round up if more than one bit is 1
        roundup = True if bin (channels).count ("1") > 1 else False
        ADCRes_bits = 0
        for i in range (14) :
            temp = channels << i
            if temp & 2**14 :
                ADCRes_bits = i
                break
        if roundup : ADCRes_bits -= 1
        if self.debug : print (ADCRes_bits)
        return self.SendControl ("SetADCResolution", set_flags=ADCRes_bits,
                                 clear_flags=0b111)

    # delay sets time between sending start and asking for status
    def StartAcquisition (self, delay=1.0) :
        return self.SendControl ("StartAcquisition", set_flags=0b10000,
                                 delay=delay) # bit 4

    # delay sets time between sending stop and asking for status
    def StopAcquisition (self, delay=0.2) :
        return self.SendControl ("StopAcquisition", clear_flags=0b10000,
                                 delay=delay) # bit 4

    # After DELETE the MCA may take up to 2 s to respond again (DTP p.10;
    # the DLL sleeps R/8 ms + 256 ms, 2.3 s at 16k), so keep prompting
    # until a status arrives or the timeout has passed.
    def ReceiveStatusAfterDelete (self, timeout=2.5) :
        timeout = timeit.default_timer () + timeout
        while (True) :
            stat = self.ReceiveStatusFromPrompt ()
            if not stat or timeit.default_timer () > timeout :
                return stat

    def DeleteDataAndTime (self, delay=0.2) :
        command = Command_DeleteDataAndTime () # default is both
        return self.SendCommandAndUpdate ("DeleteDataAndTime", command, delay,
                                          self.ReceiveStatusAfterDelete)

    def DeleteData (self, delay=0.2) :
        command = Command_DeleteDataAndTime (delete_time=False)
        return self.SendCommandAndUpdate ("DeleteData", command, delay,
                                          self.ReceiveStatusAfterDelete)

    def DeleteTime (self, delay=0.2) :
        command = Command_DeleteDataAndTime (delete_data=False)
        return self.SendCommandAndUpdate ("DeleteTime", command, delay,
                                          self.ReceiveStatusAfterDelete)

    def SetPresetTime (self, time, delay=0.2) :
        command = Command_PresetTime (time)
        return self.SendCommandAndUpdate ("SetPresetTime", command, delay)

    def Initialize (self, baudrate=115200) :
        stat = self.ReceiveStatusFromPrompt ()
        if stat :
            print ("Initialize: error getting status")
            return stat
        if self.debug : print ("Setting baud rate")
        stat = self.SetBaudRate (baudrate=baudrate)
        if stat :
            print ("Initialize: error setting baud rate")
            return stat
        stat = self.ReceiveStatusFromPrompt ()
        if stat :
            print ("Initialize: error getting status")
            return stat
        return 0
    
    # to be implemented
    
    # set group

    # set start time
    # set start date
    # get start time
    # get start date

    # I don't think these are needed if you don't care about recording
    # the external time of the experiment


