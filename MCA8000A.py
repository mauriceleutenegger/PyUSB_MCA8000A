import serial
import timeit
import numpy as np

# time.sleep is unreliable
def wait (delay=0.2) :
    timeout = timeit.default_timer () + delay
    while (True) :
        if timeit.default_timer () > timeout :
            return
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
    byte2 = start_channel % 256
    byte1 = start_channel // 256
    byte1 = byte1 << 2
    byte3 = words_arg % 256
    byte1 += words_arg // 256
    command = bytearray ([command_byte, byte1, byte2, byte3])
    return addCheckSum (command)
    

class MCA8000A :
    def __init__ (self, device_path, baudrate=4800) :
        self.device_path = device_path
        self.baudrate = baudrate
        self.serial_connection = serial.Serial \
            (self.device_path, baudrate=4800, parity='E',
             timeout=0.2, write_timeout=0.2)
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
        # status from flags
        self.ADCResolution = 2**14
        self.isLive = False
        self.isRunning = False
        self.isProtected = False
        self.isNiCad = False
        self.isBackupBatteryBad = False
        # data
        self.ChannelData = None

        
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

    def WaitForCTSFlip (self, delay=0.2) :
        timeout = timeit.default_timer () + delay
        while (True) :
            if self.IsCTSFlipped () :
                self.RememberCTS ()
                return 0 # success
            if timeit.default_timer () > timeout :
                if self.debug : print ("CTS did not flip")
                return 1 # failure
        return 2 # should not get here

    def WaitToSendData (self, delay=0.2) :
        timeout = timeit.default_timer () + delay
        while (True) :
            if self.serial_connection.out_waiting == 0 :
                return 0 # success
            if timeit.default_timer () > timeout :
                if self.debug : print ("Writing timed out")
                return 1 # failure
        return 2 # should not get here

    def SendCommand (self, command, n_retries=10) :
        for i in range (n_retries) :
            if self.debug : print ("Retry number {}".format (i))

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
            self.serial_connection.write (command)
            
            if self.WaitToSendData () :
                if self.debug : print ("Writing data failed")
                self.ResetRTS ()
                continue # retry
            
            # make sure the buffer for receiving MCA data is cleared
            self.PurgeRX ()

            # check for 2nd CTS flip
            if self.WaitForCTSFlip () :
                if self.debug : print ("Second CTS flip failed")
                self.ResetRTS ()
                continue # retry
            # end transmission
            self.ResetRTS ()
            
            return 0 # sending command succeeded

        print ("Sending command failed")
        return 1 # failure

    #def GetStatus (self) :
    #    self.serial_connection.rts = True
    #    if self.WaitForCTSFlip () :
    #        if self.debug : print ("GetStatus: CTS flip failed")
    #        return 1 # failure
    #    self.serial_connection.rts = False
    #    # should break the below to a separate function
    #    timeout = timeit.default_timer () + delay
    #    outdata = bytearray ()
    #    while (True) :
    #        if (self.serial_connection.in_waiting > 0) :
    #            timeout = timeit.default_timer () + delay # reset
    #            outdata.append (self.serial_connection.read (1))
    #        if timeit.default_timer () > timeout :
    #            print ("GetStatus: read timeout")
    #            return 1 # failure
    #        if len (outdata) == 20 :
    #            break # done getting data
    #    # just print bytes for now
    #    for i in range (20) :
    #        print (outdata[i])
    #    return 0


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
        stat = self.SendCommand (comm)
        self.ResetDTR ()
        if stat :
            print ("SetBaudRate: couldn't send command")
            # SHOULDN'T IT RESET RTS?
            return 1 # failure    
        # set baudrate on comm port, clear
        self.serial_connection.baudrate = baudrate
        #self.serial_connection.rts = False # should already be zero
        wait (0.2)

        self.ReceiveStatusFromPrompt ()
        # SHOULDN'T IT RESET RTS?
        # get status to confirm it's OK
        #self.GetStatus () # FIX ME
        return 0
    

    def PromptForStatus (self) :
        self.ResetRTS ()
        wait (0.0002)
        # could add a line to ensure oldcts is set
        # could add a check for serial connection status
        self.SetRTS ()
        stat = self.WaitForCTSFlip ()
        self.PurgeRX ()
        self.ResetRTS ()
        wait (0.0002)
        return stat

    # can this every be called without serial number?
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
            stat = self.PromptForStatus ()
            if stat : # failure
                break
        return stat

    def ReceiveStatus (self, hasSerialNumber=False) :
        stat, StatusData = self.ReceiveData (20)
        #timeout = timeit.default_timer () + delay
        #outdata = bytearray ()
        #while (True) :
        #    if (self.serial_connection.in_waiting > 0) :
        #        timeout = timeit.default_timer () + delay # reset
        #        outdata.append (self.serial_connection.read (1)[0])
        #    if timeit.default_timer () > timeout :
        #        print ("ReceiveStatus: read timeout")
        #        return 1 # failure
        #    if len (outdata) == 20 :
        #        break # done getting data
        stat = self.UpdateStatusFromData (StatusData, hasSerialNumber)
        if stat :
            print ("ReceiveStatus: failed in UpdateStatusFromData")
            return 1
        return 0 # success

    def ReceiveStatusCheckSum (self) :
        stat, StatusData = self.ReceiveData (20)
        if stat:
            return stat, NULL
        return stat, int.from_bytes (StatusData[:4], "big")
    
    def UpdateStatusFromData (self, data, hasSerialNumber=False) :
        checksum = data[-1]
        datasum = sum (data[:-1]) % 256
        if checksum != datasum :
            print ("UpdateStatusFromData: checksum error")
            if self.debug :
                for i in range (20) :
                    print (data[i])
            return 1 # failure
        #should do something with checksum if hasSerialNumber=False
        if hasSerialNumber :
            self.SerialNumber = int.from_bytes (data[0:2], "big")
            # I think group number is the next two bytes
            # but I did not check
        else :
            self.LastDataCheckSum = int.from_bytes (data[0:4], "big")
        self.PresetTime = int.from_bytes (data[4:7], "big")
        self.BatteryStatus = data[7] # leave as int
        self.RealTime = int.from_bytes (data[8:11], "big")
        RealTime75 = data[11]
        self.RealTime += (1.0 - RealTime75 / 75)
        self.LiveTime = int.from_bytes (data[12:15], "big")
        LiveTime75 = data[15]
        self.LiveTime += (1.0 - RealTime75 / 75)
        self.Threshold = int.from_bytes (data[16:18], "big")
        flags = data[18] # should parse this
        ADCRes_bits = flags & 0b111 # resolution info is in bits 0-2
        self.ADCResolution = 2**(14-ADCRes_bits)
        # parse that
        self.isLive = bool ((flags >> 3) & 1) # live/real timer flag is bit 3
        self.isRunning = bool ((flags >> 4 ) & 1) # start/stop is bit 4
        self.isProtected = bool ((flags >> 5) & 1) # protected/public is bit 5
        self.isNiCad = bool ((flags >> 6) & 1) # NiCad/Alkaline is bit 6
        self.isBackupBatteryBad = bool ((flags >> 7) & 1)
        # backup battery bad/OK is bit 7
        return 0

    # returns status and data
    # if status is bad don't use data
    def ReceiveData (self, nbytes, delay=0.2) :
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
                print ("ReceiveStatus: read timeout")
                return 1, None  # failure
            if len (outdata) >= nbytes : # done getting data
                if len (outdata) > nbytes :
                    return 1, outdata # got more than we expected
                else :
                    return 0, outdata
        return 1, None

    def ReceiveChannelData (self) :
        # this only works for resolution 1024 or below
        # I see unexpected behavior for the get data commands
        # when it's called with parameters other than 0,1024
        # and even for that parameter, I see weird data for
        # high channel numbers > 1000
        words_requested = min (self.ADCResolution, 1024)
        start_channel = 0
        # first get the lower words
        comm = Command_SendData (start_channel, words_requested)
        stat = self.SendCommand (comm)
        if stat :
            print ("ReceiveChannelData: error sending command")
            return stat
        stat = self.ReceiveStatus () # no S/N
        if stat :
            print ("ReceiveChannelData: failed getting status")
        stat, lowerdata = self.ReceiveData (words_requested*2)
        if stat :
            print ("ReceiveChannelData: error receiving data")
            return stat
        lowerdatachecksum = sum (lowerdata) % (2**16) # Amptek's prescription
        # now get the upper words
        comm = Command_SendData (start_channel, words_requested, upper=True)
        stat = self.SendCommand (comm)
        if stat :
            print ("ReceiveChannelData: error sending command")
            return stat
        stat = self.ReceiveStatus () # no S/N
        if stat :
            print ("ReceiveChannelData: failed getting status")
        # ReceiveStatus updated the checksum, now check it
        if lowerdatachecksum != self.LastDataCheckSum :
            print ("ReceiveChannelData: lower word checksum failed")
            return 1
        # now get the upper word data
        stat, upperdata = self.ReceiveData (words_requested*2)
        if stat :
            print ("ReceiveChannelData: error receiving data")
            return stat
        upperdatachecksum = sum (upperdata)  % (2**16)
        # ask for 1 word of data to prompt a status with checksum
        comm = Command_SendData (0,1)
        stat = self.SendCommand (comm)
        if stat :
            print ("ReceiveChannelData: error sending command")
            return stat
        stat = self.ReceiveStatus () # no S/N
        if stat :
            print ("ReceiveChannelData: failed getting status")
        # ReceiveStatus updated the checksum, now check it
        if upperdatachecksum != self.LastDataCheckSum :
            print ("ReceiveChannelData: lower word checksum failed")
            return 1
        self.PurgeRX () # throw out the remaining byte

        lower = np.frombuffer (lowerdata, dtype=np.int16)
        upper = np.frombuffer (upperdata, dtype=np.int16)
        self.ChannelData = lower + upper * 2**16
        return 0
        
