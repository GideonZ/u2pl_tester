from pyftdi.jtag import *
from pyftdi.ftdi import *
import time
import logging
import struct
import os

# create logger
logger = logging.getLogger('JTAG')
logger.setLevel(logging.DEBUG)
ch = logging.StreamHandler()
ch.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))

ISC_NOOP = 0xFF # 0 bits - Non-operation 
READ_ID = 0xE0 # 24 bits - Read out the 32-bit IDCODE of the device 
USERCODE = 0xC0 # 24 bits - Read 32-bit usercode 
LSC_TRACEID = 0x19 # 24 bits - Read 64 bit Trace ID code. 8 bit user, 32 bit wafer lot number,
                    # 5 bit wafer #, 7 bit wafer X location, 7 bit wafer Y location, 5 bit spare 
LSC_READ_STATUS = 0x3C # 24 bits - Read out internal status 
LSC_CHECK_BUSY = 0xF0 # 24 bits - Read 1 bit busy flag to check the command execution status 
LSC_REFRESH = 0x79 # 24 bits - Equivalent to toggle PROGRAMN pin 
ISC_ENABLE = 0xC6 # 24 bits - Enable the Offline configuration mode 
ISC_ENABLE_X = 0x74 # 24 bits - Enable the Transparent configuration mode 
ISC_DISABLE = 0x26 # 24 bits - Disable the configuration operation 
ISC_PROGRAM_USERCODE = 0xC2 # 24 bits - Write the 32-bit new USERCODE data to USERCODE register 
ISC_ERASE = 0x0E # 24 bits - Bulk erase the memory array base on the access mode and array selection 
ISC_PROGRAM_DONE = 0x5E # 24 bits - Program the DONE bit if the device is in Configuration state. 
ISC_PROGRAM_SECURITY = 0xCE # 24 bits - Program the Security bit if the device is in Configuration state 
LSC_INIT_ADDRESS = 0x46 # 24 bits - Initialize the Address Shift Register 
LSC_WRITE_ADDRESS = 0xB4 # 24 bits - Write the 16 bit Address Register to move the address quickly 
LSC_BITSTREAM_BURST = 0x7A # 24 bits - Program the device the whole bitstream sent in as the command operand 
LSC_PROG_INCR_RTI = 0x82 # 24 bits - Write configuration data to the configuration memory frame at current address and post increment the address, Byte 2~0 of the opcode indicate number of the frames included in the operand field 
LSC_PROG_INCR_ENC = 0xB6 # 24 bits - Encrypt the configuration data then write 
LSC_PROG_INCR_CMP = 0xB8 # 24 bits - Decompress the configuration data, then write 
LSC_PROG_INCR_CNE = 0xBA # 24 bits - Decompress and Encrypt the configuration data, then write 
LSC_VERIFY_INCR_RTI = 0x6A # 24 bits - Read back the configuration memory frame selected by the address register and post increment the address 
LSC_PROG_CTRL0 = 0x22 # 24 bits - Modify the Control Register 0 
LSC_READ_CTRL0 = 0x20 # 24 bits - Read the Control Register 0 
LSC_RESET_CRC = 0x3B # 24 bits - Reset 16-bit frame CRC register to 0x0000 
LSC_READ_CRC = 0x60 # 24 bits - Read 16-bit frame CRC register content 
LSC_PROG_SED_CRC = 0xA2 # 24 bits - Program the calculated 32-bit CRC based on configuration bit values only into overall CRC register 
LSC_READ_SED_CRC = 0xA4 # 24 bits - Read the 32-bit SED CRC 
LSC_PROG_PASSWORD = 0xF1 # 24 bits - Program 64-bit password into the non-volatile memory (Efuse) 
LSC_READ_PASSWORD = 0xF2 # 24 bits - Read out the 64-bit password before activated for verification 
LSC_SHIFT_PASSWORD = 0xBC # 24 bits - Shift in the password to unlock for re-configuration (necessary when password protection feature is active). 
LSC_PROG_CIPHER_KEY = 0xF3 # 24 bits - Program the 128-bit cipher key into Efuse 
LSC_READ_CIPHER_KEY = 0xF4 # 24 bits - Read out the 128-bit cipher key before activated for verification 
LSC_PROG_FEATURE = 0xE4 # 24 bits - Program User Feature, such as Customer ID, I2C Slave Address, Unique ID Header 
LSC_READ_FEATURE = 0xE7 # 24 bits - Read User Feature, such as Customer ID, I2C Slave Address, Unique ID Header 
LSC_PROG_FEABITS = 0xF8 # 24 bits - Program User Feature Bits, such as CFG port and pin persistence, PWD_EN, PWD_ALL, DEC_ONLY, Feature Row Lock etc. 
LSC_READ_FEABITS = 0xFB # 24 bits - Read User Feature Bits, such as CFH port and pin persistence, PWD_EN, PWD_ALL, DEC_ONLY, Feature Row Lock etc. 
LSC_PROG_OTP = 0xF9 # 24 bits - Program OTP bits, to set Memory Sectors One Time Programmable 
LSC_READ_OTP = 0xFA # 24 bits - Read OTP bits setting 
LSC_USER1 = 0x32
LSC_USER2 = 0x38

XILINX_USER1    = 0x02
XILINX_USER2    = 0x03
XILINX_USER3    = 0x22
XILINX_USER4    = 0x23
XILINX_IDCODE   = 0x09
XILINX_USERCODE = 0x08
XILINX_PROGRAM  = 0x0B
XILINX_START    = 0x0C
XILINX_SHUTDOWN = 0x0D
XILINX_EXTEST   = 0x26
XILINX_CFG_IN   = 0x05
XILINX_CFG_OUT  = 0x04

class JtagClientException(Exception):
    pass

class JtagClient:
    def __init__(self, url = 'ftdi://ftdi:232h/0'):
        self.url = url
        self.jtag = JtagEngine(trst=False, frequency=3e6)
        self.tool = JtagTool(self.jtag)
        self.jtag.configure(url)
        self.jtag.reset()
        self._reverse = None

    def jtag_clocks(self, clocks):
        cmd = bytearray(3)
        cnt = (clocks // 8) - 1
        if cnt:
            cmd[0] = 0x8F
            cmd[1] = cnt & 0xFF
            cmd[2] = cnt >> 8
            self.jtag._ctrl._stack_cmd(cmd)
        cnt = clocks % 8
        if cnt:
            cmd[0] = 0x8E
            cmd[1] = cnt
            self.jtag._ctrl._stack_cmd(cmd[0:2])
        
    def xilinx_read_id(self):
        self.jtag.reset()
        idcode = self.jtag.read_dr(32)
        self.jtag.go_idle()
        logger.info(f"IDCODE (reset): {int(idcode):08x}")
        return int(idcode)

    def read_status_register(self):
        bs = BitSequence(LSC_READ_STATUS, False, 8)
        self.jtag.write_ir(bs)
        self.jtag.go_idle()
        status = int(self.jtag.read_dr(32))
        self.jtag.go_idle()
        logger.info(f"Status: {status:08x}")

    def bitreverse(self, bytes):
        result = bytearray(len(bytes))
        if not self._reverse:
            self._reverse = bytearray(256)
            for i in range(256):
                byte = i
                reversed_byte = 0
                for b in range(8):
                    reversed_byte <<= 1  # Left shift to make room for the next bit
                    reversed_byte |= byte & 1  # Add the least significant bit of the original byte
                    byte >>= 1  # Right shift to process the next bit
                self._reverse[i] = reversed_byte
        for i in range(len(bytes)):
            result[i] = self._reverse[bytes[i]]
        return result

    def xilinx_load_fpga(self, filename):
	    # Reset
        logger.info("reset..")
        self.jtag.reset()

        self.jtag.write_ir(BitSequence(XILINX_PROGRAM, False, 6))
        self.jtag.reset()
        self.jtag.go_idle()
        self.jtag_clocks(10000)

        # Program
        logger.info("programming..");
        self.jtag.write_ir(BitSequence(XILINX_CFG_IN, False, 6))
        with open(filename, "rb") as f:
            self.jtag.change_state('shift_dr')
            while(True):
                buffer = f.read(16384)
                if len(buffer) <= 0:
                    break

                olen = len(buffer)-1
                cmd = bytearray((Ftdi.WRITE_BYTES_NVE_MSB, olen & 0xff,
                          (olen >> 8) & 0xff))
                cmd.extend(buffer)
                self.jtag._ctrl._stack_cmd(cmd)

        self.jtag.change_state('update_dr')
        self.jtag.go_idle()
        self.jtag.write_ir(BitSequence(XILINX_START, False, 6))
        self.jtag_clocks(32)

    def reverse_file(self, infile, outfile):
        with open(infile, "rb") as fi:
            with open(outfile, "wb") as fo:
                buffer = fi.read()
                fo.write(self.bitreverse(buffer))

    def set_user_ir(self, ir):
        self.jtag.write_ir(BitSequence(XILINX_USER4, False, 6))
        
        self.jtag.write_dr(BitSequence(ir << 1 | 1, False, 5))

        self.jtag.write_ir(BitSequence(XILINX_USER4, False, 6))
        self.jtag.change_state('shift_dr')
        # Writing the first zero selects the data registers (a '1' selects the IR register)
        self.jtag.shift_register(BitSequence(0, length = 1))
        #logger.info(ir, rb)

    def rw_user_data(self, data, update=False) -> BitSequence:
        data = self.jtag.shift_register(data)
        if update:
            self.jtag.go_idle()
        return data
    
    def read_user_data(self, bits) -> BitSequence:
        inp = BitSequence(0, length = bits)
        return self.jtag.shift_register(inp)

    def user_read_id(self):
        self.set_user_ir(0)
        user_id = int(self.read_user_data(32))
        self.jtag.go_idle()
        logger.info(f"UserID: {user_id:08x}")
        return user_id

    def user_get_inputs(self):
        self.set_user_ir(1)
        inputs = int(self.read_user_data(16))
        self.jtag.go_idle()
        logger.info(f"Inputs: {inputs:04x}")
        return inputs

    def user_set_outputs(self, value):
        self.set_user_ir(2)
        self.jtag.shift_and_update_register(BitSequence(value, False, 8))
        self.jtag.go_idle()

    def read_fifo(self, expected, cmd = 4, stopOnEmpty = False, readAll = False):
        available = 0
        readback = b''
        while expected > 0:
            self.set_user_ir(cmd)
            available = int(self.read_user_data(8))
            #logger.info(f"Number of bytes available in FIFO: {available}, need: {expected}")
            if readAll:
                available = expected # !!!!
            elif available > expected:
                available = expected
            if available == 0:
                if stopOnEmpty:
                    break
                else:
                    logger.info("No more bytes in fifo?!")
                    self.jtag.go_idle()
                    raise JtagClientException("No read data.")

            outbytes = bytearray(available)
            outbytes[-1] = 0xF0 # no read on last

            olen = len(outbytes)-1
            #jtagcmd = bytearray((Ftdi.RW_BYTES_PVE_NVE_LSB, olen & 0xff, (olen >> 8) & 0xff))
            jtagcmd = bytearray((0x3d, olen & 0xff, (olen >> 8) & 0xff))
            jtagcmd.extend(outbytes)
            self.jtag._ctrl._stack_cmd(jtagcmd)
            self.jtag._ctrl.sync()
            read_now = self.jtag._ctrl._ftdi.read_data_bytes(olen+1, 4)
            #print(len(read_now), read_now)

            #read_now = self.jtag.shift_register(BitSequence(bytes_ = outbytes)).tobytes(msby = True)
            expected -= len(read_now)
            readback += read_now

        self.jtag.go_idle()
        return readback

#################
    def ecp_clear_fpga(self):
        self.jtag.reset()
        self.ecp_jtag_cmd8(ISC_ENABLE, 0)
        self.ecp_jtag_cmd8(ISC_ERASE, 0)
        self.ecp_jtag_cmd8(LSC_RESET_CRC, 0)
        self.read_status_register()
    
    def user_read_debug(self):
        self.set_user_ir(3)
        rb = self.jtag.shift_and_update_register(BitSequence(0, False, 32))
        logger.info(f"Debug register = {rb}")
        self.jtag.go_idle()
        return int(rb)
    
    def user_read_console(self, do_print = False):
        raw = self.read_fifo(expected = 1000, cmd = 10, stopOnEmpty = True)
        text = bytearray(len(raw))
        for i in range(len(text)):
            text[i] = raw[i] & 0x7F
        text = text.decode("utf-8")
        if do_print:
            logger.info(text)
        return text
    
    def user_read_console2(self, do_print = False):
        raw = self.read_fifo(expected = 1000, cmd = 11, stopOnEmpty = True)
        text = bytearray(len(raw))
        for i in range(len(text)):
            if raw[i] < 0x20 and raw[i] != 0x0a:
                text[i] = 0x3f
            text[i] = raw[i] & 0x7F
        text = text.decode("utf-8")
        if do_print:
            logger.info(text)
        return text
    
    def user_upload(self, name, addr):
        bytes_read = 0
        with open(name, "rb") as fi:
            logger.info(f"Uploading {name} to address {addr:08x}")
            while(True):
                buffer = fi.read(16384)
                if len(buffer) <= 0:
                    break
                bytes_read += len(buffer)
                self.user_write_memory(addr, buffer + b'\x00\x00\x00\x00\x00\x00\x00\x00')
                addr += 16384
            logger.info(f"Uploaded {bytes_read:06x} bytes.")

        if bytes_read == 0:
            logger.error(f"Reading file {name} failed -> Can't upload to board.")
            raise JtagClientException("Failed to upload applictation")

        return bytes_read

    def user_run_bare(self, name):
        """Uploads the application to the board, assuming that there is no bootloader present, and the CPU starts from address 0x0."""
        self.user_set_outputs(0x00) # Reset
        _size = self.user_upload(name, 0x0)
        self.user_set_outputs(0x80) # Unreset
        time.sleep(3)
        self.user_read_id()
        #with open(name+"rb", "wb") as fo:
        #    fo.write(self.user_read_memory(0x00, size))

    def user_run_app(self, addr, reset = True):
        """Uploads the application to the board and runs it, assuming that there is a bootloader present."""
        #print("Going to write")
        #time.sleep(5)
        #for i in range(50):
        #    self.user_write_int32(0xFFFC, i)
        #time.sleep(5)
        #print("Going to read")
        #for i in range(50):
        #    print(f"{self.user_read_int32(0xFFF8):08x}")
        magic = struct.pack("<LL", addr, 0x1571babe)
        if reset:
            self.user_set_outputs(0x00) # Reset
        self.user_write_memory(0xFFF8, magic)
        print(f"{self.user_read_int32(0xFFF8):08x}")
        print(f"{self.user_read_int32(0xFFFC):08x}")
        if reset:
            self.user_set_outputs(0x80) # Unreset
    
    def user_write_memory(self, addr, buffer):
        addrbytes = struct.pack("<L", addr)
        command = bytearray([ addrbytes[0], 4, addrbytes[1], 5, addrbytes[2], 6, addrbytes[3], 7, 0x80, 0x01])
        self.set_user_ir(5)
        self.jtag.shift_and_update_register(BitSequence(bytes_ = command))
        self.set_user_ir(6)
        olen = len(buffer)-1
        cmd = bytearray((Ftdi.WRITE_BYTES_NVE_LSB, olen & 0xff,
                    (olen >> 8) & 0xff))
        cmd.extend(buffer)
        self.jtag._ctrl._stack_cmd(cmd)
        self.jtag.go_idle()
    
    def user_read_memory(self, addr, len):
        result = b''
        #logger.info(f"Reading {len} bytes from address {addr:08x}...")
        len //= 4
        #start_time = time.perf_counter()

        while(len > 0):
            now = len if len < 256 else 256
            addrbytes = struct.pack("<L", addr)
            command = bytearray([ addrbytes[0], 4, addrbytes[1], 5, addrbytes[2], 6, addrbytes[3], 7, now - 1, 0x03])
            self.set_user_ir(5)
            self.jtag.shift_and_update_register(BitSequence(bytes_ = command))
            result += self.read_fifo(now * 4, readAll = True) # Assuming reading from memory is always faster than JTAG; we can just continue reading the fifo!
            len -= now
            addr += 4*now

        #end_time = time.perf_counter()
        #execution_time = end_time - start_time
        #logger.info(f"Execution time: {execution_time:.3f} seconds")

        return result

    def user_write_int32(self, addr, value):
        self.user_write_memory(addr, struct.pack("<L", value))
    
    def user_read_int32(self, addr):
        valbytes = self.user_read_memory(addr, 4)
        return struct.unpack("<L", valbytes)[0]

    def user_write_io(self, addr, bytes):
        addrbytes = struct.pack("<L", addr)
        command = struct.pack("<BBBBBB", addrbytes[0], 4, addrbytes[1], 5, addrbytes[2], 6)
        for b in bytes:
            command += struct.pack("BB", b, 0x0f)
        self.set_user_ir(5)
        self.jtag.shift_and_update_register(BitSequence(bytes_ = command))
        self.jtag.go_idle()

    def user_read_io(self, addr, len):
        addrbytes = struct.pack("<L", addr)
        command = struct.pack("<BBBBBB", addrbytes[0], 4, addrbytes[1], 5, addrbytes[2], 6)
        for i in range(len):
            command += b'\x00\x0d'
        self.set_user_ir(5)
        self.jtag.shift_and_update_register(BitSequence(bytes_ = command))
        return self.read_fifo(len)

DDRIO_OFFSET         = 0x100100
DDR_ADDR_LOW         = DDRIO_OFFSET + 0
DDR_ADDR_HIGH        = DDRIO_OFFSET + 1
DDR_CMD              = DDRIO_OFFSET + 2
DDR_DELAY_UPDOWN     = DDRIO_OFFSET + 3
DDR_DELAY_SEL_DATA_0 = DDRIO_OFFSET + 4
DDR_DELAY_SEL_DATA_1 = DDRIO_OFFSET + 5
DDR_DELAY_SEL_DQS    = DDRIO_OFFSET + 6
DDR_BITSLIP          = DDRIO_OFFSET + 7
DDR_READ_DELAY       = DDRIO_OFFSET + 8
DDR_DQS_INVERT       = DDRIO_OFFSET + 9
DDR_CONTROL          = DDRIO_OFFSET + 12
DDR_RID              = DDRIO_OFFSET + 0
DDR_RDQS             = DDRIO_OFFSET + 4
CTRL_CLOCK_ENABLE    = 1
CTRL_ODT_ENABLE      = 2
CTRL_REFRESH_ENABLE  = 4
CTRL_CKE_ENABLE      = 8

MR         = 0x0742 # xx00.0111.0100.0010 => PD = 0, WREC=011=4, DLLReset=1, TMode=0, CAS=100=4, BT=SEQ, BL=010=4
EMR_AL3    = 0x405A # 01 0 0 0 0 (yes DQSn) 000 (no OCD) 1 (150ohm) 011 (AL=3) 0 (150 ohm) 1 (half drive) 0 (dll used)

def findDQSDelaySingle(x, dqs, val = 0x55):
    if x >= len(dqs):
        return (None, None)
    try:
        first = dqs[x:].index(val)
    except ValueError:
        return (None, None)
    first += x
    print(f'First: {first}')
    for i in range(first, len(dqs)):
        if dqs[i] != val:
            return (first, i-1)
    return (first, len(dqs)-1)

def findDQSDelay(dqs):
    # Find the first '55' byte in the DQS readback

    x = 0
    longest = 0
    first = None
    while(True):
        (left, right) = findDQSDelaySingle(x, dqs)  
        if left == None:
            break 
        if right - left > longest:
            # print(f'Left: {left}, Right: {right}, x: {x}, dqs: {dqs[x:]}')
            first = left
            last = right
            longest = right - left
        x = right + 1

    if first == None:
        return None
    
    print (f'First: {first}, Last: {last}')

    if first == 0 and last > 9:
        print (f'Choice: {last - 9}')
        return last - 9
    elif last == 31 and first < 23:
        print (f'Choice: {first + 9}')
        return first + 9
    elif (last - first) > 10:
        print (f'Choice: {(first + last)//2}')
        return (first + last) // 2
    return None    

def calibrateDQS(j):
    # Calibration is done by stepping through the delays and checking the DQS signal.
    # The DQS readback is stored for all four possible read delays at once. So from
    # this DQS readback the read delay can be determined. This is the case when a consistent
    # range of '55' bytes are found in any of the four DQS readback arrays. 
    # Running at 533 MT/s, the cycle time is 1.875 ns. Since each delay tap is 0.078125 ns,
    # the delay range is 24 taps for one unit interval. The ideal tap setting is therefore
    # the 12th tap where the '55' bytes are found. If 40-60% is acceptable, the range is
    # 9-15 taps. In both cases, the chosen position should be at least 9 taps away from
    # a detected edge. It is unlikely that both edges are detected at the same time, so
    # choosing the position to be 9 taps away from the detected edge is a good choice.
    # When the '55' bytes are not found, the bitslip is triggered and the process is repeated.

    for bs in range(4):
        dqs = [[], [], [], []]
        for rd in range(32):
            j.user_read_int32(0) # Dummy read to trigger DQS
            b = j.user_read_io(DDR_RDQS, 4)
            dqs[0].append(b[0])
            dqs[1].append(b[1])
            dqs[2].append(b[2])
            dqs[3].append(b[3])
            j.user_write_io(DDR_DELAY_SEL_DQS, b'\x03')

        print(f'Bitslip: {bs}')
        print(dqs)

        for dly in range(4):
            pos = findDQSDelay(dqs[dly])
            if pos:
                j.user_write_io(DDR_READ_DELAY, [dly])
                print(f'Calibrated to idelay {pos} and read_delay {dly} with {bs} bitslips')
                for i in range(pos):
                    j.user_write_io(DDR_DELAY_SEL_DQS, b'\x03')
                    j.user_write_io(DDR_DELAY_SEL_DATA_0, b'\xff')
                    j.user_write_io(DDR_DELAY_SEL_DATA_1, b'\xff')

                j.user_write_int32(0x104, 0xABCDEF55)
                j.user_write_int32(0x100, 0x87654321)
                print(f'{j.user_read_int32(0x100):08x}, {j.user_read_int32(0x104):08x}')
                return
        j.user_write_io(DDR_BITSLIP, [ 1 ])

    print('Calibration failed')

def dumpDelays(j):
    j.user_write_io(DDR_READ_DELAY, [1])
    #j.user_write_io(DDR_BITSLIP, [1])
    str1 = ''
    str2 = ''
    str3 = ''
    for n in range(32):
        str1 += f'{j.user_read_int32(0):08x},'
        str2 += f'{j.user_read_int32(4):08x},'
        str3 += f'{struct.unpack(">L", j.user_read_io(4, 4))[0]:08x},'
        j.user_write_io(DDR_DELAY_SEL_DQS, b'\x03')
        j.user_write_io(DDR_DELAY_SEL_DATA_0, b'\xff')
        j.user_write_io(DDR_DELAY_SEL_DATA_1, b'\xff')
    print(str1)
    print(str2)
    print(str3)
    
def init_ddr2(j):
    b = j.user_read_io(DDR_RID, 4)
    print('Controller: ', b)

    # Enable clock and clock enable
    j.user_write_io(DDR_CONTROL, b'\x09') 

    # Issue a precharge all command
    j.user_write_io(DDR_ADDR_LOW, b'\x00\x04\x02')

    # Write EMR3 to zero
    j.user_write_io(DDR_ADDR_LOW, b'\x00\xC0\x00')

    # Write EMR2 to zero
    j.user_write_io(DDR_ADDR_LOW, b'\x00\x80\x00')

    # Write EMR to additive latency 3
    j.user_write_io(DDR_ADDR_LOW, b'\x5A\x40\x00')

    # Write MR to CAS 4, BL 4, DLL Reset, Write Recovery 4
    j.user_write_io(DDR_ADDR_LOW, b'\x42\x07\x00')

    # Issue a precharge all command
    j.user_write_io(DDR_ADDR_LOW, b'\x00\x04\x02')

    # Issue a refresh command
    j.user_write_io(DDR_CMD, b'\x01')
    j.user_write_io(DDR_CMD, b'\x01')
    j.user_write_io(DDR_CMD, b'\x01')
    j.user_write_io(DDR_CMD, b'\x01')

    # Prototype only
    j.user_write_io(DDR_DQS_INVERT, b'\x01')

    # Search the other direction
    j.user_write_io(DDR_DELAY_UPDOWN, b'\x01')

#    print(j.user_read_io(0, 16))
#

#    for i in range(4):
#        j.user_write_io(DDR_READ_DELAY, [i])
#        j.user_write_io(DDR_CMD, b'\x05')
#        print(j.user_read_io(0, 16))


    j.user_write_int32(0x104, 0xABCDEF55)
    j.user_write_int32(0x100, 0x87654321)
    
def prog_fpga(logger, j):
    start_time = time.perf_counter()
    #j.xilinx_load_fpga('/home/gideon/proj/ult64/target/u64_artix/u64_artix.runs/impl_1/u64_mk2_ddr2test.bit')
    j.xilinx_load_fpga('/home/gideon/proj/ult64/target/u64_artix/u64_artix.runs/impl_1/u64_mk2_artix.bit')
    end_time = time.perf_counter()
    execution_time = end_time - start_time
    logger.info(f"Execution time: {execution_time} seconds")
    time.sleep(1)

import sys
if __name__ == '__main__':
    logger.addHandler(ch)
    j = JtagClient()
    j.xilinx_read_id()

    if j.user_read_id() == 0xdead1541:
        j.user_write_int32(0xFFFC, 0x12345)

    if len(sys.argv) > 1 and sys.argv[1] == 'prog':
        prog_fpga(logger, j)

    j.user_read_id()
    #init_ddr2(j)
    #calibrateDQS(j)
    #dumpDelays(j)

    #print(f'{j.user_read_int32(0):08x}, {j.user_read_int32(4):08x}')
    #print(f'{struct.unpack(">L", j.user_read_io(4, 4))[0]:08x}')

    #j.user_write_io(0x10, b'\n')
    #j.user_write_io(0x10, b'\n')
    #j.user_write_io(0x10, b'-')
    #j.user_write_io(0x10, b'-')
    #j.user_write_io(0x10, b'\n')
    #j.user_write_io(0x10, b'\n')


    #j.user_run_bare('/home/gideon/proj/ult64/ultimate/target/u64ii/riscv/ultimate/result/ultimate.bin')

    j.user_set_outputs(0x80) # Unreset to start bootloader
    j.user_read_id()
    time.sleep(1)
    data = j.user_read_console(do_print = True)

    # After calibration, load the application and run it
    #size = j.user_upload('/home/gideon/proj/ult64/ultimate/target/u64ii/riscv/test/result/u64ii_test.bin', 0x30000)
    size = j.user_upload('/home/gideon/proj/ult64/ultimate/target/u64ii/riscv/ultimate/result/ultimate.bin', 0x30000)
    #size = j.user_upload('/home/gideon/proj/ult64/ultimate/target/u64ii/riscv/update/result/update.bin', 0x30000)
    j.user_run_app(0x30000, reset = False)
    #j.user_set_outputs(0x80) # Unreset to start bootloader
    j.user_read_id()

    #print(data)

