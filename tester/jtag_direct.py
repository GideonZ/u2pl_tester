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

class JtagClientException(Exception):
    pass

class JtagClient:
    def __init__(self, url = 'ftdi://ftdi:2232h/1'):
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
        
    def ecp_read_id(self):
        self.jtag.reset()
        idcode = self.jtag.read_dr(32)
        self.jtag.go_idle()
        logger.info(f"IDCODE (reset): {int(idcode):08x}")
        return int(idcode)

    def ecp_read_unique_id(self):
        self.jtag.reset()
        #bs = BitSequence(bytes_ = b'\x19')
        bs = BitSequence(0x19, False, 8)
        self.jtag.write_ir(bs)
        codebytes = self.jtag.read_dr(64)
        self.jtag.go_idle()
        code = int(codebytes)

        lot = (code >> 24) & 0xFFFFFFFF
        wafer = (code >> 19) & 31
        x = (code >> 12) & 127
        y = (code >> 5) & 127
        e = (code & 31)
        logger.info(f"  Whole code: {code:016x}")
        logger.info(f"  Wafer Lot#: {lot:08x}")
        logger.info(f"  Wafer #: {wafer}")
        logger.info(f"  Wafer X/Y/E: {x}, {y}, {e}")
        return (code, lot, wafer, x, y, e)

    def ecp_jtag_cmd8(self, cmd, param):
        self.jtag.write_ir(BitSequence(cmd, False, 8))
        self.jtag.write_dr(BitSequence(param, False, 8))
        self.jtag.go_idle()
        self.jtag_clocks(32)

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

    def ecp_load_fpga(self, filename):
	    # Reset
        logger.info("reset..")
        self.jtag.reset()
        self.ecp_jtag_cmd8(ISC_ENABLE, 0)
        self.ecp_jtag_cmd8(ISC_ERASE, 0)
        self.ecp_jtag_cmd8(LSC_RESET_CRC, 0)
        self.read_status_register()

        # Program
        logger.info("programming..");
        self.jtag.write_ir(BitSequence(LSC_BITSTREAM_BURST, False, 8))
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
        self.jtag.write_ir(BitSequence(ISC_DISABLE, False, 8))
        self.jtag.go_idle()
        self.jtag_clocks(32)
        self.read_status_register()	

    def reverse_file(self, infile, outfile):
        with open(infile, "rb") as fi:
            with open(outfile, "wb") as fo:
                buffer = fi.read()
                fo.write(self.bitreverse(buffer))

    def set_user_ir(self, ir):
        self.jtag.write_ir(BitSequence(LSC_USER1, False, 8))
        
        self.jtag.write_dr(BitSequence(ir | ir << 4, False, 8))
        #self.jtag.change_state('shift_dr')
        #rb = self.jtag.shift_register(BitSequence(ir, False, 4))
        self.jtag.write_ir(BitSequence(LSC_USER2, False, 8))
        self.jtag.change_state('shift_dr')
        #logger.info(ir, rb)

    def rw_user_data(self, data, update=False) -> BitSequence:
        data = self.jtag.shift_register(data)
        if update:
            self.jtag.go_idle()
        return data
    
    def read_user_data(self, bits) -> BitSequence:
        #self.jtag.write_ir(BitSequence(LSC_USER2, False, 8))
        inp = BitSequence(0, length = bits)
        return self.jtag.shift_register(inp)

    def user_read_id(self):
        self.set_user_ir(0)
        user_id = int(self.read_user_data(32))
        self.jtag.go_idle()
        logger.info(f"UserID: {user_id:08x}")
        return user_id

    def user_set_io(self, value):
        self.set_user_ir(2)
        self.jtag.shift_and_update_register(BitSequence(value, False, 8))
        self.jtag.go_idle()

    def read_fifo(self, expected, cmd = 4, stopOnEmpty = False):
        available = 0
        readback = b''
        while expected > 0:
            self.set_user_ir(cmd)
            available = int(self.read_user_data(8))
            #logger.info(f"Number of bytes available in FIFO: {available}, need: {expected}")
            if available > expected:
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

            #olen = len(outbytes)-1
            ##jtagcmd = bytearray((Ftdi.RW_BYTES_PVE_NVE_LSB, olen & 0xff, (olen >> 8) & 0xff))
            #jtagcmd = bytearray((0x3d, olen & 0xff, (olen >> 8) & 0xff))
            #jtagcmd.extend(outbytes)
            #self.jtag._ctrl._stack_cmd(jtagcmd)
            #self.jtag._ctrl.sync()
            #read_now = self.jtag._ctrl._ftdi.read_data_bytes(olen+1, 4)
            #print(len(read_now), read_now)

            read_now = self.jtag.shift_register(BitSequence(bytes_ = outbytes)).tobytes(msby = True)
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
                self.user_write_memory(addr, buffer)
                addr += 16384

        if bytes_read == 0:
            logger.error(f"Reading file {name} failed -> Can't upload to board.")
            raise JtagClientException("Failed to upload applictation")

    def user_run_app(self, addr):
        magic = struct.pack("<LL", addr, 0x1571babe)
        self.user_set_io(0x80) # Reset
        self.user_write_memory(0xF8, magic)
        self.user_set_io(0x00) # Unreset
    
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
            result += self.read_fifo(now * 4)
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


if __name__ == '__main__':
    logger.addHandler(ch)
    j = JtagClient()
    j.ecp_read_id()
    j.ecp_read_unique_id()
    j.read_status_register()
    #j.reverse('tester/binaries/u2p_ecp5_impl1.bit', 'tester/binaries/u2p_ecp5_impl1.rev')
    start_time = time.perf_counter()
#    #j.ecp_prog_sram('binaries/u2p_ecp5_impl1.bit')
    j.ecp_load_fpga('binaries/u2p_ecp5_dut_impl1.bit')
    end_time = time.perf_counter()
    execution_time = end_time - start_time
    logger.info(f"Execution time: {execution_time} seconds")
    time.sleep(1)
    j.user_read_id()
    #logger.info(j.set_leds(BitSequence(0, length = 32)))

    #logger.info(j.set_leds(BitSequence("10111001")))
    #j.set_leds(4)
    data = j.user_read_console(do_print = True)
    j.user_write_memory(0x10, data.encode('utf-8'))
    print(j.user_read_memory(0x0, 0x100))
    print(j.user_read_int32(0x10))
    j.user_write_int32(0x14, 87)
    print(j.user_read_int32(0x14))
    print(j.user_read_io(0, 16))
