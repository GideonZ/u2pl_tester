
from jtag_direct import JtagClientException
from support import TestFail, TestFailCritical, Tester, DeviceUnderTest, find_ones, find_zeros
import os
import subprocess
import time
import math
import struct
from fft import calc_fft, calc_fft_mono
import numpy as np
import logging
from tkinter import ttk, messagebox

# create logger
logger = logging.getLogger('Tests')
logger.setLevel(logging.DEBUG)
#ch = logging.StreamHandler()
#ch.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))
#logger.addHandler(ch)

dut_fpga  = 'binaries/u2p_ecp5_dut_impl1.bit'
dut_appl  = 'binaries/dut.bin'
final_fpga = 'binaries/u2p_ecp5_impl1.bit'
final_appl = 'binaries/ultimate.app'
final_fat = 'binaries/fat.bin'
tester_fpga = 'binaries/u2pl_slot_tester_impl1.bit'
ECPPROG = '../ecpprog/ecpprog/ecpprog'

TEST_SEND_ETH = 5
TEST_RECV_ETH = 6
TEST_USB_PHY = 7
TEST_USB_HUB = 8
TEST_USB_PORTS = 9
TEST_BUTTONS = 10
TEST_USB_INIT = 11
TEST_RTC_ACCESS = 13
TEST_RTC_READ = 14
TEST_RTC_WRITE = 15
TEST_USB_SHOW = 17

pio_names = {
     0: 'SLOT_ADDR[0]',
     1: 'SLOT_ADDR[1]',
     2: 'SLOT_ADDR[2]',
     3: 'SLOT_ADDR[3]',
     4: 'SLOT_ADDR[4]',
     5: 'SLOT_ADDR[5]',
     6: 'SLOT_ADDR[6]',
     7: 'SLOT_ADDR[7]',
     8: 'SLOT_ADDR[8]',
     9: 'SLOT_ADDR[9]',
    10: 'SLOT_ADDR[10]',
    11: 'SLOT_ADDR[11]',
    12: 'SLOT_ADDR[12]',
    13: 'SLOT_ADDR[13]',
    14: 'SLOT_ADDR[14]',
    15: 'SLOT_ADDR[15]',
    16: 'SLOT_DATA[0]',
    17: 'SLOT_DATA[1]',
    18: 'SLOT_DATA[2]',
    19: 'SLOT_DATA[3]',
    20: 'SLOT_DATA[4]',
    21: 'SLOT_DATA[5]',
    22: 'SLOT_DATA[6]',
    23: 'SLOT_DATA[7]',
    24: 'SLOT_PHI2',
    25: 'SLOT_DOTCLK',
    26: 'SLOT_RSTn',
    27: 'SLOT_RWn',
    28: 'SLOT_BA',
    29: 'SLOT_DMAn',
    30: 'SLOT_EXROMn',
    31: 'SLOT_GAMEn',
    32: 'SLOT_ROMHn',
    33: 'SLOT_ROMLn',
    34: 'SLOT_IO1n',
    35: 'SLOT_IO2n',
    36: 'SLOT_IRQn',
    37: 'SLOT_NMIn',
    38: 'SLOT_VCC',
    40: 'CAS_MOTOR',
    41: 'CAS_SENSE',
    42: 'CAS_READ',
    43: 'CAS_WRITE',
}

pio_top    = [ 16, 17, 18, 19, 20, 21, 22, 23, 29, 28, 33, 35, 30, 31, 34, 25, 27, 36]
pio_bottom = [  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 24, 37, 26, 32]
pio_bottom_out = [  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 37, 26, 32]
pio_cassette = [ 40, 41, 42, 43 ]

def bitwise_and(array_a, array_b):
    out_array = bytearray(len(array_a))
    for i,b in enumerate(array_a):
        out_array[i] = b & array_b[i]
    return out_array

def bitwise_ornot(array_a, array_b):
    out_array = bytearray(len(array_a))
    for i,b in enumerate(array_a):
        out_array[i] = b | (255 - array_b[i])
    return out_array

class UltimateIIPlusLatticeTests:
    def __init__(self):
        pass

    def startup(self):
        # Startup JTAG Daemons
        os.system('killall ecpprog')
        #subprocess.Popen([ECPPROG, '-I', 'A', '-D', '6000'])
        #time.sleep(0.5)
        #subprocess.Popen([ECPPROG, '-I', 'B', '-D', '5000'])
        #time.sleep(0.5)

        self.tester = Tester()
        self.dut = DeviceUnderTest()    
        self.reset_variables()

    def shutdown(self):
        # Turn off power
        self.tester.user_set_io(0)

    def dut_off(self):
        # Turn off power
        self.tester.user_set_io(0)

    def reset_variables(self):
        self.proto = False
        self.flashid = 0
        self.unique = 0
        self.lot = 0
        self.wafer = 0
        self.x_pos = 0
        self.y_pos = 0
        self.extra = 0
        self.supply = 0
        self.current = 0
        self.refclk = 0
        self.osc = 0
        self.voltages = [ 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A' ]
        self.revision = 0
        self.time_since_battery = 0

    def test_000_boot_current(self):
        """Bootup Current Draw"""
        self.tester.user_set_io(0x30) # Turn on DUT from both 'sides'
        time.sleep(0.2) # Let power regulators come up
        self.dut.ecp_clear_fpga()
        time.sleep(0.2) # Check Power supplt after some time

        supply = self.tester.read_adc_channel('Supply', 5)
        self.supply = supply # For later use
        if supply < 5.0:
            TestFailCritical('Tester Supply Voltage Low')

        curr = self.tester.read_adc_channel('Cartridge Current', 5) + self.tester.read_adc_channel('MicroUSB Current', 5)
        self.current = curr # For later use
        logger.info(f"After CLEAR FPGA: {curr:.1f} mA")
        if curr > 250.:   # Unfortunately this is including USB sticks and network loopback attached!
            TestFailCritical('Board Current too high')

    def test_001_regulators(self):
        """Voltage Regulators"""
        v50 = self.tester.read_adc_channel('+5.0V', 5)
        v43 = self.tester.read_adc_channel('+4.3V', 5)
        v33 = self.tester.read_adc_channel('+3.3V', 5)
        v25 = self.tester.read_adc_channel('+2.5V', 5)
        v18 = self.tester.read_adc_channel('+1.8V', 5)
        v11 = self.tester.read_adc_channel('+1.1V', 5)
        v09 = self.tester.read_adc_channel('+0.9V', 5)

        ok = True
        if not (0.855 <= v09 <= 0.945) and not self.proto:
            logger.error("v09 out of range")
            ok = False
        if not (1.04 <= v11 <= 1.16):
            logger.error("v11 out of range")
            ok = False
        if not (1.71 <= v18 <= 1.89):
            logger.error("v18 out of range")
            ok = False
        if not (2.375 <= v25 <= 2.625):
            logger.error("v25 out of range")
            ok = False
        if not (3.135 <= v33 <= 3.465) and not self.proto:
            logger.error("v33 out of range")
            ok = False
        if not (4.085 <= v43 <= 4.515):
            logger.error("v43 out of range")
            ok = False
        if not (4.5 <= v50 <= self.supply) and not self.proto:
            logger.error("v50 out of range")
            ok = False

        if self.proto:
            self.voltages = [ 'N/A', f'{v43:.2f} V', 'N/A', f'{v25:.2f} V', f'{v18:.2f} V', f'{v11:.2f} V', 'N/A' ]
        else:
            self.voltages = [ f'{v50:.2f} V', f'{v43:.2f} V', f'{v33:.2f} V', f'{v25:.2f} V', f'{v18:.2f} V', f'{v11:.2f} V', f'{v09:.2f} V' ]

        if not ok:
            self.tester.report_adcs()
            raise TestFailCritical('One or more regulator voltages out of range')

    def test_019_unique_id(self):
        """Unique ID"""
        if self.dut.ecp_read_id() != 0x41111043:
            raise TestFailCritical("FPGA on DUT not recognized")

        (code, lot, wafer, x, y, extra) = self.dut.ecp_read_unique_id()
        self.unique = code
        self.lot = lot
        self.wafer = wafer
        self.x_pos = x
        self.y_pos = y
        self.extra = extra

    def test_002_power_switchover(self):
        """Power Switchover Diodes"""
        # Switch to MicroUSB supply mode
        self.tester.user_set_io(0x20)
        v18 = self.tester.read_adc_channel('+1.8V', 5)
        if v18 < 1.6:
            raise TestFail('Power went off when switching to MicroUSB power')
        curr = self.tester.read_adc_channel('Cartridge Current', 5)
        if curr > 5.0:
            raise TestFail(f'Current flow from Cartridge Supply {curr:.0f} mA')
        self.tester.user_set_io(0x30)
        # Switch to Cartridge Power Mode
        self.tester.user_set_io(0x10)
        v18 = self.tester.read_adc_channel('+1.8V', 5)
        if v18 < 1.7:
            raise TestFail('Power went off when switching to Cartridge power')
        curr = self.tester.read_adc_channel('MicroUSB Current', 5)
        if curr > 5.0:
            raise TestFail(f'Current flow from MicroUSB Supply {curr:.0f} mA')
        self.tester.user_set_io(0x30)

    def test_003_test_fpga(self):
        """FPGA Detection & Load"""
        if self.dut.ecp_read_id() != 0x41111043:
            raise TestFailCritical("FPGA on DUT not recognized")

        self.dut.ecp_load_fpga(dut_fpga)

        if self.dut.user_read_id() != 0xdead1541:
            raise TestFailCritical("DUT: User JTAG not working. (bad ID)")
        
    def test_015_leds(self):
        """LED Presence"""
        for i in range(3):
            self.dut.user_set_io(0x80)  # Put local CPU in reset
            self.tester.user_set_io(0x10) # Turn on DUT power only through Cartridge
            time.sleep(0.2)
            led0 = self.tester.read_adc_channel('Cartridge Current', 10)
            self.dut.user_set_io(0x81)
            led1 = self.tester.read_adc_channel('Cartridge Current', 10)
            self.dut.user_set_io(0x83)
            led2 = self.tester.read_adc_channel('Cartridge Current', 10)
            self.dut.user_set_io(0x87)
            led3 = self.tester.read_adc_channel('Cartridge Current', 10)
            self.dut.user_set_io(0x8F)
            led4 = self.tester.read_adc_channel('Cartridge Current', 10)
            self.dut.user_set_io(0x00)  # Take CPU out of reset
            if (led4 > led3) and (led3 > led2) and (led2 > led1) and (led1 > led0):
                logger.info("LEDs OK!")
                return
            else:
                logger.debug(f"Led currents: {led0:.1f} {led1:.1f} {led2:.1f} {led3:.1f} {led4:.1f} mA")

        raise TestFail("LEDs not properly detected.")

    def test_004_ddr2_memory(self):
        """DDR2 Memory Test"""
        # bootloader should have run by now
        time.sleep(0.5)
        text = self.dut.user_read_console(True)
        if "RAM OK!!" not in text:
            raise TestFailCritical("Memory calibration failed.")

        random = {} # Map of random byte blocks
        for i in range(6,26):
            random[i] = np.random.bytes(64)

        for i in range(6,26):
            addr = 1 << i
            logger.debug(f"Writing Addr: {addr:x}")
            self.dut.user_write_memory(addr, random[i])

        for i in range(6,26):
            addr = 1 << i
            logger.debug(f"Reading Addr: {addr:x}")
            rb = self.dut.user_read_memory(addr, 64)
            if rb != random[i]:
                logger.debug(random[i].hex())
                logger.debug(rb.hex())
                raise TestFailCritical('Verify error on DDR2 memory')

    def test_018_frequencies(self):
        """Crystal Accuracy"""
        val = 0
        for i in range(10):
            clk1 = self.dut.user_read_io(0x100400, 16)
            f = [0, 0, 0, 0]
            (f[0], f[1], f[2], f[3]) = struct.unpack("<LLLL", clk1)
            # logger.debug(str(f))
            for i in f:
                if i & 0x1000000 != 0:
                    val = i & 0xFFFFFF
                    break
            else:
                time.sleep(0.2)
                continue
            break
        freq = (val & 0xFFFFFF) / 65536
        self.refclk = freq
        self.ppm = 1e6 * ((freq / 50.) - 1.)
        logger.info(f"Frequency: {freq:.6f} MHz")
        logger.info(f"Diff: {self.ppm:.1f} ppm")

        for i in range(10):
            clk2 = self.dut.user_read_io(0x100500, 16)
            f = [0, 0, 0, 0]
            (f[0], f[1], f[2], f[3]) = struct.unpack("<LLLL", clk2)
            for i in f:
                if i & 0x1000000 != 0:
                    val = i & 0xFFFFFF
                    break
            else:
                time.sleep(0.2)
                continue
            break
        freq = (val & 0xFFFFFF) / 65536
        self.osc = freq
        logger.info(f"Oscillator: {freq:.6f} MHz")

        if abs(self.ppm) > 120.:
            raise TestFail("Reference frequency out of range.")

    def test_020_board_revision(self):
        "Board Revision"
        self.revision = int(self.dut.user_read_io(0x10000c, 1)[0]) >> 3
        self.dut.user_write_io(0x60208, b'\x03')
        self.dut.user_write_io(0x60200, b'\xFF')
        self.dut.user_write_io(0x60208, b'\x01')
        self.dut.user_write_io(0x60200, b'\x4B')
        self.dut.user_write_io(0x60200, b'\x00\x00\x00\x00')
        idbytes = self.dut.user_read_io(0x60200, 4)
        idbytes += self.dut.user_read_io(0x60200, 4)
        self.dut.user_write_io(0x60208, b'\x03')
        logger.info(f"FlashID = {idbytes.hex()}")
        self.flashid = struct.unpack(">Q", idbytes)[0]

    def test_005_start_app(self):
        """Run Application on DUT"""
        self.dut.user_upload(dut_appl, 0x100)
        self.dut.user_run_app(0x100)
        time.sleep(0.5)
        text = self.dut.user_read_console()
        if "DUT Main" not in text:
            raise TestFailCritical('Running test application failed')

    def _test_006_buttons(self):
        """Button Test"""
        logger.warning("Press each button!")
        (result, console) = self.dut.perform_test(TEST_BUTTONS, max_time = 60)
        logger.debug(f"Console Output:\n{console}")
        if result != 0:
            raise TestFail(f'Fault in buttons. Err = {result}')

    def test_007_ethernet(self):
        """Ethernet"""
        (result, console) = self.dut.perform_test(TEST_SEND_ETH)
        logger.debug(f"Console Output:\n{console}")
        if result != 0:
            raise TestFail(f"Couldn't send Ethernet Packet. Err = {result}")
        (result, console) = self.dut.perform_test(TEST_RECV_ETH)
        logger.debug(f"Console Output:\n{console}")
        if result != 0:
            raise TestFail(f"Didn't receive Ethernet Packet. Err = {result}")

    def test_008_usb_phy(self):
        """USB PHY Detection"""
        (result, console) = self.dut.perform_test(TEST_USB_PHY)
        logger.debug(f"Console Output:\n{console}")
        if result != 0:
            raise TestFail(f"Couldn't find USB PHY (USB3310) Err = {result}")

    def test_009_usb_hub(self):
        """USB HUB Detection"""
        (result, console) = self.dut.perform_test(TEST_USB_INIT)
        logger.debug(f"Console Output:\n{console}")
        time.sleep(1)
        (result, console) = self.dut.perform_test(TEST_USB_HUB)
        logger.debug(f"Console Output:\n{console}")
        if result != 0:
            raise TestFail(f"Couldn't find USB HUB (USB2503) Err = {result}")

    def _test_010_usb_sticks(self):
        """USB Sticks Detection"""
        time.sleep(4)
        (result, console) = self.dut.perform_test(TEST_USB_PORTS)
        logger.debug(f"Console Output:\n{console}")
        if result != 0:
            raise TestFail(f"Couldn't find (all) USB sticks Err = {result}")
        (_result, console) = self.dut.perform_test(TEST_USB_SHOW)
        logger.debug(f"Console Output:\n{console}")

    def test_011_rtc(self):
        """Real Time Clock"""
        #vbatt = self.tester.read_adc_channel('VBatt', 4)
        #if vbatt < 2.8:
        #    raise TestFail(f"Real Time Clock fail! Vbatt Low! {vbatt:.2f}V")

        (result, console) = self.dut.perform_test(TEST_RTC_ACCESS)
        logger.debug(f"Console Output:\n{console}")
        if result != 0:
            raise TestFail(f"Couldn't access Real Time Clock. Err = {result}")

        self.time_since_battery = self.dut.get_seconds_after_epoch()

        # Write current time in RTC
        self.dut.write_current_time()

    def test_012_audio(self):
        """Audio In / Out"""
        logger.info("Generating sine wave")
        scale = math.pow(2.0, 31) * 0.9
        data = b''
        for i in range(192):
            phase = i * (2. * math.pi / 48.) # 1000 Hz
            sampleL = scale * math.sin(phase)
            phase = i * (2. * math.pi / 64.) # 750 Hz
            sampleR = scale * math.sin(phase)
            data += struct.pack("<ll", int(sampleL), int(sampleR))

        ## data should now have 192 samples
        logger.info("Uploading sound.")
        self.dut.user_write_memory(0x1100000, data)

        # Now let's enable this sound on the output
        regs = struct.pack("<LLB", 0x1100000, 0x1100000 + 192*8, 3)
        self.dut.user_write_io(0x100210, regs)

        # Let's now also start recording (once ~4000 samples, which takes < 0.1 second to do
        regs = struct.pack("<LLB", 0x1000000, 0x1000000 + 4608*8, 1)
        self.dut.user_write_io(0x100200, regs)

        # Now download the data (assuming we will never be faster than writing the data)
        logger.info("Downloading audio data...")
        data = self.dut.user_read_memory(0x1000000, 4608 * 2 * 4)
        with open("audio.bin", 'wb') as fo:
            fo.write(data)

        (left_ampl, left_peak), (right_ampl, right_peak) = calc_fft("audio.bin", False)
        if left_ampl < 0.36:
            raise TestFail("Amplitude on left audio channel low.")
        if left_peak != 750.0:
            raise TestFail("Peak in spectrum not at 750 Hz")
        if right_ampl < 0.36:
            raise TestFail("Amplitude on right audio channel low.")
        if right_peak != 1000.0:
            raise TestFail("Peak in spectrum not at 1000 Hz")

    def test_021_iec(self):
        """IEC (Serial DIN)"""
        # IEC is only available on the dut.
        # Can only toggle each output and observe the input
        # While the pins are active low, the signals in the
        # FPGA are active high.
        patterns = b'\x00\x01\x02\x04\x08\x10\x1F\x1E\x1D\x1B\x17\x0F'
        errors = 0
        for i in patterns:
            wr = bytearray(1)
            wr[0] = i
            self.dut.user_write_io(0x100306, wr)
            rb = self.dut.user_read_io(0x100306, 1)
            if i != rb[0]:
                errors += 1

        # TODO: Report more accurately *which signal* is stuck at 0, stuck at 1, or shorted to another
        if errors > 0:
            raise TestFail("IEC local loopback failure.")

    def walking_bit_test_tester_to_dut(self, test_bits, test_continuity = True):
        # Set everything to input on dut
        # Bit 47 has to be set to 1, because that's BUFFER_EN
        mask = bytearray(6)
        for i in test_bits:
            mask[i//8] |= 1 << (i %8)

        zeros = bytearray(8)
        buf_en = bytearray(8)
        buf_en[5] = 0x80
        self.dut.user_write_io(0x100308, zeros)
        self.dut.user_write_io(0x100300, buf_en)

        # Set everything to output on tester
        ones = b'\xff' * 8
        self.tester.user_write_io(0x100308, ones)

        errors = 0
        ## Walking one test
        diffs = set()
        for i in test_bits:
            pattern = bytearray(6)
            pattern[i//8] |= 1 << (i %8)
            logger.debug("Writing: " + str(pattern.hex()))
            self.tester.user_write_io(0x100300, pattern)
            trb = self.tester.user_read_io(0x100300, 6)
            if (bitwise_and(trb, pattern) != pattern):
                logger.error(f"BIT {pio_names[i]} is stuck hard!")
                errors += 1
            elif (bitwise_and(trb, mask) != pattern):
                logger.error(f"Local short from {pio_names[i]} to others: {pattern} vs {trb}")
                errors += 1

            if not test_continuity:
                continue

            rb = bytearray(self.dut.user_read_io(0x100300, 6))
            rb2 = bitwise_and(rb, mask)
            logger.debug("Read:    " + rb.hex() + str(find_ones(rb2)))
            diff = bytearray(6)
            for i in range(6):
                diff[i] = pattern[i] ^ rb2[i]
            logger.debug("Diff:    " + diff.hex() + str(find_ones(diff)))
            diffs.update(find_ones(diff))
            if (rb2 != pattern):
                errors += 1

        # Walking zero test
        for i in test_bits:
            pattern = bytearray(b'\xFF\xFF\xFF\xFF\xFF\xFF')
            single_bit = bytearray(6)
            pattern[i//8] &= ~(1 << (i %8))
            single_bit[i//8] |= (1 << (i % 8))
            logger.debug("Writing: " + pattern.hex())
            self.tester.user_write_io(0x100300, pattern)
            trb = bytearray(self.tester.user_read_io(0x100300, 6))
            if (bitwise_ornot(trb, single_bit) != pattern):
                logger.error(f"BIT {pio_names[i]} is stuck hard!")
                errors += 1
            elif (bitwise_ornot(trb, mask) != pattern):
                logger.error(f"Local short from {pio_names[i]} to others: {pattern} vs {trb}")
                errors += 1
            if not test_continuity:
                continue

            rb = bytearray(self.dut.user_read_io(0x100300, 6))
            rb2 = bitwise_ornot(rb, mask)
            logger.debug("Read:    " + rb.hex() + str(find_zeros(rb2)))
            diff = bytearray(6)
            for i in range(6):
                diff[i] = pattern[i] ^ rb2[i]
            logger.debug("Diff:    " + diff.hex() + str(find_ones(diff)))
            diffs.update(find_ones(diff))
            if (rb2 != pattern):
                errors += 1

        self.tester.user_write_io(0x100308, zeros)
        self.tester.user_read_console(True)

        logger.info(f"Errors: {errors}")
        pins = [pio_names[x] for x in diffs]
        if len(pins) > 0:
            logger.warning(str(pins))

        return errors

    def walking_bit_test_dut_to_tester(self, test_bits, test_continuity = True):
        # Prepare mask
        mask = bytearray(8)
        for i in test_bits:
            mask[i//8] |= 1 << (i %8)

        # Set everything to input on tester
        zeros = bytearray(8)
        self.tester.user_write_io(0x100308, zeros)

        # Set everything to output on dut
        ones = b'\xff' * 8
        buf_en = bytearray(8)
        buf_en[5] = 0x80
        self.dut.user_write_io(0x100300, buf_en)
        self.dut.user_write_io(0x100308, ones)

        errors = 0
        diffs = set()

        # Walking one test; from dut to tester
        for i in test_bits:
            pattern = bytearray(6)
            pattern[5] |= 0x80 # Buffer enable
            pattern[i//8] |= 1 << (i % 8)
            logger.debug("Writing: " + pattern.hex())
            self.dut.user_write_io(0x100300, pattern)
            pattern[5] &= 0x7F # Remove buffer enable again

            trb = bytearray(self.dut.user_read_io(0x100300, 6))
            if (bitwise_and(trb, pattern) != pattern):
                logger.error(f"BIT {pio_names[i]} is stuck hard!")
                errors += 1
            elif (bitwise_and(trb, mask) != pattern):
                logger.error(f"Local short from {pio_names[i]} to others: {pattern} vs {trb}")
                errors += 1

            if not test_continuity:
                continue

            rb = bytearray(self.tester.user_read_io(0x100300, 6))
            rb2 = bitwise_and(rb, mask)
            logger.debug("Read:    " + rb.hex() + str(find_ones(rb2)))
            diff = bytearray(6)
            for i in range(6):
                diff[i] = pattern[i] ^ rb2[i]
            logger.debug("Diff:    " + diff.hex() + str(find_ones(diff)))
            diffs.update(find_ones(diff))
            if (rb2 != pattern):
                errors += 1

        logger.info(f"Errors: {errors}")
        pins = [pio_names[x] for x in diffs]
        if len(pins) > 0:
            logger.warning(str(pins))

        self.dut.user_write_io(0x100308, zeros)
        return errors

    def test_022_cartio_bottom(self):
        """Cartridge I/O (Bottom Row)"""
        errors = self.walking_bit_test_tester_to_dut(pio_bottom)
        errors += self.walking_bit_test_dut_to_tester(pio_bottom_out)
        if errors > 0:
            raise TestFail("Cartridge Bottom Row failure.")

    def test_023_cartio_top(self):
        """Cartridge I/O (Top Row)"""
        errors = self.walking_bit_test_tester_to_dut(pio_top, False)
        errors += self.walking_bit_test_dut_to_tester(pio_top, False)
        if errors > 0:
            raise TestFail("Cartridge Top Row failure.")

    def test_024_cassette_pins(self):
        """Cassette Pins"""
        errors = self.walking_bit_test_tester_to_dut(pio_cassette, False)
        errors = self.walking_bit_test_dut_to_tester(pio_cassette, False)
        if errors > 0:
            raise TestFail("Cassette Pins failure.")

    def _test_016_speaker(self):
        """Speaker Amplifier"""
        logger.info("Generating sine wave")
        scale = math.pow(2.0, 31) * 0.99
        data = b''
        for i in range(192):
            phase = i * (2. * math.pi / 192.) # 250 Hz
            sampleL = scale * math.sin(phase)
            phase = i * (2. * math.pi / 96.) # 500 Hz
            sampleR = scale * math.sin(phase)
            data += struct.pack("<ll", int(sampleL), int(sampleR))

        ## data should now have 192 samples
        logger.info("Uploading sound.")
        self.dut.user_write_memory(0x1100000, data)

        # Now let's enable this sound on the output
        regs = struct.pack("<LLB", 0x1100000, 0x1100000 + 192*8, 3)
        self.dut.user_write_io(0x100210, regs)

        # Now let's enable this sound on the output, by writing '1' to speaker enable
        # The oscillator should already be running.
        self.dut.user_write_io(0x10000C, b'\x01')

        # Let's now also start recording (once ~4000 samples, which takes < 0.1 second to do
        # This is done on the tester; not on the dut!
        regs = struct.pack("<LLB", 0x8000, 0x8000 + 4096*4, 1)
        self.tester.user_write_io(0x100200, regs)

        # Now download the data (assuming we will never be faster than writing the data)
        logger.info("Downloading audio data...")
        data = self.tester.user_read_memory(0x8000, 4096 * 4)
        with open("speaker.bin", 'wb') as fo:
            fo.write(data)

        (ampl, peak) = calc_fft_mono("speaker.bin", False)
        
        if ampl < 0.22:
            calc_fft_mono("speaker.bin", True)
            raise TestFail(f"Amplitude on speaker channel low. {ampl}")
        if int(peak) != 246:
            calc_fft_mono("speaker.bin", True)
            raise TestFail(f"Peak in spectrum not at 250 Hz {peak}")

    def program_flash(self, cb = [None, None, None]):
        """Program Flash!"""
        # Program the flash in three steps: 1) FPGA, 2) Application, 3) FAT Filesystem
        self.dut.flash_callback = cb[0]
        self.dut.ecp_prog_flash(final_fpga, 0)
        self.dut.flash_callback = cb[1]
        self.dut.ecp_prog_flash(final_appl, 0xA0000)
        self.dut.flash_callback = cb[2]
        self.dut.ecp_prog_flash(final_fat, 0x200000)

    def program_tester(self, cb = None ):
        """Program Tester!"""
        self.dut.flash_callback = cb
        self.dut.ecp_prog_flash(tester_fpga, 0)

    def late_099_boot(self):
        """Boot Test"""
        logger.info("Let's see if the unit boots...")
        self.tester.user_set_io(0x00) # Turn on DUT off
        time.sleep(0.5) 
        text = self.tester.user_read_console2(False)
        self.tester.user_set_io(0x30) # Turn on DUT from both 'sides'
        time.sleep(1.5) 
        text = self.tester.user_read_console2(True)
        self.tester.user_set_io(0x00) # Turn on DUT off
        logger.debug(f"Board replied:\n {text}")
        return "ConfigManager" in text

    def run_all(self):
        self.startup()
        all = [
            self.test_000_boot_current,
            self.test_001_regulators,
            self.test_002_power_switchover,
            self.test_003_test_fpga,
            self.test_015_leds,
            self.test_019_unique_id,
            self.test_004_ddr2_memory,
            self.test_005_start_app,
            self.test_007_ethernet,
            self.test_018_frequencies,
            self.test_012_audio,
            self.test_016_speaker,
            self.test_006_buttons,
            self.test_008_usb_phy,
            self.test_009_usb_hub,
            self.test_010_usb_sticks,
            self.test_022_cartio_bottom,
            self.test_024_cassette_pins,
            self.test_011_rtc,
        ]
        for test in all:
            try:
                test()
            except TestFail as e:
                logger.error(f"Test failed: {e}")
            except JtagClientException as e:
                logger.critical(f"JTAG Communication error: {e}")
                return

        self.shutdown()

    def get_all_tests(self):
        di = self.__class__.__dict__
        funcs = {}
        for k in di.keys():
            if k.startswith("test"):
                funcs[k] = di[k]
            if k.startswith("late"):
                funcs[k] = di[k]
        return funcs

    @staticmethod
    def add_log_handler(ch):
        global logger
        logger.addHandler(ch)


if __name__ == '__main__':
    tests = UltimateIIPlusLatticeTests()
    tests.startup()

