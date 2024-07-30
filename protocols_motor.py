from functools import reduce
from enum import Enum

gcr_tables = {
    "0b11001": 0x0,
    "0b11011": 0x1,
    "0b10010": 0x2,
    "0b10011": 0x3,
    "0b11101": 0x4,
    "0b10101": 0x5,
    "0b10110": 0x6,
    "0b10111": 0x7,
    "0b11010": 0x8,
    "0b1001": 0x9,
    "0b1010": 0xa,
    "0b1011": 0xb,
    "0b11110": 0xc,
    "0b1101": 0xd,
    "0b1110": 0xe,
    "0b1111": 0xf
}

class Sequence():
    def __init__(self):
        self.ss = None
        self.ts = None
        self.es = None

    def samples(self):
        return self.es-self.ss

class BitDshot(Sequence):
    def __init__(self, ss, ts, es):
        super().__init__()
        self.period = None
        self.duty = None
        self.bit_ = None
        self.ss, self.ts, self.es = ss, ts, es
        self.process_bit()
        return
    def process_bit(self):

        self.period = self.es - self.ss
        self.duty = self.ts - self.ss
        # Ideal duty for T0H: 33%, T1H: 66%.
        self.bit_ = (self.duty / self.period) > 0.5
        # TODO: Add tolerance

    def __bool__(self):
        if self.bit_ is None:
            raise ValueError
        return self.bit_

class DshotSettings():
    def __init__(self):
        self.samplerate = 0
        #print(self.samplerate)
        self.bidirectional = False
        self.dshot_kbaud = 300e3
        self.dshot_period = None
        self.samples_after_motorcmd = None
        self.samples_after_telempkt = None
        self.samples_pp = None
        self.telem_baudrate_midpoint = 0
        self.telem_start = None
        self.edt_force = False
        self.update()
        return

    def update(self):
        self.dshot_period = 1 / self.dshot_kbaud
        self.samples_pp = int(self.samplerate * self.dshot_period)
        self.samples_after_motorcmd = self.samples_pp * 3
        self.samples_after_telempkt = self.samples_pp * 3
        self.telem_baudrate_midpoint = int((self.samplerate / (self.dshot_kbaud * (5 / 4))) / 2.0)

class DshotCommon():
    def __init__(self,settings_Dshot=DshotSettings()):
        self.cfg = settings_Dshot
        self.crc_recv = None
        self.crc_calc = None
        self.crc_ok = False

    def checkCRC(self,data):
        if self.cfg.bidirectional:
            # TODO: Move CRC out?
            self.crc_calc = int((~(data ^ (data >> 4) ^ (data >> 8))) & 0x0F)
        else:
            self.crc_calc = int(((data ^ (data >> 4) ^ (data >> 8))) & 0x0F)

        if not (self.crc_recv == self.crc_calc):
            self.crc_ok = False
            return False
        self.crc_ok = True
        return True

class DshotCmd(DshotCommon):
    def __init__(self,*args):
        super().__init__(*args)
        self.results = None
        self.dshot_value = None
        self.telem_request = None
        return
    def handle_bits_dshot(self,results):
        # ss, es, bit
        self.results = results
        if len(results) != 16:
            return False
        # Get bits only
        bits = [bool(result) for result in self.results]
        # Convert to binary from list
        bits = reduce(lambda a, b: (a << 1) | b, bits)
        # Seperate CRC
        self.crc_recv = bits & 0xF
        bits = bits >> 4
        # Remainder is data
        data = bits
        # Telem request
        self.telem_request = bits & 0x1
        # Rest is dshot value
        bits = bits >> 1
        self.dshot_value = bits

        if not self.checkCRC(data):
            return False


        return True
            # TODO: Align this correctly

class DshotTelem(DshotCommon):
    def __init__(self,*args):
        super().__init__(*args)
        self.results = None
        self.dshot_value = None
        self.telem_request = None
        return
    def handle_telem_bit(self, matched):
        # Low
        if matched == (True, False):
            # 0 value
            return 0

        # High
        if matched == (False, True):
            # 1 value
            return 1

    def process_telem_erpm(self, packet, start, end):
        # Raw packet
        #self.put(start, end, self.out_ann, [6, ['%23s' % bin(packet)]])
        # XOR with next?
        packet &= 0x0FFFFF
        packet = (packet ^ (packet >> 1))
        #self.put(start,end, self.out_ann, [7, ['%23s' % bin(packet)]])
        # Undo GCR
        output = 0b0

        nibbles = 4
        bitmask = 0b11111 << ((nibbles - 1) * 5)

        for n in range(nibbles):
            gcr_n = bitmask & packet
            ungcr = gcr_tables[bin(gcr_n >> (nibbles - (n + 1)) * 5)]
            output = (output << 4) | ungcr
            bitmask = (bitmask >> 5)

        # Compare CRC
        crc_received = output & 0xF
        output = (output >> 4) & 0xFFF
        crc_calc = ~((output ^ (output >> 4) ^ (output >> 8))) & 0x0F

        self.put(end - ((self.telem_baudrate_midpoint * 2) * 4),
                 end, self.out_ann,
                 [7, ['%23s' % ("RX CRC: " + hex(crc_received) + " Calc CRC: " + hex(crc_calc))]])
        if crc_calc != crc_received:
            self.put(end - ((self.telem_baudrate_midpoint * 2) * 4),
                     end, self.out_ann,
                     [8, ['%23s' % ("CRC ERROR!")]])
        # The upper 12 bit contain the eperiod (1/erps) in the following bitwise encoding:
        #
        # e e e m m m m m m m m m
        #
        # The 9 bit value M needs to shifted left E times to get the period in micro seconds.
        # This gives a range of 1 us to 65408 us. Which translates to a min e-frequency of 15.29 hz or for 14 pole motors 3.82 hz.
        return

    def process_telem_edt(self, packet, start, end):

        return

    def process_telem(self, packet, start, end):
        if self.edt_force:
            self.process_telem_edt(packet, start, end)
        else:
            self.process_telem_erpm(packet, start, end)

        return