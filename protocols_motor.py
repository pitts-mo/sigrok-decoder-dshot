from functools import reduce
from enum import Enum

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
        self.update()
        return

    def update(self):
        self.dshot_period = 1 / self.dshot_kbaud
        self.samples_pp = int(self.samplerate * self.dshot_period)
        self.samples_after_motorcmd = self.samples_pp * 3
        self.samples_after_telempkt = self.samples_pp * 3

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



    def handle_telem_bit(self, matched):
        # None to raise exception if no match
        result = None

        # Low
        if matched == (True, False):
            # 0 value
            result = 0

        # High
        elif matched == (False, True):
            # 1 value
            result = 1
        return result

    def process_telem_erpm(self, packet, start, end):
        # Raw packet
        self.put(start,
                 end, self.out_ann,
                 [6, ['%23s' % bin(packet)]])
        # XOR with next?
        packet &= 0x0FFFFF
        packet = (packet ^ (packet >> 1))
        self.put(start,
                 end, self.out_ann,
                 [7, ['%23s' % bin(packet)]])
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