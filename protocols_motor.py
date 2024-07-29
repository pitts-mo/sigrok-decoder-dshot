from functools import reduce
from enum import Enum

class Packet():
    def __init__(self):
        self.ss = None
        self.es = None

    def samples(self):
        return self.es-self.ss
class DshotProtocol(Packet):
    def __init__(self):
        super().__init__()

        self.bidirectional = None
        self.results = None

        self.dshot_value = None
        self.telem_request = None
        self.received_crc = None
        self.calculated_crc = None
        return

    def handle_bits_dshot(self,results):
        # ss, es, bit
        self.results = results
        bits = [result[2] for result in self.results]

        if len(bits) != 16:
            return False

        self.dshot_value = int(reduce(lambda a, b: (a << 1) | b, bits[:11]))
        self.telem_request = bits[11]
        self.received_crc = int(reduce(lambda a, b: (a << 1) | b, bits[12:]))

        value_tocrc = int(reduce(lambda a, b: (a << 1) | b, bits[:12]))

        if self.bidirectional:
            self.calculated_crc = int((~(
                    value_tocrc ^ (value_tocrc >> 4) ^ (value_tocrc >> 8))) & 0x0F)
        else:
            self.calculated_crc = int(((value_tocrc ^ (value_tocrc >> 4) ^ (value_tocrc >> 8))) & 0x0F)

        if not self.received_crc == self.calculated_crc:
            return False
        return True
            # TODO: Align this correctly

    def handle_bit_dshot(self, ss, es, nb_ss):
        period = nb_ss - ss
        duty = es - ss
        # Ideal duty for T0H: 33%, T1H: 66%.
        bit_ = (duty / period) > 0.5

        #self.put(ss, nb_ss, self.out_ann, [0, ['%d' % bit_]])
        return [ss, nb_ss, bit_]

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