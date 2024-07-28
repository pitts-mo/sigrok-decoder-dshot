## Modified from rgb_led_ws281x - original license below:
## Copyright (C) 2023: hyp0dermik@gmail.com

##
## This file is part of the libsigrokdecode project.
##
## Copyright (C) 2016 Vladimir Ermakov <vooon341@gmail.com>
##
## This program is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 3 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program; if not, see <http://www.gnu.org/licenses/>.
##

import sigrokdecode as srd
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

class SamplerateError(Exception):
    pass

class State(Enum):
    CMD = 1,
    TELEM = 2

class State_Telem(Enum):
    START = 1,
    RECV = 2

class Decoder(srd.Decoder):
    api_version = 3
    id = 'dshot'
    name = 'DShot'
    longname = 'DShot RC Hobby Motor Protcol Decoder'
    desc = 'DShot RC Hobby Motor Protcol Decoder'
    license = 'gplv3+'
    inputs = ['logic']
    outputs = []
    tags = ['Display', 'IC']
    channels = (
        {'id': 'din', 'name': 'DIN', 'desc': 'DIN data line'},
    )

    options = (
        {'id': 'dshot_rate', 'desc': 'DShot Rate', 'default': '150','values': ('150', '300','600','1200')},
        { 'id': 'bidir', 'desc': 'Bidirectional DShot','default': 'True', 'values': ('True', 'False')},
        { 'id': 'log', 'desc': 'Write log file','default': 'no', 'values': ('yes', 'no')},
        {'id': 'edt_force', 'desc': 'Force EDT as telem type', 'default': 'no', 'values': ('True', 'False')},
    )
    annotations = (
        ('bit', 'Bit'),
        ('cmd', 'Command'),
        ('throttle', 'Throttle'),
        ('checksum', 'CRC'),
        ('errors', 'Errors'),
        ('telem_bit', 'Telem Bit'),
        ('telem_erpm', 'Telem ERPM'),
        ('telem_edt', 'Telem EDT'),
        ('telem_errors', 'Telem Errors'),
        ('telem_error2', 'Telem Errors2'),

    )
    annotation_rows = (
        ('bits', 'Bits', (0,)),
        ('dshot_data', 'DShot Data', (1,2,3)),
        ('dshot_errors', 'Dshot Errors', (4,)),
        ('telem_bits', 'Telem Bits', (5,)),
        ('dshot_telem_erpm', 'Dshot Telem ERPM', (6,)),
        ('dshot_telem_edt', 'Dshot Telem', (7,)),
        ('dshot_telem_errors', 'Dshot Errors', (8,)),
        ('dshot_telem_errors2', 'Dshot Errors', (9,)),
    )

    #dshot_period_lookup = {'150': 6.67e-6, '300': 3.33e-6,'600':1.67e-6,'1200':0.83e-6}


    def __init__(self):
        self.reset()

    def reset(self):
        self.state = State.CMD
        self.samplerate = None

        self.debug = False

        self.inreset = False
        self.bidirectional = False
        self.dshot_kbaud = 300e3
        self.dshot_period = 3.33e-6
        self.actual_period = None
        self.halfbitwidth = None
        self.currbit_ss = None
        self.currbit_es = None
        self.samples_after_motorcmd = None
        self.samples_pp = None

        self.telem_start = None
        self.state_telem = State_Telem.START
        self.telem_baudrate_midpoint = 0
        self.edt_force = False

    def start(self):
        self.bidirectional = True if self.options['bidir'] == 'True' else False
        self.edt_force = True if self.options['edt_force'] == 'True' else False
        self.dshot_kbaud = int(self.options['dshot_rate'])*1000
        self.dshot_period = 1/self.dshot_kbaud
        self.samples_pp =  int(self.samplerate*self.dshot_period)
        self.samples_after_motorcmd = self.samples_pp * 3
        self.samples_after_telempkt = self.samples_pp * 3

        self.out_ann = self.register(srd.OUTPUT_ANN)
        self.telem_baudrate_midpoint = int((self.samplerate / (self.dshot_kbaud*(5/4))) / 2.0)
        if self.debug:
            print("telem_midpoint",self.telem_baudrate_midpoint)

    def metadata(self, key, value):
        if key == srd.SRD_CONF_SAMPLERATE:
            self.samplerate = value

    def handle_bits_dshot(self, results):
        #ss, es, bit
        bits = [result[2] for result in results]

        if len(bits) == 16:
            dshot_value = int(reduce(lambda a, b: (a << 1) | b, bits[:11]))
            telem_request = bits[11]
            received_crc = int(reduce(lambda a, b: (a << 1) | b, bits[12:]))
        
            value_tocrc = int(reduce(lambda a, b: (a << 1) | b, bits[:12]))

            if self.bidirectional:
                calculated_crc = int((~(value_tocrc ^ (value_tocrc >> 4) ^ (value_tocrc >> 8)))&0x0F)
            else:
                calculated_crc = int(((value_tocrc ^ (value_tocrc >> 4) ^ (value_tocrc >> 8)))&0x0F)

            if received_crc == calculated_crc:
                crc_ok = True 
            else:
                crc_ok = False

            # TODO: Align this correctly
            crc_startsample = results[12][0]
            
            # Split annotation based on value type
            if dshot_value < 48:
                # Command
                self.put(results[0][0], crc_startsample, self.out_ann,
                        [1, ['%04d' % dshot_value]])
            else:
                # Throttle
                 self.put(results[0][0], crc_startsample, self.out_ann,
                        [2, ['%04d' % dshot_value]])

            self.put(crc_startsample, results[15][1], self.out_ann, [3, ['Calc CRC: '+('%04d' % calculated_crc)+' TXed CRC:'+('%04d' % received_crc)]])
            if not crc_ok:
                self.put(crc_startsample, results[15][1], self.out_ann,
                     [4, ['CRC INVALID']])
            return True
        else:
            return False

    def handle_bit_dshot(self, ss, es, nb_ss):
        period = nb_ss - ss
        duty = es - ss
        # Ideal duty for T0H: 33%, T1H: 66%.
        bit_ = (duty / period) > 0.5

        self.put(ss, nb_ss, self.out_ann,
        [0, ['%d' % bit_]])
        return [ss,nb_ss,bit_]

    def handle_telem_bit(self,matched):
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

    def process_telem_erpm(self,packet,start,end):
        # Raw packet
        self.put(start,
                 end, self.out_ann,
                 [6, ['%23s' % bin(packet)]])
        # XOR with next?
        packet &= 0x0FFFFF
        packet = (packet^(packet>>1))
        self.put(start,
                 end, self.out_ann,
                 [7, ['%23s' % bin(packet)]])
        # Undo GCR
        output = 0b0

        nibbles = 4
        bitmask = 0b11111 << ((nibbles-1)*5)

        for n in range(nibbles):
            gcr_n = bitmask & packet
            ungcr = gcr_tables[bin(gcr_n >> (nibbles - (n + 1)) * 5)]
            output = (output << 4) | ungcr
            bitmask = (bitmask >> 5)

        # Compare CRC
        crc_received = output & 0xF
        output = (output >> 4) & 0xFFF
        crc_calc = ~((output ^ (output >> 4) ^ (output >> 8))) & 0x0F



        self.put(end - ((self.telem_baudrate_midpoint * 2)*4),
                 end, self.out_ann,
                 [7, ['%23s' % ("RX CRC: "+hex(crc_received)+" Calc CRC: "+hex(crc_calc))]])
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
    def process_telem_edt(self,packet,start,end):

        return
    def process_telem(self,packet,start,end):
        if self.edt_force:
            self.process_telem_edt(packet,start,end)
        else:
            self.process_telem_erpm(packet,start,end)

        return




    def decode(self):
        if not self.samplerate:
            raise SamplerateError('Cannot decode without samplerate.')
        
        results = []
        telem = 0b0
        tlm_start = 0
        while True:

            match self.state:
                case State.CMD:
                    if not self.bidirectional:
                        pins = self.wait([{0: 'r'}, {0: 'f'}, {'skip': self.samples_after_motorcmd}])
                    else:
                        pins = self.wait([{0: 'f'}, {0: 'r'}, {'skip': self.samples_after_motorcmd}])
                    #TODO: Increase skip to maximum time for effiency
                    #TODO: Mark any changes in this time as errors?  Option to reduce load?

                    if self.currbit_ss and self.currbit_es and self.matched[2]:
                        # Assume end of packet if have seen start and end of a potential bit but no further change within 3 periods
                        # TODO: Confirm wait period this works with spec
                        results += [self.handle_bit_dshot(self.currbit_ss, self.currbit_es,
                                                          (self.currbit_ss + self.samples_pp))]
                        self.currbit_ss = None
                        self.currbit_es = None

                        # Pass results to decoder
                        result = self.handle_bits_dshot(results)
                        if result and self.bidirectional:
                            self.state = State.TELEM
                        results = []

                    if self.matched[0] and not self.currbit_ss and not self.currbit_es:
                        # Start of bit
                        self.currbit_ss = self.samplenum
                    elif self.matched[1] and self.currbit_ss and not self.currbit_es:
                        # End of bit
                        self.currbit_es = self.samplenum
                    elif self.matched[0] and self.currbit_es and self.currbit_ss:
                        # Have complete bit, can handle bit now
                        result = [self.handle_bit_dshot(self.currbit_ss, self.currbit_es, self.samplenum)]
                        # print(result)
                        results += result
                        self.currbit_ss = self.samplenum
                        self.currbit_es = None
                case State.TELEM:
                    match self.state_telem:
                        case State_Telem.START:
                            # First wait for falling edge (idle high)
                            pins = self.wait([{0: 'f'}])
                            # Save start pulse
                            tlm_start = self.samplenum
                            # Switch to receiving state
                            self.state_telem = State_Telem.RECV
                            # TODO: Check if still low after 1/8 bitlength for error det?
                        case State_Telem.RECV:
                            # First conditions skips half bit width and matches low
                            # Second condition skips half bit width and matches high
                            pins = self.wait([{0: 'l', 'skip': self.telem_baudrate_midpoint},
                                              {0: 'h', 'skip': self.telem_baudrate_midpoint}])

                            # Append next bit
                            curr_bit = self.handle_telem_bit(self.matched)
                            self.put(self.samplenum - self.telem_baudrate_midpoint,
                                     self.samplenum + self.telem_baudrate_midpoint,
                                     self.out_ann,
                                     [5, ['%04d' % curr_bit]])
                            telem = telem | curr_bit

                            # Skip half bitwidth to end of bit
                            pins = self.wait([{'skip': self.telem_baudrate_midpoint}])

                            if telem.bit_length() >= 20-1:
                                self.process_telem(telem,tlm_start,self.samplenum)

                                # Reset
                                telem = 0b0
                                self.state_telem = State_Telem.START
                                self.state = State.CMD
                            else:
                                # Shift for next bit
                                telem = telem << 1
                            # If not mark as error

                            # Then skip x samples and sample
                            # Repeat for 21 bits (TBC)

            #TODO: What happens if it gets stuck in the wrong state?




            
             




