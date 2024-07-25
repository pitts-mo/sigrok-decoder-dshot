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
    0b11001: 0x0,
    0b11011: 0x1,
    0b10010: 0x2,
    0b10011: 0x3,
    0b11101: 0x4,
    0b10101: 0x5,
    0b10110: 0x6,
    0b10111: 0x7,
    0b11010: 0x8,
    0b01001: 0x9,
    0b01010: 0xa,
    0b01011: 0xb,
    0b11110: 0xc,
    0b01101: 0xd,
    0b01110: 0xe,
    0b01111: 0xf
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
        { 'id': 'bidir', 'desc': 'Bidirectional DShot','default': 'False', 'values': ('True', 'False')},
        { 'id': 'log', 'desc': 'Write log file','default': 'no', 'values': ('yes', 'no')},
    )
    annotations = (
        ('bit', 'Bit'),
        ('cmd', 'Command'),
        ('throttle', 'Throttle'),
        ('checksum', 'CRC'),
        ('errors', 'Errors'),

    )
    annotation_rows = (
        ('bits', 'Bits', (0,)),
        ('dshot_data', 'DShot Data', (1,2,3)),
        ('dshot_errors', 'Dshot Errors', (4,)),
    )

    dshot_period_lookup = {'150': 6.67e-6, '300': 3.33e-6,'600':1.67e-6,'1200':0.83e-6}

    def __init__(self):
        self.reset()

    def reset(self):
        self.state = State.CMD
        self.samplerate = None
        # self.oldpin = None
        # self.ss_packet = None
        # self.ss = None
        # self.es = None
        # self.bits = []
        self.inreset = False
        self.bidirectional = False
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

    def start(self):
        self.bidirectional = True if self.options['bidir'] == 'True' else False
        self.dshot_period = self.dshot_period_lookup[self.options['dshot_rate']]
        self.samples_pp =  int(self.samplerate*self.dshot_period)
        self.samples_after_motorcmd = self.samples_pp * 3
        self.samples_after_telempkt = self.samples_pp * 3
        # self.halfbitwidth = int((self.samplerate / self.dshot_period) / 2.0)
        #print("start period",self.dshot_period)
        self.out_ann = self.register(srd.OUTPUT_ANN)
        self.telem_baudrate_midpoint = int((self.samplerate / self.dshot_period*(5/4)) / 2.0)
        print("telem_midpoint",self.telem_baudrate_midpoint)

    def metadata(self, key, value):
        if key == srd.SRD_CONF_SAMPLERATE:
            self.samplerate = value

    def handle_bits_dshot(self, results):
        #ss, es, bit
        #print(results)
        bits = [result[2] for result in results]
        # print(bits)


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
         

            self.bits = []
            self.ss_packet = None
            return True
        else:
            return False
            #  self.put(results[0][0], results[-1::1][1], self.out_ann,
            #              [1, ['ERROR: INVALID PACKET LENGTH', 'ERR', 'E']])


    def handle_bit_dshot(self, ss, es, nb_ss):

        period = nb_ss - ss
        duty = es - ss
        # Ideal duty for T0H: 33%, T1H: 66%.
        bit_ = (duty / period) > 0.5

        self.put(ss, nb_ss, self.out_ann,
        [0, ['%d' % bit_]])
        return [ss,nb_ss,bit_]

    
    def decode(self):
        if not self.samplerate:
            raise SamplerateError('Cannot decode without samplerate.')
        
        results = []
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
                            self.telem_start = self.samplenum
                            self.state_telem = State_Telem.RECV
                            # Check if still low after 1/8 bitlength
                        case State_Telem.RECV:
                            pins = self.wait([{'skip': self.telem_baudrate_midpoint}])
                            # If not mark as error

                            # Then skip x samples and sample
                            # Repeat for 21 bits (TBC)

            #TODO: What happens if it gets stuck in the wrong state?




            
             




