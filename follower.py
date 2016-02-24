import pypruss
import numpy as np
import time
import struct
from collections import defaultdict
from ctypes import c_long
import Adafruit_BBIO.GPIO as GPIO
import matplotlib
matplotlib.use('GTKagg')
import matplotlib.pyplot as plt


class follower(object):

    PRU0_OFFSET_DRAM_HEAD = 0x1014
    PRU_MAX_SHORT_SAMPLES = 128*1024
    PRU_EVTOUT_0 = 3


    def __init__(self, pru = 0, pru0_fw="arm/pru00.bin", pru1_fw="arm/pru01.bin"):
        if pru == 0:
            pru_dataram = pypruss.PRUSS0_PRU0_DATARAM
        else:
            pru_dataram = pypruss.PRUSS0_PRU1_DATARAM

        self._spare = 0

        print "\tINFO: setting up power control line\n"
        GPIO.setup("P9_18",GPIO.OUT)

        print "\tINFO: pruss init\n"
        pypruss.init()						# Init the PRU

        print "\tINFO: pruss open\n"
        ret = pypruss.open(self.PRU_EVTOUT_0)

        print "\tINFO: pruss intc init\n"
        pypruss.pruintc_init()					# Init the interrupt controller

        print "\tINFO: mapping memory \n"
        self._data = pypruss.map_prumem(pru_dataram)
        
        print "\tINFO: data segment len=" + str(len(self._data)) + "\n"
        
        print "\tINFO: setting tail \n"
        self._tail = 0
        struct.pack_into('l', self._data, self.PRU0_OFFSET_DRAM_HEAD, self._tail)
        
        print "\tINFO: mapping extmem \n"
        self._extmem = pypruss.map_extmem()
        print "\tINFO: ext segment len=" + str(len(self._extmem))

        print "\tINFO: setup mem \n"
        self.ddrMem = pypruss.ddr_addr()
        print "V extram_base = " + hex(self.ddrMem) + "\n"
        
        print "\tINFO: loading pru00 code \n"
        pypruss.exec_program(0, pru0_fw)
        
        print "\tINFO: loading pru01 code \n"
        pypruss.exec_program(1, pru1_fw)

    def power_on(self=None):
        GPIO.output("P9_18",GPIO.HIGH)

    def power_off(self=None):
        GPIO.output("P9_18",GPIO.LOW)

    def read_uint(self, offset=0):
        return self.readData('L', offset, 1)[0]

    def readData(self, dtype='l', offset=0, samples = 1):
        return struct.unpack_from(dtype*samples, self._extmem, offset)

    def get_sample_block(self, bytes_in_block = 4096):

        # In theory we could wrap around the end of the buffer but in practice 
        # (2*self.PRU_MAX_SHORT_SAMPLES) should be a multiple of bytes_in_block
        # This allows for much simpler code
        head_offset = self._tail
        if (head_offset + bytes_in_block) > 2*self.PRU_MAX_SHORT_SAMPLES:
          head_offset=0

        tail_offset = struct.unpack_from("l", self._data, self.PRU0_OFFSET_DRAM_HEAD)[0]
        while (tail_offset - head_offset)%(2*self.PRU_MAX_SHORT_SAMPLES) < bytes_in_block:
            tail_offset = struct.unpack_from("l", self._data, self.PRU0_OFFSET_DRAM_HEAD)[0]

        # dtype='4<u4' means an array of dimension 4 of 4 unsigned integer written in little endian
        # (16 bytes per row, hence the /16 for the offsets and counts)
        result = np.frombuffer(self._extmem, dtype='4<u4', count=bytes_in_block/16, offset=head_offset/16)
        result.dtype = np.int32

        self._tail = (self._tail + bytes_in_block)%(2*self.PRU_MAX_SHORT_SAMPLES)

        return result

    def follow_stream(self):
        quit = False
        bytes_in_block = 4096*4
        fftfreq = np.fft.rfftfreq(bytes_in_block/16, d=1.0/40000) # /16 -> /4 channels /4 bytes per channel
        self._tail = struct.unpack_from("l", self._data, self.PRU0_OFFSET_DRAM_HEAD)[0]
        self._tail -= self._tail % bytes_in_block
        while (not quit):
            samples=self.get_sample_block(bytes_in_block)
            
            #Get rid of the channel markings in 0xFF000000
            output = np.left_shift(samples, 8)
            
            #Invert dimensions
            channels = np.transpose(output)
            fft = np.fft.rfft(channels[3])
            plt.plot(fftfreq, fft.real)
            plt.show()
