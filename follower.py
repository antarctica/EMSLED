import pypruss
import numpy as np
import time
import struct
from collections import defaultdict
from ctypes import c_long
import matplotlib
matplotlib.use('GTKagg')
import matplotlib.pyplot as plt


class follower(object):

    PRU0_OFFSET_DRAM_HEAD = 0x1014
    PRU_MAX_SHORT_SAMPLES = 128*1024
    PRU_EVTOUT_0 = 3


    def __init__(self, pru = 0):
        if pru == 0:
            pru_dataram = pypruss.PRUSS0_PRU0_DATARAM
        else:
            pru_dataram = pypruss.PRUSS0_PRU1_DATARAM

        self._spare = 0

        print "\tINFO: pruss init\n"
        pypruss.init()						# Init the PRU

        print "\tINFO: pruss open\n"
        ret = pypruss.open(self.PRU_EVTOUT_0)

        print "\tINFO: pruss intc init\n"
        pypruss.pruintc_init()					# Init the interrupt controller

        print "\tINFO: mapping memory \n"
        self._data = pypruss.map_prumem(pru_dataram)
        print "\tINFO: data segment len=" + str(len(self._data)) + "\n"
        self._tail = 0
        
        self._extmem = pypruss.map_extmem()
        print "\tINFO: ext segment len=" + str(len(self._extmem))

        print "\tINFO: setup mem \n"
        self.ddrMem = pypruss.ddr_addr()
        print "V extram_base = " + hex(self.ddrMem) + "\n"

    def read_uint(self, offset=0):
        return self.readData('L', offset, 1)[0]

    def readData(self, dtype='l', offset=0, samples = 1):
        return struct.unpack_from(dtype*samples, self._extmem, offset)

    def get_sample_block(self, bytes_in_block = 4096):

        head_offset = self._tail
        tail_offset = struct.unpack_from("l", self._data, self.PRU0_OFFSET_DRAM_HEAD)[0]
        while abs(tail_offset - head_offset) < bytes_in_block:
            tail_offset = struct.unpack_from("l", self._data, self.PRU0_OFFSET_DRAM_HEAD)[0]
            self._spare += 1

        # dtype='4<u4' means an array of dimension 4 of 4 unsigned integer written in little endian
        # (16 bytes per row, hence the /16 for the offsets and counts)
        result = np.frombuffer(self._extmem, dtype='4<u4', count=bytes_in_block/16, offset=head_offset/16)
        result.dtype = np.int32

        self._tail = (self._tail + bytes_in_block)%(2*self.PRU_MAX_SHORT_SAMPLES)

        return result

    def follow_stream(self):
        quit = False
        channels = defaultdict(list)
        index = 0
        channel = 3
        iter = 0
        bytes_in_block = 4096*4
        self._tail = struct.unpack_from("l", self._data, self.PRU0_OFFSET_DRAM_HEAD)[0]
        self._tail -= self._tail % bytes_in_block
        with open("samples.bin", 'wb') as f:
         while (iter < 1000):
             iter += 1
             samples=self.get_sample_block(bytes_in_block)
             output = np.left_shift(samples, 8)
             channel4 = np.transpose(output) #[3]
             fft = np.fft.rfft(channel4[3])
             fftfreq = np.fft.rfftfreq(channel4[3].size, d=1.0/40000)
             print str(len(fft)) + " " + str(len(fftfreq))
             plt.plot(fftfreq, fft.real)
             plt.show()