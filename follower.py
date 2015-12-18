import pypruss
import numpy as np
import time
import struct
from collections import defaultdict


class follower(object):

    PRU0_OFFSET_DRAM_HEAD = 0 #0x1014
    PRU_MAX_SHORT_SAMPLES = 128*1024
    PRU_EVTOUT_0 = 3


    def __init__(self, pru = 0):
        if pru == 0:
            pru_dataram = pypruss.PRUSS0_PRU0_DATARAM
        else:
            pru_dataram = pypruss.PRUSS0_PRU1_DATARAM

        print "\tINFO: pruss init\n"
        pypruss.init()						# Init the PRU

        print "\tINFO: pruss open\n"
        ret = pypruss.open(self.PRU_EVTOUT_0)

        print "\tINFO: pruss intc init\n"
        pypruss.pruintc_init()					# Init the interrupt controller

        print "\tINFO: mapping memory \n"
        self._data = pypruss.map_prumem(pru_dataram)
        print "\tINFO: data segment len=" + str(len(self._data)) + "\n"
        self._tail = self._head = self.PRU0_OFFSET_DRAM_HEAD

        print "\tINFO: setup mem \n"
        self.ddrMem = pypruss.ddr_addr()
        print "V extram_base = " + hex(self.ddrMem) + "\n"

    def read_uint(self, offset=0):
        return self.readData('L', offset, 1)[0]

    def readData(self, dtype='l', offset=0, samples = 1):
        return struct.unpack_from(dtype*samples, self._data, offset)

    def get_sample_block(self, bytes_in_block = 4096):

        """
        The following code witht the mask thing is equivalent to
        """
        head_offset = self._head
        tail_offset = self.read_uint(self._tail)
        while tail_offset - head_offset < bytes_in_block:
            tail_offset = self.read_uint(self._tail)

        """
        mask = ~(bytes_in_block - 1)
        head_offset = self._head & mask
        tail_offset = self.read_uint(self._tail) & mask
        while head_offset == tail_offset:
            tail_offset = self.read_uint(self._tail) & mask
        """

        result = self.readData('L', self._head/struct.calcsize('L'), bytes_in_block/struct.calcsize('L'))

        self._tail = (self._tail + bytes_in_block)%(2*self.PRU_MAX_SHORT_SAMPLES)

        return result

    def follow_stream(self):
        quit = False
        channels = defaultdict(list)
        index = 0
        channel = 3
        while (not quit):
            samples=self.get_sample_block()
            for sample in samples:
		print(hex(sample) + " " + hex((sample & 0xFFFF0000) >> 16 ^ (sample & 0x00FF) << 16))
                #output = sample & 0x00FFFFFF
                output = (sample & 0xFFFF0000) >> 16 ^ (sample & 0x00FF) << 16
                output = ((2.5*output)/8388608)
		if sample & 0x00000080: output = output - 5
                channels[index].append(output)
                if channel == 0: index += 1
                channel = (channel - 1 ) % 4
            quit = 1
            print(channels)