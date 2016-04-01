import pypruss
import numpy as np
import struct
import Adafruit_BBIO.GPIO as GPIO
import time
from sample import sample,waveform
import logging


class follower(object):

    PRU_MAX_SHORT_SAMPLES =   128*1024
    PRU0_OFFSET_SRAM_HEAD =   0x1000
    PRU0_OFFSET_DRAM_PBASE =  0x1004
    PRU0_OFFSET_SPIN_COUNT =  0x1008
    PRU0_OFFSET_RES1 =        0x100C
    PRU0_OFFSET_SRAM_TAIL =   0x1010
    PRU0_OFFSET_DRAM_HEAD =   0x1014
    PRU0_OFFSET_RES2 =        0x1018
    PRU0_OFFSET_RES3 =        0x101C
    PRU_EVTOUT_0 = 3


    def __init__(self, pru = 0, pru0_fw="arm/pru00.bin", pru1_fw="arm/pru01.bin"):
        if pru == 0:
            pru_dataram = pypruss.PRUSS0_PRU0_DATARAM
        else:
            pru_dataram = pypruss.PRUSS0_PRU1_DATARAM

        self._spare = 0

        logging.debug("[ADC] setting up power control line")
        GPIO.setup("P9_18",GPIO.OUT)

        logging.debug("[ADC] pruss init")
        pypruss.init()						# Init the PRU

        logging.debug("[ADC] pruss open")
        ret = pypruss.open(self.PRU_EVTOUT_0)

        logging.debug("[ADC] pruss intc init")
        pypruss.pruintc_init()					# Init the interrupt controller

        logging.debug("[ADC] mapping memory")
        self._data = pypruss.map_prumem(pru_dataram)
        
        logging.debug("[ADC] data segment len=%d" % len(self._data))
        
        logging.debug("[ADC] setting tail")
        self._tail = 0
        struct.pack_into('l', self._data, self.PRU0_OFFSET_DRAM_HEAD, self._tail)
        
        logging.debug("[ADC] mapping extmem")
        self._extmem = pypruss.map_extmem()
        logging.debug("[ADC] ext segment len=%d" % len(self._extmem))

        logging.debug("[ADC] setup mem")
        self.ddrMem = pypruss.ddr_addr()
        logging.debug("[ADC] V extram_base = 0x%x" % self.ddrMem)

        self._pru01_phys = int(open("/sys/class/uio/uio1/maps/map1/addr", 'r').read(), 16)
        struct.pack_into('L', self._data, self.PRU0_OFFSET_SRAM_HEAD, 0)
        struct.pack_into('L', self._data, self.PRU0_OFFSET_DRAM_PBASE, self._pru01_phys)
        struct.pack_into('L', self._data, self.PRU0_OFFSET_SPIN_COUNT, 0)
        struct.pack_into('L', self._data, self.PRU0_OFFSET_RES1, 0xdeadbeef)
        struct.pack_into('L', self._data, self.PRU0_OFFSET_SPIN_COUNT, 0)
        struct.pack_into('L', self._data, self.PRU0_OFFSET_SPIN_COUNT, 0)
        struct.pack_into('L', self._data, self.PRU0_OFFSET_RES2, 0xbabedead)
        struct.pack_into('L', self._data, self.PRU0_OFFSET_RES3, 0xbeefcafe)
        
        logging.debug("[ADC] loading pru00 code")
        pypruss.exec_program(0, pru0_fw)
        
        logging.debug("[ADC] loading pru01 code")
        pypruss.exec_program(1, pru1_fw)

    def power_on(self=None):
        logging.debug("[ADC] Powering the ADC on")
        GPIO.output("P9_18",GPIO.HIGH)
        #Wait for everything to come to life
        time.sleep(1)

    def power_off(self=None):
        logging.debug("[ADC] Powering the ADC off")
        GPIO.output("P9_18",GPIO.LOW)

    def stop(self):
        pypruss.pru_disable(0)
        pypruss.pru_disable(1)

    def read_uint(self, offset=0):
        return self.readData('L', offset, 1)[0]

    def readData(self, dtype='l', offset=0, samples = 1):
        return struct.unpack_from(dtype*samples, self._extmem, offset)

    def get_sample_block(self, bytes_in_block = 4096):

        # In theory we could wrap around the end of the buffer but in practice 
        # (2*self.PRU_MAX_SHORT_SAMPLES) should be a multiple of bytes_in_block
        # This allows for much simpler code
        head_offset = (self._tail // 4096) * 4096
        if (head_offset + bytes_in_block) > 2*self.PRU_MAX_SHORT_SAMPLES:
          head_offset=0

        self._spare = 0
        tail_offset = struct.unpack_from("l", self._data, self.PRU0_OFFSET_DRAM_HEAD)[0]
        diff = (tail_offset - head_offset)%(2*self.PRU_MAX_SHORT_SAMPLES)
        while ( (diff < bytes_in_block) or (diff > 2*self.PRU_MAX_SHORT_SAMPLES - bytes_in_block) ):
            tail_offset = struct.unpack_from("l", self._data, self.PRU0_OFFSET_DRAM_HEAD)[0]
            diff = (tail_offset - head_offset)%(2*self.PRU_MAX_SHORT_SAMPLES)
            self._spare = 1
            time.sleep(0.02)

        # dtype='4<u4' means an array of dimension 4 of 4 unsigned integer written in little endian
        # (16 bytes per row, hence the /16 for the offsets and counts)
        result = np.frombuffer(self._extmem, dtype='4<i4', count=bytes_in_block/16, offset=head_offset/16)
        result.dtype = np.int32

        self._tail = (head_offset + bytes_in_block)%(2*self.PRU_MAX_SHORT_SAMPLES)

        return result

    #SPS: Samples Per Second, this must be calibrated
    def follow_stream(self, SPS=40000, dispFFT=False, axis=[0,15000,-1e12,1e12], FFTchannels=[1,2,3], selected_freq=None):
        if dispFFT:
          import matplotlib
          matplotlib.use('GTKCairo')
          import matplotlib.pyplot as plt
          plt.ion()
          plt.show()
        quit = False
        if selected_freq:
          samples_count = int(1.0*SPS/selected_freq*50)
        else:
          samples_count = 1024
        bytes_in_block = samples_count * 16 #4 channels, 4B per sample

        fftfreq = np.fft.rfftfreq(bytes_in_block/16, d=1.0/SPS) # /16 -> /4 channels /4 bytes per channel
        if selected_freq:
          selected_index = np.argmin(np.abs(fftfreq - selected_freq))

        self._tail = struct.unpack_from("l", self._data, self.PRU0_OFFSET_DRAM_HEAD)[0]
        self._tail -= self._tail % bytes_in_block - bytes_in_block

        while (not quit):
            samples=self.get_sample_block(bytes_in_block)
            #Invert dimensions
            channels = np.transpose(samples)
            if axis != None and dispFFT and self._spare:
              plt.axis(axis)
            ostring=""
            for chan in FFTchannels:
              fft = np.fft.rfft(channels[chan]) / samples_count

              #Disregard the DC component.
              fft.real[0] = 0
              
              if selected_freq == None:
                selected_index = np.argmax(np.absolute(fft))
              ostring += "Channel " + str(chan) + ": %-*sHz = %-*s\t" %  (5, int(fftfreq[selected_index]), 12, int(np.average(np.absolute(fft[selected_index-5:selected_index+5])))) 
              if dispFFT and self._spare:
                plt.plot(fftfreq, np.absolute(fft), label="Channel %d"%chan)
                #plt.plot(channels[chan], label="Channel %d"%chan)
            print ostring
            if dispFFT and self._spare:
              plt.legend()
              plt.draw()
              plt.pause(0.001)
              plt.cla()

    def get_sample_freq(self, selected_freq, SPS=40000, dispFFT=False, FFTchannels=[1,2,3], axis=None):
        samples_count = int(1.0*SPS/selected_freq*50)
        bytes_in_block = samples_count * 16 #4 channels, 4B per sample
        fftfreq = np.fft.rfftfreq(bytes_in_block/16, d=1.0/SPS) # /16 -> /4 channels /4 bytes per channel
        selected_index = np.argmin(np.abs(fftfreq - selected_freq))
        
        self._tail = struct.unpack_from("l", self._data, self.PRU0_OFFSET_DRAM_HEAD)[0]
        self._tail -= self._tail % bytes_in_block - bytes_in_block

        samples=self.get_sample_block(bytes_in_block)

        #Invert dimensions
        channels = np.transpose(samples)

        ref_wave = waveform(selected_freq, np.fft.rfft(channels[0])[selected_index])
        selected_sample = sample(ref_wave)
        for chan in FFTchannels:
          fft = np.fft.rfft(channels[chan]) / samples_count
          chan_wave=waveform(selected_freq, fft[selected_index])
          selected_sample.add_channel(chan_wave)
        return selected_sample
