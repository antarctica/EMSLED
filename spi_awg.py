# This is for configuring the AWG over a bit-banged SPI bus created by PRU1.

# This cannot be used at the same time as the ADC, as that uses PRU1 as well. 


import pypruss						# The Programmable Realtime Unit Library
import numpy as np					# Needed for braiding the pins with the delays
import mmap,struct
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import time

def start():
	GPIO.setup("P8_30",GPIO.OUT)	#SPI CS for AWG
	GPIO.output("P8_30",GPIO.HIGH)	#Set CS high initally - i.e. disabled
	GPIO.setup("P9_23",GPIO.OUT)	#AFG External trigger.  NB check JP1 setting too!
	GPIO.setup("P9_24",GPIO.OUT)	#What is this for?  Possibly old code for the old SPI non bit banged bus?
	PWM.start("P8_34",50,70e6) 	# setup ADC master clock 13e6 = 25khz  ~ NCO sees 50MHz clock - connects to J10 on AFG

# NCO removed P8_46 because of a device tree conflict with SPI_overlay_PRU1
#	PWM.start("P8_46",50,70e6) # 100e5

	pypruss.modprobe()					# This only has to be called once pr boot


# This appears to be a clock function BUT what for?  Seems to be for old SPI non bit banged bus?  Is this non longer needed?
def c():
	GPIO.output("P9_24",GPIO.HIGH)
	time.sleep(0.001)
	GPIO.output("P9_24",GPIO.LOW)
	time.sleep(0.001)

# This appears to be a trigger or CS line BUT what for?  Seems to be for old SPI non bit banged bus?  Is this no longer needed?
def trigger():
	print "Trigger"
	GPIO.output("P9_23",GPIO.HIGH)
	time.sleep(0.1)
	GPIO.output("P9_23",GPIO.LOW)




def sendData(addr,data,rx=0):
	if(rx==0):
		data = [int((0x00 << 24) + (addr << 16) + data)]
	else:
		data = [int((0x80 << 24) + (addr << 16) + 0x0000)]
	
	# data = [0x80272345]	
	GPIO.output("P8_30",GPIO.LOW)
	pypruss.init()						# Init the PRU
	pypruss.open(0)						# Open PRU event 0 which is PRU0_ARM_INTERRUPT
	pypruss.pruintc_init()					# Init the interrupt controller
	pypruss.pru_write_memory(1, 0, data)			# Load the data in the PRU ram
	pypruss.exec_program(1, "arm/spi_awg.bin")		# Load firmware "mem_write.bin" on PRU 0
	pypruss.wait_for_event(0)				# Wait for event 0 which is connected to PRU0_ARM_INTERRUPT
	pypruss.clear_event(0)					# Clear the event
	# data = [0x12345678]
	pypruss.pru_write_memory(1, 0, data)			# Load the data in the PRU ram
	GPIO.output("P8_30",GPIO.HIGH)
	if (rx == 1):

		PRU_ICSS = 0x4A300000  
		PRU_ICSS_LEN = 512*1024 

		# RAM0_START = 0x00000000
		# RAM1_START = 0x00002000
		RAM2_START = 0x00012000


		with open("/dev/mem", "r+b") as f:
			ddr_mem = mmap.mmap(f.fileno(), PRU_ICSS_LEN, offset=PRU_ICSS) 
			# local = struct.unpack('LLLL', ddr_mem[RAM1_START:RAM1_START+16])
			shared = struct.unpack('L', ddr_mem[RAM2_START:RAM2_START+4])
			print hex(shared[0])
		f.close()
	pypruss.exit()							# Exit
	if (rx == 1):
		return shared[0]
	else:
		return 0


def writeData(addr,value,label="",validate=1):
	out = ""
	while ( out != value):
		c()
		sendData(addr,value)
		c()
		sendData(0x1D,0x0001)
		c()
		out = sendData(addr,0x0000,rx=1)
	
		print label,"=",hex(out),hex(value),out==value
		if validate == 0: out = value
	return out

def getData(addr,label="",output=1):
	c()
	out = sendData(addr,0x0000,rx=1)
	if label != "":
		print label,hex(out)
	return out
		

def getRegisterMap():
	map = []
	for x in range(0x34,0x38):
		out = getData(x)
		print hex(x),hex(out)
		if (out != 0):
			map.append([x,hex(out)])
	print ""
	print ""
	for x in map:
		print hex(x[0]),x[1]


def program():
	GPIO.output("P9_23",GPIO.HIGH)
	writeData(0x1E,0x0000,label="Program mode")
def run():
	getData(0x1E,label="Current run mode?")
	writeData(0x1E,0x0001,label="Run mode")
	trigger()
	getData(0x1E,label="Current run mode?")

def configureGaussian():
	program()
	writeData(0x27,0x0030,label="Gaussian Mode")
	writeData(0x31,0xFF00,label="DC value")
	writeData(0x35,0x4000,label="DC gain")
	run()
def configureDC():
	# not working yet?  not going to work through rf transformer
	program()
	writeData(0x27,0x0001,label="DC Mode")
	writeData(0x31,0x0000,label="DC value")
	writeData(0x35,0x4000,label="DC gain")
	run()
def configureSawtooth():
	program()
	writeData(0x27,0x1111,label="Sawtooth Mode") # sawtooth dac1 dac2
	writeData(0x37,0xF0F0,label="Step length")
	writeData(0x35,0x5000,label="DC gain")
	run()
def configureSineWave():
	program()
	writeData(0x27,0x0031,label="Sinewave Mode")
	writeData(0x45,0x0000,label="Static phase/freq")
	writeData(0x3E,0x0002,label="Freq MSB") # 0x00 0x48
	writeData(0x3F,0x0000,label="Freq LSB")
	writeData(0x35,0x0800,label="DC gain")
	run()
def configure2SineWave():
	program()
	writeData(0x27,0x3131,label="Sinewave Mode")
	time.sleep(0.1)
	writeData(0x45,0x0000,label="Static phase/freq")
	time.sleep(0.1)
	writeData(0x3E,0x0032,label="Freq MSB") # 0x03 0x62
	time.sleep(0.1)
	writeData(0x3F,0x0000,label="Freq LSB")
	time.sleep(0.1)
	writeData(0x35,0x4000,label="DC gain - TX") # don't go higher than 4000
	time.sleep(0.1)
	writeData(0x34,0x4000,label="DC gain 2 - Bucking") # careful, can saturate rx coiil
	time.sleep(0.1)
	writeData(0x43,150,label="Phase offset") # 0x1000 = 30 degrees ?? 22
	time.sleep(0.1)
	run()

def setAmplitude(amplitude):
	program()
	writeData(0x34,amplitude,label="DC gain 2 - Bucking",validate=0) # careful, can saturate rx coiil
	run()

def setPhase(phase):
	program()
	writeData(0x43,phase,label="Phase offset",validate=0) # 0x1000 = 30 degrees ?? 22
	time.sleep(0.1)
	run()
	
def finish():
	GPIO.cleanup()
if __name__ == "__main__":
	start()
	configure2SineWave()
	#for x in range(36):
	#	setPhase(int(10*x*2**16/360))
	#	time.sleep(3)
	
	#finish()
	#configureSawtooth()
	# print writeData(0x27,0x3233)
	# getRegisterMap()

