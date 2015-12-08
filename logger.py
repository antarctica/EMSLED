# This script does all that is necessary to setup the 
# Beaglebone, the ADC and the AWG
# and then logs the raw data to a file.

# Something similar to this should be the template for 
# autonomous operations of the setup


import os,sys
import subprocess
import Adafruit_BBIO.GPIO as GPIO
import spi_awg
import analogue_IO
import setup_BB


def startup():
	global f
	setup_BB.setup_BB_slots()
	# prepare log file
	try:
		f = open(sys.argv[1],"w")
	except: f = open("log.txt","w")

	spi_awg.start() # start AWG
	spi_awg.configure2SineWave() # configure for 2 sinewaves

	analogue_IO.enable() # enable TX on analogue board

	os.system("arm/pructl -clean")
	os.system("arm/pructl -start") # start ADC

def setPhase(x):
	spi_awg.setPhase(x)
	os.system("arm/pructl -clean")
	os.system("arm/pructl -start") # start ADC
def setAmplitude(x):
	spi_awg.setAmplitude(x)
	os.system("arm/pructl -clean")
	os.system("arm/pructl -start") # start ADC

def logger():
	global f
	log = True
	p = subprocess.Popen('arm/follower_fft_francois',shell=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
	output = []
	while log:
		try:
			line = p.stdout.readline().replace("\n","")
			if line != '': 
				output.append(line)
				print "OUTPUT ",line
				f.write(line+"\n")
		except KeyboardInterrupt:
			log = False	

def finish():
	global f
	print "Finished"

	f.close()

	analogue_IO.disable() # disable TX
	os.system("arm/pructl -stop") # disable ADC
	GPIO.cleanup() # free GPIO ports

if __name__ == "__main__":
	startup()
	logger()
	setPhase(150)
	finish()
