# This script does all that is necessary to setup the 
# Beaglebone, the ADC and the AWG
# and then logs the raw data to a file.

# Something similar to this should be the template for 
# autonomous operations of the setup


import os,sys,signal
import Adafruit_BBIO.GPIO as GPIO
import spi_awg
import analogue_IO
import setup_BB
import follower
import time # For testing only
import config

def signal_handler(signum, frame):
  if signum == signal.SIGINT:
    print "Ctrl-C received, exitting"
    finish()

def startup():
  global f
  global ADC
  setup_BB.setup_BB_slots()
  # prepare log file
  try:
    f = open(sys.argv[1],"w")
  except: f = open("log.txt","w")

  spi_awg.start(**config.hardware['AWG']) # start AWG

  args=config.test_params.copy()
  args.update(config.hardware['AWG'])
  spi_awg.configure2SineWave(**args) # configure for 2 sinewaves

  analogue_IO.enable(**config.hardware['IO']) # enable TX on analogue board

  ADC = follower.follower()
  ADC.power_on()

def setPhase(x):
  spi_awg.setPhase(x)
def setAmplitude(x):
  spi_awg.setAmplitude(x)

def logger():
  global f, ADC
  args=config.hardware['ADC'].copy()
  args.update({'selected_freq': config.test_params['tx_freq']})
  ADC.follow_stream(**args)

def finish():
  global f
  print "Finished"

  f.close()
  ADC.power_off()
  ADC.stop()

  analogue_IO.disable() # disable TX
  GPIO.cleanup() # free GPIO ports
  exit(0)

if __name__ == "__main__":
  signal.signal(signal.SIGINT, signal_handler)
  startup()
  logger()
  setPhase(150)
  finish()
