# This script does all that is necessary to setup the 
# Beaglebone, the ADC and the AWG
# and then logs the raw data to a file.

# Something similar to this should be the template for 
# autonomous operations of the setup


import os,sys,signal
import Adafruit_BBIO.GPIO as GPIO
import spi_awg as AWG
import analogue_IO
import setup_BB
import follower
import time # For testing only
import config
import numpy as np
import logging


def signal_handler(signum, frame):
  if signum == signal.SIGINT:
    print "Ctrl-C received, exitting"
    finish()

def startup():
  global f
  global ADC

  # Setup Logging
  logging.basicConfig(level=logging.INFO, stream=sys.stdout)

  setup_BB.setup_BB_slots()

  AWG.start(**config.hardware['AWG']) # start AWG

  args=config.test_params.copy()
  args.update(config.hardware['AWG'])
  logging.info("[LOGGER] Starting the AWG")
  AWG.configure2SineWave(**args) # configure for 2 sinewaves

  logging.info("[LOGGER] Settin analogue amplification")
  analogue_IO.enable(**config.hardware['IO']) # enable TX on analogue board

  logging.info("[LOGGER] Loading ADC PRU code")
  ADC = follower.follower()
  logging.info("[LOGGER] TX Power on and start sampling")
  ADC.power_on()

def setPhase(x):
  AWG.setPhase(x)
def setAmplitude(x):
  AWG.setAmplitude(x)

def logger():
  global ADC
  args=config.hardware['ADC'].copy()
  args.update({'selected_freq': config.test_params['tx_freq']})
  calibrate()
  finish()
  ADC.follow_stream(**args)
  ADC.display_phase_shift(**args)

def finish():
  print "Finished"

  ADC.power_off()
  ADC.stop()

  analogue_IO.disable() # disable TX
  GPIO.cleanup() # free GPIO ports
  exit(0)

def calibrate():
  logging.info("[LOGGER] Starting calibration procedure")
  global ADC
  target_amp = 2**35
  args_adc = config.hardware['ADC'].copy()
  args_adc.update({'selected_freq': config.test_params['tx_freq']})

  for chan in AWG.REGISTERS_GAIN:
    AWG.setGain(chan, 0)
    AWG.setPhaseShift(chan, 0)

  analogue_IO.enable(gain = 0)
  for txgain in np.linspace(0.01, 1.99, 199):
    AWG.setGain('tx', txgain)
    ADC = follower.follower()
    ADC.power_on()
    time.sleep(1)
    print ADC.get_sample_freq(**args_adc)

if __name__ == "__main__":
  signal.signal(signal.SIGINT, signal_handler)
  startup()
  logger()
  setPhase(150)
  finish()
