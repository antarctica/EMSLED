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

  logging.info("[LOGGER] Setting up analogue amplification")
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
  max_amp = 1e11
  min_amp = 5e11
  args_adc = config.hardware['ADC'].copy()
  args_adc.update({'selected_freq': config.test_params['tx_freq']})
  zero_gains()
  analogue_IO.enable(gain = 0)
  for chan in ["X", "Y", "Z"]:
    logging.info("[LOGGER] - Calibration, searching channel %s", chan)
    for txgain in np.linspace(0.01, 1, 100):
      zero_gains()
      noise_level = get_trimmed_mean_amp(args_adc, chan, max_dev=1)*3
      AWG.setGain('tx', txgain)
      ADC = follower.follower()
      ADC.power_on()
      sample = ADC.get_sample_freq(**args_adc)
      amplitude_tx = sample.channels[ord(chan) - ord('X')].get_amplitude()
      if amplitude_tx > min_amp or txgain == 1:
        print "Chan %s, Test amp %f at gain %f" % (chan, amplitude_tx, txgain)
        zero_gains()
        for bcgain in np.linspace(0.001, 1, 10000/config.test_params['tx_freq']*1000):
          AWG.setGain(chan, bcgain)
          amplitude_bc = get_trimmed_mean_amp(args_adc, chan)
          if (amplitude_bc > amplitude_tx):
            print "Chan %s, Test bucking amp %f at gain %f" % (chan, amplitude_bc, bcgain)
            AWG.setGain('tx', txgain)
            best_phase = phase_min(args_adc, 0, 360, 18, chan, samples=5)
            best_phase = phase_min(args_adc, best_phase - 10, best_phase + 10, 1, chan)
            print "Chan %s, Test bucking phase at angle %d" % (chan, best_phase)
            break
        break

def phase_min(args_adc, start, end, step, chan, samples=3):
  amplitude_ps = np.ones(end) * 1e50
  for phase in range(start, end, step):
    AWG.setPhaseShift(chan, phase, deg=True)
    amplitude_ps[phase] = get_trimmed_mean_amp(args_adc, chan, samples=samples)
  return np.argmin(amplitude_ps)

def get_trimmed_mean_amp(args_adc, chan, max_dev=None, samples=3):
  amps = []
  ADC = follower.follower()
  ADC.power_on()
  for i in range(samples):
    sample = ADC.get_sample_freq(**args_adc)
    amps.append(sample.channels[ord(chan) - ord('X')].get_amplitude())
  avg = np.average(amps)
  if max_dev:
    max_dev_val = max_dev*avg
  else:
    max_dev_val = np.std(amps) * 2
  mask = np.abs(amps - avg) > max_dev_val
  return np.ma.array(amps, mask=mask).mean()


def zero_gains():
  AWG.program()
  for chan in AWG.REGISTERS_GAIN:
    AWG.setGain(chan, 0)
    AWG.setPhaseShift(chan, 0)
  AWG.run()

if __name__ == "__main__":
  signal.signal(signal.SIGINT, signal_handler)
  startup()
  logger()
  setPhase(150)
  finish()
