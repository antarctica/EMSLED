#!/usr/bin/python

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
  logging.info("[CALIBRATION] Starting the AWG")
  AWG.configure2SineWave(**args) # configure for 2 sinewaves

  logging.info("[CALIBRATION] Setting up analogue amplification")
  analogue_IO.enable(**config.hardware['IO']) # enable TX on analogue board

  logging.info("[CALIBRATION] Loading ADC PRU code")
  ADC = follower.follower()
  logging.info("[CALIBRATION] TX Power on and start sampling")
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

def finish():
  print "Finished"

  ADC.power_off()
  ADC.stop()

  analogue_IO.disable() # disable TX
  GPIO.cleanup() # free GPIO ports
  exit(0)

def calibrate():
  logging.info("[CALIBRATION] Starting calibration procedure")
  global ADC
  max_amp = 2**32
  target_coeff = 1
  args_adc = config.hardware['ADC'].copy()
  args_adc.update({'selected_freq': config.test_params['tx_freq']})
  zero_gains()
  analogue_IO.enable(gain = 0)
  for chan in ["X", "Y", "Z"]:
    logging.info("[CALIBRATION] - Calibration, searching channel %s", chan)
    ADC = follower.follower()
    ADC.power_on()
    amplitude_tx = noise_level = get_trimmed_mean_amp(args_adc, chan, max_dev=1)
    ADC.stop()
    logging.debug("[CALIBRATION] - Find tx_coeff (what's the read amplitude it the current channel for a tx gain of 1)")
    tx_coeff = get_tx_coeff(args_adc, 0.001, max_amp, 'tx', chan)
    test_tx_gain = min(target_coeff, 0.5 * max_amp / tx_coeff)
    ADC = follower.follower()
    ADC.power_on()
    waveform_tx_only = ADC.get_sample_freq(**args_adc)
    ADC.stop()
    AWG.setGain('tx', 0, oneshot=True)
    logging.debug("[CALIBRATION] - Find bc_coeff (what's the read amplitude it the current channel for a bucking gain of 1)")
    bc_coeff = get_tx_coeff(args_adc, 0.001, max_amp, chan, chan)
    test_bc_gain = min(target_coeff, 0.7 * test_tx_gain * tx_coeff / bc_coeff, 0.3 * max_amp / bc_coeff)
    logging.debug("[CALIBRATION] - Use the coefficients we've found to find the best phase shift")
    AWG.program()
    AWG.setGain('tx', test_tx_gain)
    AWG.setGain(chan, test_bc_gain)
    AWG.run()
    best_phase = phase_min(args_adc, chan, samples=10)
    logging.debug("[CALIBRATION] - Use the coefficients we've found to find the best phase shift (2nd step)")
    AWG.program()
    AWG.setGain('tx', 0)
    AWG.setPhaseShift(chan, best_phase, deg=True)
    AWG.run()
    logging.info("[CALIBRATION] - Channel %s first pass results: tx=%f, bc=%f @ %f deg", chan, test_tx_gain, test_bc_gain, best_phase)
    # Now for the fine tuning:
    if tx_coeff >= bc_coeff:
      # Then our maximum tx_gain will be 1 and the bucking gain will be <1
      while True:
        tx_gain = target_coeff
        bc_gain = bc_coeff / tx_coeff
        AWG.program()
        AWG.setGain('tx', tx_gain)
        AWG.setGain(chan, bc_gain)
        AWG.setPhaseShift(chan, best_phase, deg=True)
        AWG.run()
        current_amp = get_trimmed_mean_amp(args_adc, chan)
        ADC = follower.follower()
        ADC.power_on()
        waveform_now = ADC.get_sample_freq(**args_adc)
        ADC.stop()
        if abs(waveform_now.compare_phase_shift(ord(chan) - ord('X'), waveform_tx_only)) > np.pi / 2:
          # The secondary field (from the bucking coil) is stronger, read a negative amplitude
          current_amp = -current_amp
        logging.info("[CALIBRATION] - I am a lazy program, giving up for now, channel %s residual signal amplitude: %f with tx=%f and bc=%f %f deg", chan, current_amp, tx_gain, bc_gain, best_phase)
        break
    else:
      logging.info("[CALIBRATION] - I am a lazy program, bc_coeff > tx_coeff not implemented, channel %s", chan)


def get_tx_coeff(args_adc, start_gain, max_amp, chantx, chanrx):
  txgain = start_gain / 2
  amplitude_tx = 0
  while txgain < 1 and amplitude_tx < max_amp / 2:
    txgain = min(2 * txgain, 1)
    AWG.setGain(chantx, txgain, oneshot=True)
    amplitude_tx = get_trimmed_mean_amp(args_adc, chanrx)
  return amplitude_tx / txgain
  
def phase_min(args_adc, chan, samples=3):
  amplitude_ps = []
  for phase in np.linspace(0, 360.0/64*63, 64):
    AWG.setPhaseShift(chan, phase, deg=True, oneshot=True)
    amplitude_ps.append(get_trimmed_mean_amp(args_adc, chan, samples=samples))
    print "PHASE: %d:%d" % (int(phase), int(amplitude_ps[-1]))
  fft = np.fft.rfft(amplitude_ps)
  return np.angle(fft[1], deg=True)%360

def get_trimmed_mean_amp(args_adc, chan, max_dev=None, samples=3):
  amps = []
  ADC = follower.follower()
  ADC.power_on()
  for i in range(samples):
    sample = ADC.get_sample_freq(**args_adc)
    amps.append(sample.channels[ord(chan) - ord('X')].get_amplitude())
  ADC.stop()
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
