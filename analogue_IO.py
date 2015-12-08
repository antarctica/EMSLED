# This script is responsible for setting up and controlling the 
# GPIO that are connected to the analogue board.
# There are currently 3 control lines, TX on, and 2 TX gain control bits

import Adafruit_BBIO.GPIO as GPIO


GPIO.setup("P9_14",GPIO.OUT) # TX gain control
GPIO.setup("P9_15",GPIO.OUT) # TX gain control
GPIO.setup("P9_16",GPIO.OUT) # TX power


def enable():
	GPIO.output("P9_14",GPIO.HIGH)
	GPIO.output("P9_15",GPIO.HIGH)
	GPIO.output("P9_16",GPIO.HIGH)

def disable():
	GPIO.output("P9_14",GPIO.LOW)
	GPIO.output("P9_15",GPIO.LOW)
	GPIO.output("P9_16",GPIO.LOW)
