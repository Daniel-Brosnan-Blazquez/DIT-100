
# Program to print the frequency of the pwm signal received on the decided pins

import RPi.GPIO as GPIO
import numpy
import os
import signal
import sys

PIN = 13
begin_on = []
end_on = []

def falling_handler (channel):
	import time
	measured_time = time.time ()
	if len (begin_on) == 0:
		begin_on.append (measured_time)
	elif len (begin_on) == 1:
		begin = begin_on.pop()
		end_on = measured_time - begin
		end = 1.0/float (end_on)
		print "Frequency = %.20f" %  end
	return

# Configure the board format
GPIO.setmode(GPIO.BOARD)

def control_c_handler(signal, frame):
	''' Remove event detection on the decided pins '''
	print "Canceling the PWM monitoring"
	GPIO.remove_event_detect (PIN)
	sys.exit (0)

signal.signal(signal.SIGINT, control_c_handler)

# Configure the events to be monitored on the decided pins
GPIO.setup (PIN, GPIO.IN)
GPIO.add_event_detect (PIN, GPIO.FALLING, falling_handler)

while True:
	continue

		

