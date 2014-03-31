
# Program to perform the PWM signal

import sys
sys.path.append ("../../lib")
import pwm
import time 
import signal

def control_c_handler(signal, frame):

	for channel in xrange (12,16):
		(status, message) = pwm.set_pwm (P, channel, 400)
		if not status:
			print message
			
	return


signal.signal(signal.SIGINT, control_c_handler)

channel = 12

# Create the PWM context
#pwm.reset ()
P = pwm.Init()

print "The frequency is set to %s" % pwm.get_freq (P)[1]
print "The default channel to output the PWM signal is %s" % channel

raw = ''

while raw != 'c':
	raw = raw_input("\tPress f to change the frequency\n" +
			"\tPress g to get the frequency value\n" +
			"\tPress d to change the duty cycle\n" +
			"\tPress v to get the duty cycle\n" +
			"\tPress o to change the output channel\n" +
			"\tPress a to change velocity on every motor\n" +
			"\tPress s to stop every motor\n" +
			"\tPress c to cancel the program:")        

	if raw == 'f':
		raw = raw_input("\tEnter the frequency:")   		
		try:
			freq = int (raw)
			(status, message) = pwm.set_freq (P, freq)
			if not status:
				print message
		except ValueError as e:
			print "The frequency has to be of type integer in the range of (40,1000)" 
	elif raw == 'o':
		raw = raw_input("\tEnter the channel:")   		
		try:
			channel = int (raw)
			raw = 'd'
		except ValueError as e:
			print "The channel has to be of type integer in the range of (0,15)" 

	elif raw == 'a':
		raw = raw_input("\tEnter the duty cycle:")   		
		try:
			duty_cycle = int (raw)

			for channel in xrange (12,16):
				(status, message) = pwm.set_pwm (P, channel, duty_cycle)
				if not status:
					print message

		except ValueError as e:
			print "The duty_cycle has to be of type integer in the range of (0,100)" 

	elif raw == 'at':
		tstart = raw_input("\tEnter the time of waiting to move the motors:")   		
		tstop = raw_input("\tEnter the time of waiting to stop the motors:")   		
		raw = raw_input("\tEnter the duty cycle:")   		
		try:
			time.sleep (int (tstart))
			duty_cycle = int (raw)

			for channel in xrange (12,16):
				(status, message) = pwm.set_pwm (P, channel, duty_cycle)
				if not status:
					print message

			time.sleep (int (tstop))
			for channel in xrange (12,16):
				(status, message) = pwm.set_pwm (P, channel, 400)
				if not status:
					print message


		except ValueError as e:
			print "The duty_cycle has to be of type integer in the range of (0,100)" 

	elif raw == 'p':
		raw1 = raw_input("\tEnter the channel:")
		raw2 = raw_input("\tEnter the duty cycle:")   		
		try:
			channel = int (raw1)
			duty_cycle = int (raw2)

			(status, message) = pwm.set_pwm (P, channel, duty_cycle)
			if not status:
				print message

			time.sleep (0.0025)

			(status, message) = pwm.set_pwm (P, channel, 400)
			if not status:
				print message

		except ValueError as e:
			print "The duty_cycle has to be of type integer in the range of (0,100) or the channel has to be of type integer in the range of (12,15)" 

	elif raw == 's':
		duty_cycle = 400

		for channel in xrange (12,16):
			(status, message) = pwm.set_pwm (P, channel, duty_cycle)
			if not status:
				print message

	if raw == 'd':
		raw = raw_input("\tEnter the duty cycle:")   		
		try:
			duty_cycle = int (raw)
			(status, message) = pwm.set_pwm (P, channel, duty_cycle)
			if not status:
				print message

		except ValueError as e:
			print "The duty_cycle has to be of type integer" 

	if raw == 'g':
		print "The frequency is set to %s" % pwm.get_freq (P)[1]

	if raw == 'v':
		(status, values) = pwm.get_pwm (P, channel)
		if not status:
			print values
		(duty_cycle, delay) = values
		print "The duty_cycle is set to %s" % duty_cycle
	
		

