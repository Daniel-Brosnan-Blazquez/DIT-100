
# Program to setup the PWM signal to run the ESC with the proper configuration

import sys
sys.path.append ("../../lib")
import pwm

# Create the PWM context
pwm.reset ()
P = pwm.Init()

(status, message) = pwm.set_freq (P, 400)
if not status:
	print message

for channel in xrange (12,16):
	print "Arming the ESC connected to the channel: %s" % channel
	(status, message) = pwm.set_pwm (P, channel, 400)
	if not status:
		print message

	
		

