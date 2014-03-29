# -*- coding: utf-8 -*-

# Collector for the PWM device
import sys
sys.path.append ("../")
import lib.pwm as lpwm
import lib.common as common

def initialize (params):
    '''
    Initialize the pwm device
    '''
    
    if type (params) != dict:
        return (False, "Unable to initialize the PWM device. The params parameter is not of type dict, is of type %s" % type (params))
    
    # Reset pwm device
    lpwm.reset ()

    # Create pwm context
    params['pwm'] = lpwm.Init ()
    if params['pwm']['error'][0]:
        return (False, "Unable to initialize the PWM device. The pwm context could not be created. The error was %s" % params['pwm']['error'][1])
    
    # Set frequency to the right value
    (status, message) = lpwm.set_freq (params['pwm'], params['freq'])
    if not status:
	return (False, "Unable to initialize the PWM device. The error was %s" % message)

    # Enable auto-increment
    (status, message) = lpwm.enable_ai (params['pwm'], True)
    if not status:
        return (False, "Unable to enable the auto-increment feature on the PWM device. The error was %s" % message)

    (status, message) = lpwm.set_pwm_ai (params['pwm'], params['minch'], [(400,0),(400,0),(400,0),(400,0)])
    if not status:
        return (False, "Unable to initialize the PWM device. The error was %s" % message)

    return (True, None)
