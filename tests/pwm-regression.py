#!/usr/bin/python
# -*- coding: utf-8 -*-

# import sys for command line parsing
import sys
sys.path.append ("../lib")

import time
import pwm
import common_i2c
import inspect

####################
# regression tests #
####################

#######
# PWM #
#######

def reset_p (P = None):
    '''
    Reset the values of the registers of the PWM device
    '''
    (status, message) = pwm.reset (P)
    if not status:
        error (lineno(),"Error while reseting the PWM device. The error was %s." % message)
        return False
    
    return True

def create_p ():
    ''' 
    Check the creation of the PWM context
    '''
    # Default parameters
    P = pwm.Init ()
    if P['error'][0]:
        error (lineno(),"Error while creating the PWP. Error message was: " + P['error'][1])
        return False

    if P['addr'] != 0x40:
        error (lineno(),"Error while creating the PWM context. The slave address was incorrect. Correct slave address: " + hex (0x40) + ", storaged slave address: " + hex (P['addr']))
        return False

    # Finished test
    del P
    
    return True

def test_01 ():
    '''
    Check the default values of the PWM device
    '''

    P = pwm.Init ()    
    if P['error'][0]:
        error (lineno(),"Error while creating the PWM. Error message was: " + P['error'][1])
        return False

    # Read the value of the MODE1 register
    (status, value) = common_i2c.read (P, pwm.__MODE1, 0xff)
    if value != 0x0:
        error (lineno(),"The default value of the register MODE1 is incorrect. THe value was %s." % value)
        return False

    # Read the value of the MODE2 register
    (status, value) = common_i2c.read (P, pwm.__MODE2, 0xff)
    if value != 0x4:
        error (lineno(),"The default value of the register MODE2 is incorrect. The value was %s." %value)
        return False

    # Read the value of the SUBADR1 register
    (status, value) = common_i2c.read (P, pwm.__SUBADR1, 0xff)
    if value != 0xE2:
        error (lineno(),"The default value of the register SUBADR1 is incorrect")
        return False

    # Read the value of the SUBADR2 register
    (status, value) = common_i2c.read (P, pwm.__SUBADR2, 0xff)
    if value != 0xE4:
        error (lineno(),"The default value of the register SUBADR2 is incorrect")
        return False

    # Read the value of the SUBADR3 register
    (status, value) = common_i2c.read (P, pwm.__SUBADR3, 0xff)
    if value != 0xE8:
        error (lineno(),"The default value of the register SUBADR3 is incorrect")
        return False

    # Read the value of the ALLCALLADR register
    (status, value) = common_i2c.read (P, pwm.__ALLCALLADR, 0xff)
    if value != 0xE0:
        error (lineno(),"The default value of the register ALLCALLADR is incorrect")
        return False

    # Read the value of the LEDX register
    for i in xrange (16):
        (status, value) = common_i2c.read (P, eval ("pwm.__LED" + str (i) + "_ON_L"), 0xff)
        if value != 0x0:
            error (lineno(),"The default value of the register __LED" + str (i) + "_ON_L is incorrect")
            return False

        (status, value) = common_i2c.read (P, eval ("pwm.__LED" + str (i) + "_ON_H"), 0xff)
        if value != 0x0:
            error (lineno(),"The default value of the register __LED" + str (i) + "_ON_H is incorrect")
            return False

        (status, value) = common_i2c.read (P, eval ("pwm.__LED" + str (i) + "_OFF_L"), 0xff)
        if value != 0x0:
            error (lineno(),"The default value of the register __LED" + str (i) + "_OFF_L is incorrect")
            return False

        (status, value) = common_i2c.read (P, eval ("pwm.__LED" + str (i) + "_OFF_H"), 0xff)
        if value != 0x10:
            error (lineno(),"The default value of the register __LED" + str (i) + "_OFF_H is incorrect")
            return False

    # Read the value of the ALL_LED_ON_L register
    (status, value) = common_i2c.read (P, pwm.__ALL_LED_ON_L, 0xff)
    if value != 0x0:
        error (lineno(),"The default value of the register ALL_LED_ON_L is incorrect")
        return False

    # Read the value of the ALL_LED_ON_H register
    (status, value) = common_i2c.read (P, pwm.__ALL_LED_ON_H, 0xff)
    if value != 0x0:
        error (lineno(),"The default value of the register ALL_LED_ON_H is incorrect")
        return False

    # Read the value of the ALL_LED_OFF_L register
    (status, value) = common_i2c.read (P, pwm.__ALL_LED_OFF_L, 0xff)
    if value != 0x0:
        error (lineno(),"The default value of the register ALL_LED_OFF_L is incorrect")
        return False

    # Read the value of the ALL_LED_OFF_H register
    (status, value) = common_i2c.read (P, pwm.__ALL_LED_OFF_H, 0xff)
    if value != 0x0:
        error (lineno(),"The default value of the register ALL_LED_OFF_H is incorrect")
        return False

    # Read the value of the PRE_SCALE register
    (status, value) = common_i2c.read (P, pwm.__PRE_SCALE, 0xff)
    if value != 0x1E:
        error (lineno(),"The default value of the register PRE_SCALE is incorrect")
        return False

    # Finished test
    del P

    return True

def test_02 ():
    '''
    Check the functions of the PWM API
    '''

    P = pwm.Init ()    
    if P['error'][0]:
        error (lineno(),"Error while creating the PWM. Error message was: " + P['error'][1])
        return False

    # Reset the values of the registers
    reset_p (P)

    #########
    # MODE1 #
    #########

    # Set powermode normal
    (status, message) = pwm.set_powermode (P, 'normal')
    if not status:
        error (lineno(),"Error while setting power mode to normal. Error message was: " + message)
        return False

    (status, powermode) = pwm.get_powermode (P)
    if not status or powermode != 'normal':
        error (lineno(),"Error while getting the power mode. The function to set succeeded but the bit was not set")
        return False

    (status, value) = common_i2c.read (P, pwm.__MODE1, 0x10)
    if value != 0x0:
        error (lineno(),"Error while getting the power mode. The function to set succeeded but the bit was not set")
        return False

    # Put to operate the device for prove the restart function
    (status, message) = pwm.set_pwm (P, 0, 100)
    if not status:
        error (lineno(),"Unable to set pwm signal on the channel 0. The error was %s" % message)
        return False

    # Set powermode sleep
    (status, message) = pwm.set_powermode (P, 'sleep')
    if not status:
        error (lineno(),"Error while setting power mode to sleep. Error message was: " + message)
        return False

    (status, powermode) = pwm.get_powermode (P)
    if not status or powermode != 'sleep':
        error (lineno(),"Error while getting the power mode. The function to set succeeded but the bit was not set")
        return False

    (status, value) = common_i2c.read (P, pwm.__MODE1, 0x10)
    if value != 0x1:
        error (lineno(),"Error while getting the power mode. The function to set succeeded but the bit was not set")
        return False

    # Restart
    (status, message) = pwm.restart (P)
    if not status:
        error (lineno(),"Error while restarting the device. Error message was: " + message)
        return False

    (status, powermode) = pwm.get_powermode (P)
    if not status or powermode != 'normal':
        error (lineno(),"Error while getting the power mode. The function to set succeeded but the bit was not set")
        return False

    # Enable all call address
    (status, message) = pwm.enable_allcall (P, True)
    if not status:
        error (lineno(),"Error while enabling all call address. Error message was: " + message)
        return False

    (status, enabled) = pwm.isenabled_allcall (P)
    if not status or not enabled:
        error (lineno(),"Error while enabling all call address. The function to enable the all call address succeeded but the bit was not set")
        return False

    (status, value) = common_i2c.read (P, pwm.__MODE1, 0x01)
    if value != 0x1:
        error (lineno(),"Error while enabling all call address. The function to enable the all call address succeeded but the bit was not set")
        return False

    # Disable all call address
    (status, message) = pwm.enable_allcall (P, False)
    if not status:
        error (lineno(),"Error while disabling all call address. Error message was: " + message)
        return False

    (status, enabled) = pwm.isenabled_allcall (P)
    if not status or enabled:
        error (lineno(),"Error while disabling all call address. The function to disable the all call address succeeded but the bit was not set")
        return False

    (status, value) = common_i2c.read (P, pwm.__MODE1, 0x01)
    if value != 0x0:
        error (lineno(),"Error while disabling all call address. The function to disable the all call address succeeded but the bit was not set")
        return False

    # Set frequency
    (status, message) = pwm.set_freq (P, 40)
    if not status:
        error (lineno(),"Error while setting frequency. Error message was: " + message)
        return False

    (status, freq) = pwm.get_freq (P)
    if not status or freq != 40:
        error (lineno(),"Error while getting the frequency. The function to set succeeded but the bit was not set")
        return False

    # Set PWM on every channel
    for i in xrange (16):
        (status, message) = pwm.set_pwm (P, i, 90, 90)
        if not status:
            error (lineno(),"Error while setting the pwm on channel %s. The error was %s" % (i, message))
            return False

        (status, values) = pwm.get_pwm (P, i)
        if not status:
            error (lineno(),"Error while getting the pwm on channel %s. The error was %s" % (i, values))
            return False
        
        (duty_cycle, delay) = values
        if duty_cycle != 90 or delay != 90:
            error (lineno(),"Error while getting the pwm on channel %s. The duty cycle or the delay are incorrect, duty_cycle = %s, delay = %s" % (i, duty_cycle, delay))
            return False
        
    # Full values for every channel
    for i in xrange (16):
        # Enable full on 
        (status, message) = pwm.full_on (P, i, True)
        if not status:
            error (lineno(),"Error while enabling full on. Error message was: " + message)
            return False

        (status, enabled) = pwm.isfull_on (P, i)
        if not status or not enabled:
            error (lineno(),"Error while enabling full on. The function to enable the full on succeeded but the bit was not set0")
            return False

        (status, value) = common_i2c.read (P, eval ("pwm.__LED" + str (i) + "_ON_H"), 0x10)
        if value != 0x1:
            error (lineno(),"Error while enabling full on. The function to enable the full on succeeded but the bit was not set1")
            return False

        (status, value) = common_i2c.read (P, eval ("pwm.__LED" + str (i) + "_OFF_H"), 0x10)
        if value != 0x0:
            error (lineno(),"Error while enabling full on. The function to enable the full on succeeded but the bit was not set2")
            return False
        
        # Disable full on
        (status, message) = pwm.full_on (P, i, False)
        if not status:
            error (lineno(),"Error while disabling full on. Error message was: " + message)
            return False
        
        (status, enabled) = pwm.isfull_on (P, i)
        if not status or enabled:
            error (lineno(),"Error while disabling full on. The function to disable the full on succeeded but the bit was not set")
            return False
        
        (status, value) = common_i2c.read (P, eval ("pwm.__LED" + str (i) + "_ON_H"), 0x10)
        if value != 0x0:
            error (lineno(),"Error while disabling full on. The function to disable the full on succeeded but the bit was not set")
            return False
        
        (status, value) = common_i2c.read (P, eval ("pwm.__LED" + str (i) + "_OFF_H"), 0x10)
        if value != 0x0:
            error (lineno(),"Error while disabling full on. The function to disable the full on succeeded but the bit was not set")
            return False
        
        # Enable full off
        (status, message) = pwm.full_off (P, i, True)
        if not status:
            error (lineno(),"Error while enabling full off. Error message was: " + message)
            return False
        
        (status, enabled) = pwm.isfull_off (P, i)
        if not status or not enabled:
            error (lineno(),"Error while enabling full off. The function to enable the full off succeeded but the bit was not set")
            return False
        
        (status, value) = common_i2c.read (P, eval ("pwm.__LED" + str (i) + "_ON_H"), 0x00)
        if value != 0x0:
            error (lineno(),"Error while enabling full off. The function to enable the full off succeeded but the bit was not set")
            return False
        
        (status, value) = common_i2c.read (P, eval ("pwm.__LED" + str (i) + "_OFF_H"), 0x10)
        if value != 0x1:
            error (lineno(),"Error while enabling full off. The function to enable the full off succeeded but the bit was not set")
            return False
        
        # Disable full off
        (status, message) = pwm.full_off (P, i, False)
        if not status:
            error (lineno(),"Error while disabling full off. Error message was: " + message)
            return False
    
        (status, enabled) = pwm.isfull_off (P, i)
        if not status or enabled:
            error (lineno(),"Error while disabling full off. The function to disable the full off succeeded but the bit was not set")
            return False
        
        (status, value) = common_i2c.read (P, eval ("pwm.__LED" + str (i) + "_ON_H"), 0x10)
        if value != 0x0:
            error (lineno(),"Error while disabling full off. The function to disable the full off succeeded but the bit was not set")
            return False
        
        (status, value) = common_i2c.read (P, eval ("pwm.__LED" + str (i) + "_OFF_H"), 0x10)
        if value != 0x0:
            error (lineno(),"Error while disabling full off. The function to disable the full off succeeded but the bit was not set")
            return False

        
    # Finished test
    del P

    return True

###########################
# intrastructure support #
###########################

def lineno():
    """Returns the current line number in our program."""
    return inspect.currentframe().f_back.f_lineno

def info (msg):
    print "[ INFO  ] : " + msg

def error (line,msg):
    if line != None:
        print "[ ERROR ] line %s: %s" % (line, msg)
    else:
        print "[ ERROR ]: %s"  % msg

def ok (msg):
    print "[  OK   ] : " + msg

def run_all_tests ():
    test_count = 0
    for test in tests:
        
         # print log
        info ("TEST-" + str(test_count) + ": Running " + test[1])
        
        # call test
        if not test[0]():
            error (lineno(),"detected test failure at: " + test[1])
            return False

        # next test
        test_count += 1
    
    ok ("All tests ok!")
    return True

# declare list of tests available
tests = [
   (reset_p,   "Reset the values of the registers of the PWM device"),
   (create_p,  "Check the creation of the PWM context"),
   (test_01,   "Check the defaults values of each register of the PWM device"),
   (test_02,   "Check the functions of the PWM API"),
   (reset_p,   "Reset the values of the registers of the PWM device")
]

if __name__ == '__main__':

    # call to run all tests
    run_all_tests ()



# TODO:
# Check the consistency of all registers when one of them is modified
# Check errors on the parameters of the API
# Check parameters in Init function
