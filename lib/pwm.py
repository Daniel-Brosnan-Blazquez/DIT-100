# -*- coding: utf-8 -*-

# Library for the Raspberry Pi that interfaces with PCA9685
# PWM controller on Adafruit board

from smbus import SMBus
from common_i2c import *
import bitOps
import numpy
import time
import os

DEV_SLAVE_ADDR = 0x40         # Default device slave address
DEV_BUS_ID     = 1            # Default device bus id
CLOCK          = 25000000.0   # Internal clock frequency 25MHz
RANGE_OF_PERCENTAGE = 1000.0  # Range of percentage begining at 0
def Init (bus = None, 
          percentage = RANGE_OF_PERCENTAGE, 
          slaveAddr = DEV_SLAVE_ADDR):
    '''
    bus:       indicates the bus associated to the i2c device /dev/i2cX
    slaveAddr: indicates the address of the slave device
    '''
    P = {}
    P['bus'] = bus
    if bus == None:
        try:
            P['bus'] = SMBus(DEV_BUS_ID)
        except IOError as e:
            P['error'] = (True, "Unable to power on the PWM. The error was: I/O error (%s): %s" %(e.errno, e.strerror))
            return P

    P['addr'] = DEV_SLAVE_ADDR
    P['percentage'] = percentage
    P['error'] = (False, None)

    # Power on the device
    (status, error) = set_powermode (P, 'normal')
    if not status:
        P['error'] = (True, "Unable to power on the PWM. The error was: %s" % error)
        return P

    # Disable responds to all call address
    enable_allcall (P, False)

    return P

#############
# REGISTERS #
#############

__MODE1               = Register ('MODE1', 0x0, mode = 'rw')
__MODE2               = Register ('MODE2', 0x1, mode = 'rw')
__SUBADR1             = Register ('SUBADR1', 0x2, mode = 'rw')
__SUBADR2             = Register ('SUBADR2', 0x3, mode = 'rw')
__SUBADR3             = Register ('SUBADR3', 0x4, mode = 'rw')
__ALLCALLADR          = Register ('ALLCALLADR', 0x5, mode = 'rw')

__LED0_ON_L           = Register ('LED0_ON_L', 0x6, mode = 'rw')
__LED0_ON_H           = Register ('LED0_ON_H', 0x7, mode = 'rw')
__LED0_OFF_L          = Register ('LED0_OFF_L', 0x8, mode = 'rw')
__LED0_OFF_H          = Register ('LED0_OFF_H', 0x9, mode = 'rw')

__LED1_ON_L           = Register ('LED1_ON_L', 0xa, mode = 'rw')
__LED1_ON_H           = Register ('LED1_ON_H', 0xb, mode = 'rw')
__LED1_OFF_L          = Register ('LED1_OFF_L', 0xc, mode = 'rw')
__LED1_OFF_H          = Register ('LED1_OFF_H', 0xd, mode = 'rw')

__LED2_ON_L           = Register ('LED2_ON_L', 0xe, mode = 'rw')
__LED2_ON_H           = Register ('LED2_ON_H', 0xf, mode = 'rw')
__LED2_OFF_L          = Register ('LED2_OFF_L', 0x10, mode = 'rw')
__LED2_OFF_H          = Register ('LED2_OFF_H', 0x11, mode = 'rw')

__LED3_ON_L           = Register ('LED3_ON_L', 0x12, mode = 'rw')
__LED3_ON_H           = Register ('LED3_ON_H', 0x13, mode = 'rw')
__LED3_OFF_L          = Register ('LED3_OFF_L', 0x14, mode = 'rw')
__LED3_OFF_H          = Register ('LED3_OFF_H', 0x15, mode = 'rw')

__LED4_ON_L           = Register ('LED4_ON_L', 0x16, mode = 'rw')
__LED4_ON_H           = Register ('LED4_ON_H', 0x17, mode = 'rw')
__LED4_OFF_L          = Register ('LED4_OFF_L', 0x18, mode = 'rw')
__LED4_OFF_H          = Register ('LED4_OFF_H', 0x19, mode = 'rw')

__LED5_ON_L           = Register ('LED5_ON_L', 0x1a, mode = 'rw')
__LED5_ON_H           = Register ('LED5_ON_H', 0x1b, mode = 'rw')
__LED5_OFF_L          = Register ('LED5_OFF_L', 0x1c, mode = 'rw')
__LED5_OFF_H          = Register ('LED5_OFF_H', 0x1d, mode = 'rw')

__LED6_ON_L           = Register ('LED6_ON_L', 0x1e, mode = 'rw')
__LED6_ON_H           = Register ('LED6_ON_H', 0x1f, mode = 'rw')
__LED6_OFF_L          = Register ('LED6_OFF_L', 0x20, mode = 'rw')
__LED6_OFF_H          = Register ('LED6_OFF_H', 0x21, mode = 'rw')

__LED7_ON_L           = Register ('LED7_ON_L', 0x22, mode = 'rw')
__LED7_ON_H           = Register ('LED7_ON_H', 0x23, mode = 'rw')
__LED7_OFF_L          = Register ('LED7_OFF_L', 0x24, mode = 'rw')
__LED7_OFF_H          = Register ('LED7_OFF_H', 0x25, mode = 'rw')

__LED8_ON_L           = Register ('LED8_ON_L', 0x26, mode = 'rw')
__LED8_ON_H           = Register ('LED8_ON_H', 0x27, mode = 'rw')
__LED8_OFF_L          = Register ('LED8_OFF_L', 0x28, mode = 'rw')
__LED8_OFF_H          = Register ('LED8_OFF_H', 0x29, mode = 'rw')

__LED9_ON_L           = Register ('LED9_ON_L', 0x2a, mode = 'rw')
__LED9_ON_H           = Register ('LED9_ON_H', 0x2b, mode = 'rw')
__LED9_OFF_L          = Register ('LED9_OFF_L', 0x2c, mode = 'rw')
__LED9_OFF_H          = Register ('LED9_OFF_H', 0x2d, mode = 'rw')

__LED10_ON_L          = Register ('LED10_ON_L', 0x2e, mode = 'rw')
__LED10_ON_H          = Register ('LED10_ON_H', 0x2f, mode = 'rw')
__LED10_OFF_L         = Register ('LED10_OFF_L', 0x30, mode = 'rw')
__LED10_OFF_H         = Register ('LED10_OFF_H', 0x31, mode = 'rw')

__LED11_ON_L          = Register ('LED11_ON_L', 0x32, mode = 'rw')
__LED11_ON_H          = Register ('LED11_ON_H', 0x33, mode = 'rw')
__LED11_OFF_L         = Register ('LED11_OFF_L', 0x34, mode = 'rw')
__LED11_OFF_H         = Register ('LED11_OFF_H', 0x35, mode = 'rw')

__LED12_ON_L          = Register ('LED12_ON_L', 0x36, mode = 'rw')
__LED12_ON_H          = Register ('LED12_ON_H', 0x37, mode = 'rw')
__LED12_OFF_L         = Register ('LED12_OFF_L', 0x38, mode = 'rw')
__LED12_OFF_H         = Register ('LED12_OFF_H', 0x39, mode = 'rw')

__LED13_ON_L          = Register ('LED13_ON_L', 0x3a, mode = 'rw')
__LED13_ON_H          = Register ('LED13_ON_H', 0x3b, mode = 'rw')
__LED13_OFF_L         = Register ('LED13_OFF_L', 0x3c, mode = 'rw')
__LED13_OFF_H         = Register ('LED13_OFF_H', 0x3d, mode = 'rw')

__LED14_ON_L          = Register ('LED14_ON_L', 0x3e, mode = 'rw')
__LED14_ON_H          = Register ('LED14_ON_H', 0x3f, mode = 'rw')
__LED14_OFF_L         = Register ('LED14_OFF_L', 0x40, mode = 'rw')
__LED14_OFF_H         = Register ('LED14_OFF_H', 0x41, mode = 'rw')

__LED15_ON_L          = Register ('LED15_ON_L', 0x42, mode = 'rw')
__LED15_ON_H          = Register ('LED15_ON_H', 0x43, mode = 'rw')
__LED15_OFF_L         = Register ('LED15_OFF_L', 0x44, mode = 'rw')
__LED15_OFF_H         = Register ('LED15_OFF_H', 0x45, mode = 'rw')

__ALL_LED_ON_L        = Register ('ALL_LED_ON_L', 0xfa, mode = 'rw')
__ALL_LED_ON_H        = Register ('ALL_LED_ON_H', 0xfb, mode = 'rw')
__ALL_LED_OFF_L       = Register ('ALL_LED_OFF_L', 0xfc, mode = 'rw')
__ALL_LED_OFF_H       = Register ('ALL_LED_OFF_H', 0xfd, mode = 'rw')

__PRE_SCALE           = Register ('PRE_SCALE', 0xfe, mode = 'rw')

#################################
# MASKS TO MODIFY THE REGISTERS #
#################################

# MODE1
__MASK_ALLCALL        = 0x01      # Enable All call 
__MASK_SUB1           = 0x02      # Subaddress 1
__MASK_SUB2           = 0x04      # Subaddress 2
__MASK_SUB3           = 0x08      # Subaddress 3
__MASK_SLEEP          = 0x10      # Power mode
__MASK_AI             = 0x20      # Enable autoincrement
__MASK_EXTCLK         = 0x40      # Use external clock
__MASK_RESTART        = 0x80      # Enable restart

# MODE2
__MASK_OUTNET         = 0x03      # Enable output drivers
__MASK_OUTDRV         = 0x04      # Configure outputs
__MASK_OCH            = 0x08      # Outputs chance
__MASK_INVRT          = 0x10      # Outputs chance

# PWM channel
__MASK_ONOFF          = 0x10      # On the PWM channel
__MASK_HIGH           = 0x0F      # High part of the value
__MASK_LOW            = 0xFF      # Low part of the value

# Subaddress
__MASK_ADR         = 0xFE      # Subaddress

##########################
# VALUE REGISTER MAPPING #
##########################

# Enable/disable
__Enable = {True: 0x1, False: 0x0}

# Power mode
__PowerMode = {'normal': 0x0, 'sleep': 0x1}

#############
# FUNCTIONS #
#############
def enable_restart(P):
    ''' Enable restart '''
    (status, message) = write(P, __MODE1, __MASK_RESTART, __Enable[True])
    if not status:
        return (False, 'Unable to enable restart operation for the PWM device. The error was %s' % message)
    return (True, None)
    
def isenabled_restart(P):
    ''' Check if restart operation is enabled '''
    (status, enabled) = read_match(P, __MODE1, __MASK_RESTART, __Enable)
    if not status:
        return (False, 'Unable to check if restart operation is enabled for the PWM device. The error was %s' % enabled)
    return (True, enabled)

def restart (P):
    ''' Restart all the previously active PWM channels '''
    (status, enabled) = isenabled_restart (P)
    if not status:
        return (False, "Unable to check if restart is enabled on the PWM device. The error was %s" % enabled)

    if not enabled:
        return (False, "Unable to restart the PWM device. The restart bit is not set.")

    (status, message) = set_powermode (P, 'normal')
    if not status:
        return (False, "Unable to restart the previously active PWM channels. The error was %s" % message)

    time.sleep (0.001)

    (status, message) = enable_restart (P)
    if not status:
        return (False, "Unable to restart the previously active PWM channels. The error was %s" % message)

    return (True, None)

def enable_ai (P, enable):
    '''Enable or disable auto-increment '''
    (status, message) = write (P, __MODE1, __MASK_AI, __Enable[enable])
    if not status:
        return (False, "Unable to enable or disable auto-increment on the PWM device. The error was %s" % message)

    return (True, None)

def set_powermode (P, mode):
    ''' Set power mode '''
    if mode not in __PowerMode.keys():
        return (False, 'Mode %s not in range of allowed power mode values for the PWM device' % mode)
    
    (status, message) = write (P, __MODE1, __MASK_SLEEP, __PowerMode[mode])
    if not status:
        return (False, 'Unable to set the power mode on the PWM device. The error was %s' % message)

    return (True, None)

def get_powermode(P):
    ''' Get power mode '''
    # Check power-down mode
    (status, powermode) = read_match(P, __MODE1, __MASK_SLEEP, __PowerMode)

    if not status:
        return (False, 'Unable to get the power mode on the PWM device. The error was %s' % powermode)

    return (True, powermode)

def enable_allcall(P, enable):
    ''' Enable or disable responds to all call address '''
    (status, message) = write(P, __MODE1, __MASK_ALLCALL, __Enable[enable])
    if not status:
        return (False, 'Unable to enable or disable responds to all call address for the PWM device. The error was %s' % message)
    return (True, None)

def isenabled_allcall(P):
    ''' Check if it is enabled responds to all call address '''
    (status, enabled) = read_match(P, __MODE1, __MASK_ALLCALL, __Enable)
    if not status:
        return (False, 'Unable to check if it is enabled responds to all call address for the PWM device. The error was %s' % enabled)
    return (True, enabled)

def set_freq(P, freq):
    ''' Set the PWM frequency '''
    if type (freq) != int:
        return (False, "Unable to set the PWM frequency. The frequency is not an integer value, is of type %s" % type (freq))
    if freq < 40 or freq > 1000:
        return (False, "Unable to set the PWM frequency. The value is not in the range of (40,1000)")

    # Calculate the prescale value
    prescale = int (round (CLOCK / (4096*freq)) - 1)

    (status, message) = set_powermode (P, 'sleep')
    if not status:
        return (False, "Unable to set the frequency on the PWM device. The error was %s.", message)

    (status, message) = write(P, __PRE_SCALE, 0xFF, prescale) 
    if not status:
        return (False, "Unable to set the frequency on the PWM device. The error was %s.", message)

    (status, message) = set_powermode (P, 'normal')
    if not status:
        return (False, "Unable to set the frequency on the PWM device. The error was %s.", message)

    return (True, None)

def get_freq(P):
    ''' Get the PWM frequency '''
    (status, out) = read(P, __PRE_SCALE, 0xFF)
    if not status:
        return (False, "Unable to get the frequency on the PWM device. The error was %s.", out)

    # Calculate frequency
    period = ((out+1) * 4096) / CLOCK
    freq = round (1.0/period, 0)
    
    return (True, freq)

def percentage_to_counts (P, p):
    ''' Convert the percentage value to counts '''
    return int (round ((p * 4095) / P['percentage'], 0))

def counts_to_percentage (P, c):
    ''' Convert the counts value to percentage '''
    return int (round ((c * P['percentage']) / 4095.0, 0))

def set_pwm (P, channel, duty_cycle, delay = 0, change_delay = True):
    ''' 
    Set the pwm on the channel provided 
    channel: channle number to configure (0-15)
    duty_cycle: percentage of cycle that the signal is high
    delay: percentage of cycle to wait to put on the signal (default = 0)
    '''
    if type (channel) != int:
        return (False, "Unable to set the pwm configuration. The channel is not an integer value, is of type %s" % type (channel))
    if channel < 0 or channel > 15:
        return (False, "Unable to set the pwm configuration. The channel is not in the range of (0,15)")

    if type (duty_cycle) != int:
        return (False, "Unable to set the pwm configuration. The duty_cycle is not an integer value")
    if duty_cycle < 0 or duty_cycle > P['percentage']:
        return (False, "Unable to set the pwm configuration. The duty_cycle is not in the range of (0,%s)" % P['percentage'])

    if type (delay) != int:
        return (False, "Unable to set the pwm configuration. The delay is not an integer value")
    if delay < 0 or delay > P['percentage']:
        return (False, "Unable to set the pwm configuration. The delay is not in the range of (0,%s)" % P['percentage'])
    
    # Obtain the on part
    on = percentage_to_counts (P,delay)

    # Obtain the off part
    delay_on = on + percentage_to_counts (P,duty_cycle)
    off = delay_on
    if delay_on > 4095:
        off = delay_on - 4095

    # Write the values to the proper registers
    on_h = (on & 0xF00) >> 8
    on_l = on & 0xFF

    off_h = (off & 0xF00) >> 8
    off_l = off & 0xFF

    # Check if it is needed to change delay value. This is to reduce
    # time
    if change_delay:
        # LOW ON
        (status, message) = write(P, eval ("__LED" + str (channel) + "_ON_L"), 0xFF, on_l, get_current_value = True) 
        if not status:
            return (False, "Unable to set the low on part on the PWM device. The error was %s.", message)

        # HIGH ON
        (status, message) = write(P, eval ("__LED" + str (channel) + "_ON_H"), 0xFF, on_h, get_current_value = True) 
        if not status:
            return (False, "Unable to set the high on part on the PWM device. The error was %s.", message)

    # LOW OFF
    (status, message) = write(P, eval ("__LED" + str (channel) + "_OFF_L"), 0xFF, off_l, get_current_value = True) 
    if not status:
        return (False, "Unable to set the low off part on the PWM device. The error was %s." % message)

    # HIGH OFF
    (status, message) = write(P, eval ("__LED" + str (channel) + "_OFF_H"), 0xFF, off_h, get_current_value = True) 
    if not status:
        return (False, "Unable to set the high off part on the PWM device. The error was %s." % message)

    return (True, None)

def set_pwm_ai (P, start_channel, values):
    ''' 
    Set the duty cycle on the channels starting on start_chanel untill
    start_chanel + len (values)
    start_channel: channle number start writing the values (0-15)
    values: every value that represents the percentage of cycle that the signal is high and the percentage of cycle to wait to put on the signal
    '''
    # Get the number of values
    number_of_values = len (values)

    if type (start_channel) != int:
        return (False, "Unable to set the pwm configuration. The channel is not an integer value, is of type %s" % type (start_channel))
    if start_channel < 0 or (start_channel + number_of_values-1) > 15:
        return (False, "Unable to set the pwm configuration. The channel is not in the range of (0,15)")

    values_to_write = []

    for value in values:
        duty_cycle = value[0]
        delay = value[1]
        # Obtain the on part
        on = percentage_to_counts (P,delay)

        # Obtain the off part
        delay_on = on + percentage_to_counts (P,duty_cycle)
        off = delay_on
        if delay_on > 4095:
            off = delay_on - 4095

        # Obtain the high and the low parts
        on_h = (on & 0xF00) >> 8
        on_l = on & 0xFF

        off_h = (off & 0xF00) >> 8
        off_l = off & 0xFF

        values_to_write += [on_l, on_h, off_l, off_h]

    # Write values
    (status, message) = write_block (P, eval ("__LED" + str (start_channel) + "_ON_L").get_addr (), values_to_write)
    if not status:
        return (False, "Unable to set the off part on the PWM device. The error was %s." % message)

    return (True, None)

def write_counts (P, channel, off, on = 0):

    if type (channel) != int:
        return (False, "Unable to set the pwm configuration. The channel is not an integer value, is of type %s" % type (channel))
    if channel < 0 or channel > 15:
        return (False, "Unable to set the pwm configuration. The channel is not in the range of (0,15)")

    if type (on) != int:
        return (False, "Unable to set the pwm configuration. The on is not an integer value")
    if on < 0 or on > 4095:
        return (False, "Unable to set the pwm configuration. The on is not in the range of (0,4095)")

    if type (off) != int:
        return (False, "Unable to set the pwm configuration. The off is not an integer value")
    if off < 0 or off > 4095:
        return (False, "Unable to set the pwm configuration. The off is not in the range of (0,4095)")

    if off > 4095:
        off -= 4095

    # Write the values to the proper registers
    on_h = (on & 0xF00) >> 8
    on_l = on & 0xFF

    off_h = (off & 0xF00) >> 8
    off_l = off & 0xFF

    # LOW ON
    (status, message) = write(P, eval ("__LED" + str (channel) + "_ON_L"), 0xFF, on_l) 
    if not status:
        return (False, "Unable to set the frequency on the PWM device. The error was %s.", message)

    # HIGH ON
    (status, message) = write(P, eval ("__LED" + str (channel) + "_ON_H"), 0xFF, on_h) 
    if not status:
        return (False, "Unable to set the frequency on the PWM device. The error was %s.", message)

    # LOW OFF
    (status, message) = write(P, eval ("__LED" + str (channel) + "_OFF_L"), 0xFF, off_l) 
    if not status:
        return (False, "Unable to set the frequency on the PWM device. The error was %s.", message)

    # HIGH OFF
    (status, message) = write(P, eval ("__LED" + str (channel) + "_OFF_H"), 0xFF, off_h) 
    if not status:
        return (False, "Unable to set the frequency on the PWM device. The error was %s.", message)

    return (True, None)

def get_pwm (P, channel):
    ''' Get the duty cycle of the specified channel '''
    if type (channel) != int:
        return (False, "Unable to get the duty cycle. The channel is not an integer value, is of type %s" % type (channel))
    if channel < 0 or channel > 15:
        return (False, "Unable to get the duty cycle. The channel is not in the range of (0,15)")

    # LOW ON
    (status, on_l) = read(P, eval ("__LED" + str (channel) + "_ON_L"), 0xFF) 
    if not status:
        return (False, "Unable to get the duty cycle on the PWM device. The error was %s.", on_l)

    # HIGH ON
    (status, on_h) = read(P, eval ("__LED" + str (channel) + "_ON_H"), 0xFF) 
    if not status:
        return (False, "Unable to get the duty cycle on the PWM device. The error was %s.", on_h)

    # LOW OFF
    (status, off_l) = read(P, eval ("__LED" + str (channel) + "_OFF_L"), 0xFF) 
    if not status:
        return (False, "Unable to get the duty cycle on the PWM device. The error was %s.", off_l)

    # HIGH OFF
    (status, off_h) = read(P, eval ("__LED" + str (channel) + "_OFF_H"), 0xFF) 
    if not status:
        return (False, "Unable to get the duty cycle on the PWM device. The error was %s.", off_h)

    on = (on_h << 8) + on_l
    delay = counts_to_percentage (P,on)

    off = (off_h << 8) + off_l
    if off < on:
        duty_cycle = counts_to_percentage (P,(off + 4096) - on)
    else:
        duty_cycle = counts_to_percentage (P,off - on)

    return (True, (duty_cycle, delay))
    
    
def full_on (P, channel, enable = True):
    ''' 
    Set the full on the signal on the channel provided 
    channel: channle number to configure (0-15)
    '''
    if type (channel) != int:
        return (False, "Unable to set the pwm configuration. The channel is not an integer value, is of type %s" % type (channel))
    if channel < 0 or channel > 15:
        return (False, "Unable to set the pwm configuration. The channel is not in the range of (0,15)")

    value = int (enable) * 0x10

    # LOW ON
    (status, message) = write(P, eval ("__LED" + str (channel) + "_ON_L"), 0xFF, 0x0) 
    if not status:
        return (False, "Unable to set the frequency on the PWM device. The error was %s.", message)

    # HIGH ON
    (status, message) = write(P, eval ("__LED" + str (channel) + "_ON_H"), 0xFF, value) 
    if not status:
        return (False, "Unable to set the frequency on the PWM device. The error was %s.", message)

    # LOW OFF
    (status, message) = write(P, eval ("__LED" + str (channel) + "_OFF_L"), 0xFF, 0x0) 
    if not status:
        return (False, "Unable to set the frequency on the PWM device. The error was %s.", message)

    # HIGH OFF
    (status, message) = write(P, eval ("__LED" + str (channel) + "_OFF_H"), 0xFF, 0x0) 
    if not status:
        return (False, "Unable to set the frequency on the PWM device. The error was %s.", message)

    return (True, None)

def isfull_on(P, channel):
    ''' Check if the signal on the channel is full on '''
    if type (channel) != int:
        return (False, "Unable to check if the signal on the channel is full on for the PWM device. The channel is not an integer value, is of type %s" % type (channel))
    if channel < 0 or channel > 15:
        return (False, "Unable to check if the signal on the channel is full on for the PWM device. The channel is not in the range of (0,15)")

    # HIGH ON
    (status, on_h) = read(P, eval ("__LED" + str (channel) + "_ON_H"), 0x10) 
    if not status:
        return (False, "Unable to check if the signal on the channel is full on for the PWM device. The error was %s.", on_h)

    # HIGH OFF
    (status, off_h) = read(P, eval ("__LED" + str (channel) + "_OFF_H"), 0x10) 
    if not status:
        return (False, "Unable to check if the signal on the channel is full on for the PWM device. The error was %s.", off_h)

    return (True, on_h and not off_h)

def full_off (P, channel, enable = True):
    ''' 
    Set the full off the signal on the channel provided 
    channel: channle number to configure (0-15)
    '''
    if type (channel) != int:
        return (False, "Unable to set the pwm configuration. The channel is not an integer value, is of type %s" % type (channel))
    if channel < 0 or channel > 15:
        return (False, "Unable to set the pwm configuration. The channel is not in the range of (0,15)")
    
    value = int (enable) * 0x10

    # LOW ON
    (status, message) = write(P, eval ("__LED" + str (channel) + "_ON_L"), 0xFF, 0x0) 
    if not status:
        return (False, "Unable to set the frequency on the PWM device. The error was %s.", message)

    # HIGH ON
    (status, message) = write(P, eval ("__LED" + str (channel) + "_ON_H"), 0xFF, 0x00) 
    if not status:
        return (False, "Unable to set the frequency on the PWM device. The error was %s.", message)

    # LOW OFF
    (status, message) = write(P, eval ("__LED" + str (channel) + "_OFF_L"), 0xFF, 0x0) 
    if not status:
        return (False, "Unable to set the frequency on the PWM device. The error was %s.", message)

    # HIGH OFF
    (status, message) = write(P, eval ("__LED" + str (channel) + "_OFF_H"), 0xFF, value) 
    if not status:
        return (False, "Unable to set the frequency on the PWM device. The error was %s.", message)

    return (True, None)

def isfull_off(P, channel):
    ''' Check if the signal on the channel is full off '''
    if type (channel) != int:
        return (False, "Unable to check if the signal on the channel is full off for the PWM device. The channel is not an integer value, is of type %s" % type (channel))
    if channel < 0 or channel > 15:
        return (False, "Unable to check if the signal on the channel is full off for the PWM device. The channel is not in the range of (0,15)")

    # HIGH OFF
    (status, off_h) = read(P, eval ("__LED" + str (channel) + "_OFF_H"), 0x10) 
    if not status:
        return (False, "Unable to check if the signal on the channel is full off for the PWM device. The error was %s.", off_h)
    
    return (True, bool (off_h))

def reset (P = None):
    '''
    Reset the values of the registers
    '''
    
    delete_pwm = False
    # Create the pwm
    if P == None:
        delete_pwm = True
        P = Init ()
        if P['error'][0]:
            return (False, "Error while creating the PWM. Error message was: " + P['error'][1])

    # Write the default value of the MODE1 register
    (status, message) = write (P, __MODE1, 0xff, 0x00)
    if not status:
        return (False, "Error while writing the default value of the MODE1 register of the PWM device. Error message was: " + message)

    # Write the default value of the MODE2 register
    (status, message) = write (P, __MODE2, 0xff, 0x4)
    if not status:
        return (False, "Error while writing the default value of the MODE2 register of the PWM device. Error message was: " + message)

    # Write the default value of the SUBADR1 register
    (status, message) = write (P, __SUBADR1, 0xff, 0xE2)
    if not status:
        return (False, "Error while writing the default value of the SUBADR1 register of the PWM device. Error message was: " + message)

    # Write the default value of the SUBADR2 register
    (status, message) = write (P, __SUBADR2, 0xff, 0xE4)
    if not status:
        return (False, "Error while writing the default value of the SUBADR2 register of the PWM device. Error message was: " + message)

    # Write the default value of the SUBADR3 register
    (status, message) = write (P, __SUBADR3, 0xff, 0xE8)
    if not status:
        return (False, "Error while writing the default value of the SUBADR3 register of the PWM device. Error message was: " + message)

    # Write the default value of the ALLCALLADR register
    (status, message) = write (P, __ALLCALLADR, 0xff, 0xE0)
    if not status:
        return (False, "Error while writing the default value of the ALLCALLADR register of the PWM device. Error message was: " + message)

    # Write the default value of the LEDX register
    for i in xrange (16):
        (status, message) = write (P, eval ("__LED" + str (i) + "_ON_L"), 0xff, 0x0)
        if not status:
            return (False, "Error while writing the default value of the LEDX_ON_L register of the PWM device. Error message was: " + message)

        (status, message) = write (P, eval ("__LED" + str (i) + "_ON_H"), 0xff, 0x0)
        if not status:
            return (False, "Error while writing the default value of the LEDX_ON_H register of the PWM device. Error message was: " + message)

        (status, message) = write (P, eval ("__LED" + str (i) + "_OFF_L"), 0xff, 0x0)
        if not status:
            return (False, "Error while writing the default value of the LEDX_OFF_L register of the PWM device. Error message was: " + message)

        (status, message) = write (P, eval ("__LED" + str (i) + "_OFF_H"), 0xff, 0x10)
        if not status:
            return (False, "Error while writing the default value of the LEDX_OFF_H register of the PWM device. Error message was: " + message)
    
    # Write the default value of the ALL_LED_ON_L register
    (status, message) = write (P, __ALL_LED_ON_L, 0xff, 0x0)
    if not status:
        return (False, "Error while writing the default value of the ALL_LED_ON_L register of the PWM device. Error message was: " + message)

    # Write the default value of the ALL_LED_ON_H register
    (status, message) = write (P, __ALL_LED_ON_H, 0xff, 0x00)
    if not status:
        return (False, "Error while writing the default value of the ALL_LED_ON_H register of the PWM device. Error message was: " + message)

    # Write the default value of the ALL_LED_OFF_L register
    (status, message) = write (P, __ALL_LED_OFF_L, 0xff, 0x00)
    if not status:
        return (False, "Error while writing the default value of the ALL_LED_OFF_L register of the PWM device. Error message was: " + message)

    # Write the default value of the ALL_LED_OFF_H register
    (status, message) = write (P, __ALL_LED_OFF_H, 0xff, 0x10)
    if not status:
        return (False, "Error while writing the default value of the ALL_LED_OFF_H register of the PWM device. Error message was: " + message)

    (status, message) = set_powermode (P, 'sleep')
    if not status:
        return (False, "Unable to set the frequency on the PWM device. The error was %s.", message)

    # Write the default value of the PRE_SCALE register
    (status, message) = write (P, __PRE_SCALE, 0xff, 0x1E)
    if not status:
        return (False, "Error while writing the default value of the PRE_SCALE register of the PWM device. Error message was: " + message)

    (status, message) = set_powermode (P, 'normal')
    if not status:
        return (False, "Unable to set the frequency on the PWM device. The error was %s.", message)

    # Finished test
    if delete_pwm:
        del P
    
    return (True, None)


