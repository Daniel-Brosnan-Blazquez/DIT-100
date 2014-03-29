# -*- coding: utf-8 -*-

# Library for the Raspberry Pi that interfaces with MPC3008 ADC. It is used for battery monitoring

import numpy
import time
import os
import SUNXI_GPIO as GPIO

# Default pin configuration
SPICLK = 267                  # Default clock pin (PI11)
SPIMISO = 269                 # Default master input slave output pin (PI13)
SPIMOSI = 268                 # Default master input slave output pin (PI12)
SPICS = 266                   # Default chip selection pin (PI10)

# Default circuit values
VREF = 5                   # Default value for Vref (when the Rpi is supplied through the BEC)
# VREF = 4.82                   # Used value when Rpi is supplied through the electrical net
APPLY_VOLTAGE_DIVIDER = True  # Indicates if it is needed apply
                              # resistance values to the returned
                              # value
R1 = 100000.0                 # Default value for R1 resistance
R2 = 173000.0                 # Default value for R2 resistance
RESISTANCES = [(R2, R2), (R1, R2), (R1, R2)]

# Default minimum and maximum voltage values of each cell
VMAX = 4.2                    # Default minimum value for the voltage
                              # of each cell
VMIN = 3.6                    # Default maximum value for the voltage
                              # of each cell

def Init (clock = SPICLK, miso = SPIMISO,
          mosi = SPIMOSI, cs = SPICS, min_channel = 0,
          max_channel = 2, vref = VREF,
          apply_voltage_divider = APPLY_VOLTAGE_DIVIDER,
          r = RESISTANCES):
    '''
    clock: indicates clock pin
    miso: indicates master input slave output pin
    mosi: indicates master input slave output pin
    cs: chip selection pin
    min_channel: indicates the minimum channel where receive the analog inputs
    min_channel: indicates the maximum channel where receive the analog inputs
    apply_voltage_divider: indicates wether to apply the voltage divider in the calc of the read voltage
    r1: value of the R1 resistance in the voltage divider
    r2: value of the R2 resistance in the voltage divider
    '''
    ADC = {}
    ADC['clk'] = clock
    ADC['miso'] = miso
    ADC['mosi'] = mosi
    ADC['cs'] = cs
    ADC['vref'] = vref
    ADC['voltage_div'] = apply_voltage_divider
    ADC['r'] = r
    ADC['error'] = (False, None)

    if ((max_channel > 7) or (min_channel < 0)):
        ADC['error'] = (True, "Unable to create the ADC. The values for the channels are not in the range (0,7)")
        return ADC
    
    ADC['maxch'] = max_channel
    ADC['minch'] = min_channel

    # set up the SPI interface pins
    GPIO.init ()
    GPIO.setcfg(mosi, GPIO.OUT)
    GPIO.setcfg(miso, GPIO.IN)
    GPIO.setcfg(clock, GPIO.OUT)
    GPIO.setcfg(cs, GPIO.OUT)


    return ADC

def percentage (value):
    ''' Return the percentage load from the battery '''
    # Get the percentage applying interpolation
    # VMAX  -> 100
    # value -> x
    # VMIN  -> 0
    return 100 + (value - VMAX)* ((0-100) / (VMIN-VMAX))

def readadc(ADC):
    '''
    Read adc data from specified channels
    '''

    # ADC inputs
    readings = {}
    cell = 0
    prev_cell_values = 0
    
    for channel in range(ADC['minch'], ADC['maxch'] + 1):

        # Start the communication with the the device
        # If the device was powered up with the CS line low, it must be
        # brought high and back low to initiate communication.
        # So always here, first the CS signal is put high and then put low.
        GPIO.output(ADC['cs'], True)
        
        # Perform the start signal
        GPIO.output(ADC['clk'], False)
        GPIO.output(ADC['cs'], False)

        
        # Set the command
        # Start and single bits set
        # Examples: let channel 7, 7 | 0x18 = 0x1F = 11111
        #           Start Single/Diff D2 D1 D0
        #             1        1       1  1  1
        #           let channel 5, 5 | 0x18 = 0x1D = 11101
        #           Start Single/Diff D2 D1 D0
        #             1        1       1  0  1
        command = 0
        command = channel | 0x18

        for i in range (5):        
            # Check most significant bit of the five
            if (command & 0x10):
                GPIO.output(ADC['mosi'], True)
            else:
                GPIO.output(ADC['mosi'], False)

            # Shift left the command to send the next bit
            command <<= 1
            GPIO.output(ADC['clk'], True)
            GPIO.output(ADC['clk'], False)
            
        adcout = 0
        # read one empty bit, one null bit and 10 ADC data bits
        for i in range(12):
            GPIO.output(ADC['clk'], True)
            GPIO.output(ADC['clk'], False)

            data = GPIO.input(ADC['miso'])

            # Shift left the reading from the ADC to set the next bit
            adcout <<= 1
                
            # Introduce the received bit
            adcout |= data

        GPIO.output(ADC['cs'], True)

        # Set the reading of the corresponding channel
        # The less significant bit is a meaningless bit (NULL bit)
        adcout >>= 1
        if ADC['voltage_div']:
            print "adcout = %s" % adcout
            # Read voltage from the voltage divider
            vin = (adcout * ADC['vref']) / 1024.0
            # Read voltage from the battery
            r1 = ADC['r'][cell][0]
            r2 = ADC['r'][cell][1]
            value = (vin * (r1 + r2)) / r1
            # Voltage of the corresponding cell
            readings[cell] = (value - prev_cell_values, percentage (value - prev_cell_values))
            prev_cell_values = value
        else:
            # Read voltage from the voltage divider
            vin = (adcout * ADC['vref']) / 1024.0
            readings[cell] = ((adcout * ADC['vref']) / 1024.0, 100.0)
        
        cell +=1
    
    return readings
