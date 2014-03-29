#!/usr/bin/python
# -*- coding: utf-8 -*-

# import sys for command line parsing
import sys
sys.path.append ("../lib")

import time
import gyro
import accel
import mag
import alt
import common_i2c
import inspect

imu_ex = "../examples/IMU/"

####################
# regression tests #
####################

########
# GYRO #
########

def create_g ():
    ''' 
    Check the creation of the gyro context
    '''
    # Default parameters
    G = gyro.Init (scale = '250dps')
    if G['error'][0]:
        error (lineno(),"Error while creating the gyro. Error message was: " + G['error'][1])
        return False

    if G['addr'] != 0x6b:
        error (lineno(),"Error while creating the gyro context. The slave address was incorrect. Correct slave address: " + hex (0x6b) + ", storaged slave address: " + hex (G['addr']))
        return False

    if G['cal_iter'] != 20:
        error (lineno(),"Error while creating the gyro context. The number of iterations for calibration was incorrect. Correct number: " + str (20) + ", storaged number: " + str (G['cal_iter']))
        return False
    
    if G['gain'] != 0.00875:
        error (lineno(),"Error while creating the gyro context. The value of the gain was incorrect. Correct value: " + str (0.00875) + ", storaged number: " + str (G['gain']))
        return False

    (status, scale) = gyro.get_scale (G)
    if not status:
        error (lineno(),"Error while getting the scale. Error message was: " + scale)        
        return False
    
    if scale != '250dps':
        error (lineno(),"Error while setting the scale. The function to set succeeded but the bits were not set")
        return False

    # Finished test
    del G
    
    return True

def reset_g (G = None):
    '''
    Reset the values of the registers of the gyroscope device
    '''
    (status, message) = gyro.reset (G)
    if not status:
        error (lineno(),"Error while reseting the gyro device. The error was %s." % message)
        return False
    
    return True

def test_00 ():
    '''
    Check the default values
    '''

    G = gyro.Init ()    
    if G['error'][0]:
        error (lineno(),"Error while creating the gyro. Error message was: " + G['error'][1])
        return False

    # Read the value of the WHO_AM_I register
    (status, value) = common_i2c.read (G, gyro.__WHO_AM_I, 0xff)
    if value != 0xd4:
        error (lineno(),"The default value of the register WHO_AM_I is incorrect. It should be 0xd4 and it was %s" % value)
        return False

    # Read the value of the CTRL_REG1 register
    (status, value) = common_i2c.read (G, gyro.__CTRL_REG1, 0xff)
    if value != 0xF:
        error (lineno(),"The default value of the register CTRL_REG1 is incorrect")
        return False

    # Read the value of the CTRL_REG2 register
    (status, value) = common_i2c.read (G, gyro.__CTRL_REG2, 0xff)
    if value != 0x0:
        error (lineno(),"The default value of the register CTRL_REG2 is incorrect")
        return False

    # Read the value of the CTRL_REG3 register
    (status, value) = common_i2c.read (G, gyro.__CTRL_REG3, 0xff)
    if value != 0x0:
        error (lineno(),"The default value of the register CTRL_REG3 is incorrect")
        return False

    # Read the value of the CTRL_REG4 register
    (status, value) = common_i2c.read (G, gyro.__CTRL_REG4, 0xff)
    if value != 0x0:
        error (lineno(),"The default value of the register CTRL_REG4 is incorrect")
        return False
    
    # Read the value of the CTRL_REG5 register
    (status, value) = common_i2c.read (G, gyro.__CTRL_REG5, 0xff)
    if value != 0x0:
        error (lineno(),"The default value of the register CTRL_REG5 is incorrect")
        return False

    # Read the value of the REFERENCE register
    (status, value) = common_i2c.read (G, gyro.__REFERENCE, 0xff)
    if value != 0x0:
        error (lineno(),"The default value of the register REFERENCE is incorrect")
        return False
    
    # Read the value of the FIFO_CTRL_REG register
    (status, value) = common_i2c.read (G, gyro.__FIFO_CTRL_REG, 0xff)
    if value != 0x0:
        error (lineno(),"The default value of the register FIFO_CTRL_REG is incorrect")
        return False

    # Read the value of the CTRL_REG2 register
    (status, value) = common_i2c.read (G, gyro.__CTRL_REG2, 0xff)
    if value != 0x0:
        error (lineno(),"The default value of the register CTRL_REG2 is incorrect")
        return False

    # Finished test
    del G

    return True

def test_01 ():
    '''
    Check that the read only registers are not writable
    '''

    G = gyro.Init ()    
    if G['error'][0]:
        error (lineno(),"Error while creating the gyro. Error message was: " + G['error'][1])
        return False

    # Reset the values of the registers
    reset_g (G)

    # Try to write in the WHO_AM_I register
    (status, message) = common_i2c.write (G, gyro.__WHO_AM_I, 0xff, 0xFF)
    if status:
        error (lineno(),"Success on the attempt to write in the unwritable register WHO_AM_I")
        return False

    # Try to write in the OUT_TEMP register
    (status, message) = common_i2c.write (G, gyro.__OUT_TEMP, 0xff, 0xFF)
    if status:
        error (lineno(),"Success on the attempt to write in the unwritable register OUT_TEMP")
        return False

    # Try to write in the STATUS_REG register
    (status, message) = common_i2c.write (G, gyro.__STATUS_REG, 0xff, 0xFF)
    if status:
        error (lineno(),"Success on the attempt to write in the unwritable register STATUS_REG")
        return False
    
    # Try to write in the OUT_X_L register
    (status, message) = common_i2c.write (G, gyro.__OUT_X_L, 0xff, 0xFF)
    if status:
        error (lineno(),"Success on the attempt to write in the unwritable register OUT_X_L")
        return False

    # Try to write in the OUT_X_H register
    (status, message) = common_i2c.write (G, gyro.__OUT_X_H, 0xff, 0xFF)
    if status:
        error (lineno(),"Success on the attempt to write in the unwritable register OUT_X_H")
        return False

    # Try to write in the OUT_Y_L register
    (status, message) = common_i2c.write (G, gyro.__OUT_Y_L, 0xff, 0xFF)
    if status:
        error (lineno(),"Success on the attempt to write in the unwritable register OUT_Y_L")
        return False

    # Try to write in the OUT_Y_H register
    (status, message) = common_i2c.write (G, gyro.__OUT_Y_H, 0xff, 0xFF)
    if status:
        error (lineno(),"Success on the attempt to write in the unwritable register OUT_Y_H")
        return False

    # Try to write in the OUT_Z_L register
    (status, message) = common_i2c.write (G, gyro.__OUT_Z_L, 0xff, 0xFF)
    if status:
        error (lineno(),"Success on the attempt to write in the unwritable register OUT_Z_L")
        return False

    # Try to write in the OUT_Z_H register
    (status, message) = common_i2c.write (G, gyro.__OUT_Z_H, 0xff, 0xFF)
    if status:
        error (lineno(),"Success on the attempt to write in the unwritable register OUT_Z_H")
        return False

    # Try to write in the FIFO_SRC_REG register
    (status, message) = common_i2c.write (G, gyro.__FIFO_SRC_REG, 0xff, 0xFF)
    if status:
        error (lineno(),"Success on the attempt to write in the unwritable register FIFO_SRC_REG")
        return False

    # Finished test
    del G
    
    return True

def test_02 ():
    '''
    Check the functions of the gyro API
    '''

    G = gyro.Init ()    
    if G['error'][0]:
        error (lineno(),"Error while creating the gyro. Error message was: " + G['error'][1])
        return False

    # Reset the values of the registers
    reset_g (G)

    (status, value) = gyro.get_deviceid (G)
    if value != 0xd4:
        error (lineno(),"The value of the register WHO_AM_I is incorrect")
        return False

    #############
    # CTRL_REG1 #
    #############

    # Enable X
    (status, message) = gyro.enable_x (G, True)
    if not status:
        error (lineno(),"Error while enabling axis X. Error message was: " + message)
        return False

    (status, enabled) = gyro.isenabled_x (G)
    if not status or not enabled:
        error (lineno(),"Error while enabling axis X. The function to enable the axis succeeded but the bit was not set")
        return False

    (status, value) = common_i2c.read (G, gyro.__CTRL_REG1, 0x01)
    if value != 0x1:
        error (lineno(),"Error while enabling axis X. The function to enable the axis succeeded but the bit was not set")
        return False

    # Disable X
    (status, message) = gyro.enable_x (G, False)
    if not status:
        error (lineno(),"Error while disabling axis X. Error message was: " + message)
        return False

    (status, enabled) = gyro.isenabled_x (G)
    if not status or enabled:
        error (lineno(),"Error while disabling axis X. The function to disaable the axis succeeded but the bit was not set")
        return False

    (status, value) = common_i2c.read (G, gyro.__CTRL_REG1, 0x01)
    if value != 0x0:
        error (lineno(),"Error while disabling axis X. The function to disaable the axis succeeded but the bit was not set")
        return False

    # Enable Y
    (status, message) = gyro.enable_y (G, True)
    if not status:
        error (lineno(),"Error while enabling axis Y. Error message was: " + message)
        return False

    (status, enabled) = gyro.isenabled_y (G)
    if not status or not enabled:
        error (lineno(),"Error while enabling axis Y. The function to enable the axis succeeded but the bit was not set")
        return False

    (status, value) = common_i2c.read (G, gyro.__CTRL_REG1, 0x02)
    if value != 0x1:
        error (lineno(),"Error while enabling axis Y. The function to enable the axis succeeded but the bit was not set")
        return False

    # Disable Y
    (status, message) = gyro.enable_y (G, False)
    if not status:
        error (lineno(),"Error while disabling axis Y. Error message was: " + message)
        return False

    (status, enabled) = gyro.isenabled_y (G)
    if not status or enabled:
        error (lineno(),"Error while disabling axis Y. The function to disaable the axis succeeded but the bit was not set")
        return False

    (staus, value) = common_i2c.read (G, gyro.__CTRL_REG1, 0x02)
    if value != 0x0:
        error (lineno(),"Error while disabling axis Y. The function to disaable the axis succeeded but the bit was not set")
        return False

    # Enable Z
    (status, message) = gyro.enable_z (G, True)
    if not status:
        error (lineno(),"Error while enabling axis Z. Error message was: " + message)
        return False

    (status, enabled) = gyro.isenabled_z (G)
    if not status or not enabled:
        error (lineno(),"Error while enabling axis Z. The function to enable the axis succeeded but the bit was not set")
        return False

    (status, value) = common_i2c.read (G, gyro.__CTRL_REG1, 0x04)
    if value != 0x1:
        error (lineno(),"Error while enabling axis Z. The function to enable the axis succeeded but the bit was not set")
        return False

    # Disable Z
    (status, message) = gyro.enable_z (G, False)
    if not status:
        error (lineno(),"Error while disabling axis Z. Error message was: " + message)
        return False

    (status, enabled) = gyro.isenabled_z (G)
    if not status or enabled:
        error (lineno(),"Error while disabling axis Z. The function to disaable the axis succeeded but the bit was not set")
        return False

    (status, value) = common_i2c.read (G, gyro.__CTRL_REG1, 0x04)
    if value != 0x0:
        error (lineno(),"Error while disabling axis Z. The function to disaable the axis succeeded but the bit was not set")
        return False

    # Set power-down mode
    (status, message) = gyro.set_powerdownmode (G)
    if not status:
        error (lineno(),"Error while setting power-down mode. Error message was: " + message)
        return False

    (status, powermode) = gyro.get_powermode (G)
    if not status or powermode != 'power_down':
        error (lineno(),"Error while setting power-down mode. The function to set the mode succeeded but the bit was not set")
        return False

    (status, value) = common_i2c.read (G, gyro.__CTRL_REG1, 0x08)
    if value != 0x0:
        error (lineno(),"Error while setting power-down mode. The function to set the mode succeeded but the bit was not set")
        return False

    # Set normal mode
    gyro.enable_x (G, True)
    (status, message) = gyro.poweron (G)
    if not status:
        error (lineno(),"Error while setting normal mode. Error message was: " + message)
        return False

    (status, powermode) = gyro.get_powermode (G)
    if not status or powermode != 'normal':
        error (lineno(),"Error while setting normal mode. The function to set the mode succeeded but the bit was not set")
        return False

    (status, value) = common_i2c.read (G, gyro.__CTRL_REG1, 0x08)
    if value != 0x1:
        error (lineno(),"Error while setting normal mode. The function to set the mode succeeded but the bit was not set")
        return False

    # Set sleep mode
    (status, message) = gyro.set_sleepmode (G)
    if not status:
        error (lineno(),"Error while setting sleep mode. Error message was: " + message)
        return False

    (status, powermode) = gyro.get_powermode (G)
    if not status or powermode != 'sleep':
        error (lineno(),"Error while setting sleep mode. The function to set the mode succeeded but the bit was not set")
        return False

    (status, value) = common_i2c.read (G, gyro.__CTRL_REG1, 0x0F)
    if value != 0x8:
        error (lineno(),"Error while setting sleep mode. The function to set the mode succeeded but the bit was not set")
        return False

    # set data rate and bandwidth
    for datarate in gyro.__DRBW.keys ():
        for bandwidth in gyro.__DRBW[datarate].keys():
            (status, message) = gyro.set_drbw (G, datarate, bandwidth)
            if not status:
                error (lineno(),"Error while setting the data rate and the bandwidth. Error message was: " + message)
                return False

            (status, drbw) = gyro.get_drbw (G)
            if not status:
                error (lineno(),"Error while getting the data rate and the bandwidth. Error message was: " + drbw)                
                return False

            if drbw != (datarate, bandwidth):
                error (lineno(),"Error while setting the data rate and the bandwidth. The function to set succeeded but the bits were not set")
                return False

    #############
    # CTRL_REG2 #
    #############

    # set high pass filter mode
    for hpm in gyro.__HPM.keys ():
        (status, message) = gyro.set_hpfm (G, hpm)
        if not status:
            error (lineno(),"Error while setting the high pass filter mode. Error message was: " + message)            
            return False

        (status, mode) = gyro.get_hpfm (G)
        if not status:
            error (lineno(),"Error while getting the high pass filter mode. Error message was: " + mode)
            return False

        if mode != hpm:
            error (lineno(),"Error while setting the high pass filter mode. The function to set succeeded but the bits were not set")
            return False

    # set high pass filter cutoff frequency
    for hpcf in gyro.__HPCF.keys ():
        for datarate in gyro.__HPCF[hpcf].keys ():
            # Set the proper frequency
            for bandwidth in gyro.__DRBW[datarate]:
                (status, message) = gyro.set_drbw (G, datarate, bandwidth)
                if not status:
                    error (lineno(),"Error while setting the data rate and the bandwidth. Error message was: " + message)
                    return False          
                # Just set a tuple datarate and bandwidth value to set the HPCF
                continue
  
        (status, message) = gyro.set_hpcf (G, hpcf)
        if not status:
            error (lineno(),"Error while setting the high pass filter cutoff frequency. Error message was: " + message)            
            return False

        (status, mode) = gyro.get_hpcf (G)
        if not status:
            error (lineno(),"Error while getting the high pass filter cutoff frequency. Error message was: " + mode)
            return False

        if mode != hpcf:
            error (lineno(),"Error while setting the high pass filter cutoff frequency. The function to set succeeded but the bits were not set")
            return False

    #############
    # CTRL_REG4 #
    #############

    gains = {'250dps': 0.00875, '500dps': 0.0175, '2000dps': 0.07}

    # set the scale
    for scale in gyro.__Scales:
        gyro.set_scale (G, scale)
        if not status:
            error (lineno(),"Error while setting the scale. Error message was: " + message)            
            return False

        if G['gain'] != gains[scale]:
            error (lineno(),"Error while setting the scale. The gain value should have been %s and it was %s" % (gains[scale], G['gain']))            
            return False

        (status, scale_aux) = gyro.get_scale (G)
        if not status:
            error (lineno(),"Error while getting the scale. Error message was: " + scale)        
            return False
        
        if scale != scale_aux:
            error (lineno(),"Error while setting the scale. The function to set succeeded but the bits were not set")
            return False
            
    # set the endianness
    (status, message) = gyro.set_endianness (G, 'big_endian')
    if not status:
        error (lineno(),"Error while setting the endianness. Error message was: " + message)                    
        return False

    (status, endianness) = gyro.get_endianness (G)
    if not status:
        error (lineno(),"Error while getting the endianness. Error message was: " + endianness)                    
        return False        

    if endianness != 'big_endian':
        error (lineno(),"Error while setting the endianess. The function to set succeeded but the bits were not set")
        return False        

    (status, message) = gyro.set_endianness (G, 'little_endian')
    if not status:
        error (lineno(),"Error while setting the endianness. Error message was: " + message)                    
        return False

    (status, endianness) = gyro.get_endianness (G)
    if not status:
        error (lineno(),"Error while getting the endianness. Error message was: " + endianness)                    
        return False        

    if endianness != 'little_endian':
        error (lineno(),"Error while setting the endianess. The function to set succeeded but the bits were not set")
        return False        

    gyro.set_endianness (G, 'big_endian')

    # set block data update
    (status, message) = gyro.set_bdu (G, 'continous_update')
    if not status:
        error (lineno(),"Error while setting the block data update. Error message was: " + message)                    
        return False
        
    (status, bdu) = gyro.get_bdu (G)
    if not status:
        error (lineno(),"Error while setting the block data update. Error message was: " + bdu)                    
        return False

    if bdu != 'continous_update':
        error (lineno(),"Error while setting the block data update. The function to set succeeded but the bits were not set")
        return False

    (status, message) = gyro.set_bdu (G, 'not_updated_until_reading')
    if not status:
        error (lineno(),"Error while setting the block data update. Error message was: " + message)                    
        return False
        
    (status, bdu) = gyro.get_bdu (G)
    if not status:
        error (lineno(),"Error while setting the block data update. Error message was: " + bdu)                    
        return False

    if bdu != 'not_updated_until_reading':
        error (lineno(),"Error while setting the block data update. The function to set succeeded but the bits were not set")
        return False

    #############
    # CTRL_REG5 #
    #############

    # set the boot mode
    (status, message) = gyro.set_bootmode (G, 'normal')
    if not status:
        error (lineno(),"Error while setting the boot mode. Error message was: " + message)                    
        return False
        
    (status, bootmode) = gyro.get_bootmode (G)
    if not status:
        error (lineno(),"Error while setting the block data update. Error message was: " + bootmode)                    
        return False

    if bootmode != 'normal':
        error (lineno(),"Error while setting the boot mode. The function to set succeeded but the bits were not set")
        return False

    # It is not posible to set the boot mode to reboot memory content,
    # although it is written a value of 1 it does not change.
    # Commented the code to check this to not obtain an error

    # (status, message) = gyro.set_bootmode (G, 'reboot_memory_content')
    # if not status:
    #     error (lineno(),"Error while setting the boot mode. Error message was: " + message)                    
    #     return False
        
    # (status, bootmode) = gyro.get_bootmode (G)
    # if not status:
    #     error (lineno(),"Error while setting the block data update. Error message was: " + bootmode)                    
    #     return False

    # common_i2c.write(G, gyro.__CTRL_REG5, 0x80, 0x1) 
    # print common_i2c.read (G, gyro.__CTRL_REG5, 0xFF)
    # print bootmode
    # if bootmode != 'reboot_memory_content':
    #     error (lineno(),"Error while setting the boot mode. The function to set succeeded but the bits were not set")
    #     return False
    
    # enable FIFO
    (status, message) = gyro.enable_fifo (G, True)
    if not status:
        error (lineno(),"Error while enabling fifo. Error message was: " + message)                    
        return False
        
    (status, enable) = gyro.isenabled_fifo (G)
    if not status:
        error (lineno(),"Error while enabling fifo. Error message was: " + enable)                    
        return False

    if not enable:
        error (lineno(),"Error while enabling fifo. The function to enable succeeded but the bits were not set")
        return False

    (status, message) = gyro.enable_fifo (G, False)
    if not status:
        error (lineno(),"Error while disabling fifo. Error message was: " + message)                    
        return False
        
    (status, enable) = gyro.isenabled_fifo (G)
    if not status:
        error (lineno(),"Error while disabling fifo. Error message was: " + enable)                    
        return False

    if enable:
        error (lineno(),"Error while disabling fifo. The function to enable succeeded but the bits were not set")
        return False

    # enable high-pass filter
    (status, message) = gyro.enable_hp (G, True)
    if not status:
        error (lineno(),"Error while enabling high-pass filter. Error message was: " + message)                    
        return False
        
    (status, enable) = gyro.isenabled_hp (G)
    if not status:
        error (lineno(),"Error while enabling high-pass filter. Error message was: " + enable)                    
        return False

    if not enable:
        error (lineno(),"Error while enabling high-passh filter. The function to enable succeeded but the bits were not set")
        return False

    (status, message) = gyro.enable_hp (G, False)
    if not status:
        error (lineno(),"Error while disabling high-pass filter. Error message was: " + message)                    
        return False
        
    (status, enable) = gyro.isenabled_hp (G)
    if not status:
        error (lineno(),"Error while disabling high-pass filter. Error message was: " + enable)                    
        return False

    if enable:
        error (lineno(),"Error while disabling high-passh filter. The function to enable succeeded but the bits were not set")
        return False
    
    # set out selection
    (status, message) = gyro.set_outsel (G, 'LPF1')
    if not status:
        error (lineno(),"Error while setting the out selection. Error message was: " + message)                    
        return False
        
    (status, out) = gyro.get_outsel (G)
    if not status:
        error (lineno(),"Error while setting the out selection. Error message was: " + out)                    
        return False

    if out != 'LPF1':
        error (lineno(),"Error while setting the out selection. The function to enable succeeded but the bits were not set")
        return False

    (status, message) = gyro.set_outsel (G, 'HPF')
    if not status:
        error (lineno(),"Error while setting the out selection. Error message was: " + message)                    
        return False
        
    (status, out) = gyro.get_outsel (G)
    if not status:
        error (lineno(),"Error while setting the out selection. Error message was: " + out)                    
        return False

    if out != 'HPF':
        error (lineno(),"Error while setting the out selection. The function to enable succeeded but the bits were not set")
        return False

    (status, message) = gyro.set_outsel (G, 'LPF2')
    if not status:
        error (lineno(),"Error while setting the out selection. Error message was: " + message)                    
        return False
        
    (status, out) = gyro.get_outsel (G)
    if not status:
        error (lineno(),"Error while setting the out selection. Error message was: " + out)                    
        return False

    if out != 'LPF2':
        error (lineno(),"Error while setting the out selection. The function to enable succeeded but the bits were not set")
        return False

    #############
    # REFERENCE #
    #############

    # Set reference
    (status, message) = gyro.set_reference (G, 0xff)
    if not status:
        error (lineno(),"Error while setting the reference. Error message was: " + message)                    
        return False
        
    (status, reference) = gyro.get_reference (G)

    if reference != 0xff:
        error (lineno(),"Error while setting the reference. The function to set succeeded but the bits were not set")
        return False

    #############
    # OUT_TEMP  #
    #############

    # Get the temperature
    if type (gyro.get_temp (G)[1]) != int:
        error (lineno(),"Error while getting the temperature")
        return False

    ##############
    # STATUS_REG #
    ##############

    # Get the data overrun
    for dor in gyro.isdata_overrun (G)[1]:
        if type (dor) != bool:
            error (lineno(),"Error while getting the data overrun")
            return False

    # Get the data available
    for dav in gyro.isdata_available (G)[1]:
        if type (dav) != bool:
            error (lineno(),"Error while getting the data available")
            return False

    #########
    # OUT_X #
    #########

    # Get the x axis angular rate
    if type (gyro.get_x (G)[1]) != float:
        error (lineno(),"Error while getting the x axis angular rate")
        return False

    #########
    # OUT_Y #
    #########

    # Get the y axis angular rate
    if type (gyro.get_y (G)[1]) != float:
        error (lineno(),"Error while getting the y axis angular rate")
        return False

    #########
    # OUT_Z #
    #########

    # Get the z axis angular rate
    if type (gyro.get_z (G)[1]) != float:
        error (lineno(),"Error while getting the z axis angular rate")
        return False

    # Get the three axis angular rates
    (status, xyz) = gyro.get_xyz(G)
    if not status:
        error (lineno(),"Error while getting the three axis angular rates. The error was " + xyz)
        return False
    for axis in xyz:        
        if type (axis) != float:
            error (lineno(),"Error while getting the three axis angular rates")
            return False

    # Reset the values of the registers
    reset_g (G)

    # calibrate
    gyro.calibrate (G)

    # Get the calibrated x axis angular rate
    if type (gyro.get_calx (G)[1]) != float:
        error (lineno(),"Error while getting the calibrated x axis angular rate")
        return False

    # Get the calibrated y axis angular rate
    if type (gyro.get_caly (G)[1]) != float:
        error (lineno(),"Error while getting the calibrated y axis angular rate")
        return False

    # Get the calibrated z axis angular rate
    if type (gyro.get_calz (G)[1]) != float:
        error (lineno(),"Error while getting the calibrated z axis angular rate")
        return False

    #################
    # FIFO_CTRL_REG #
    #################
    
    # Set fifo threshold value
    (status, message) = gyro.set_fifoth (G, 0x1F)
    if not status:
        error (lineno(),"Error while setting the fifo threshold value. Error message was: " + message)                    
        return False

    thr = gyro.get_fifoth (G)[1]
    if thr != 0x1f:
        error (lineno(),"Error while setting the fifo threshold value. The function to set succeeded but the bits were not set")
        return False

    # Set fifo mode
    (status, message) = gyro.set_fifomode (G, 'bypass')
    if not status:
        error (lineno(),"Error while setting the fifo mode to bypass. Error message was: " + message)                    
        return False

    (status, mode) = gyro.get_fifomode (G)
    if not status:
        error (lineno(),"Error while setting the fifo mode to bypass. Error message was: " + out)                    
        return False

    if mode != 'bypass':
        error (lineno(),"Error while setting the fifo mode to bypass. The function to set succeeded but the bits were not set")
        return False

    (status, message) = gyro.set_fifomode (G, 'FIFO')
    if not status:
        error (lineno(),"Error while setting the fifo mode to FIFO. Error message was: " + message)                    
        return False

    (status, mode) = gyro.get_fifomode (G)
    if not status:
        error (lineno(),"Error while setting the fifo mode to FIFO. Error message was: " + out)                    
        return False

    if mode != 'FIFO':
        error (lineno(),"Error while setting the fifo mode to FIFO. The function to set succeeded but the bits were not set")
        return False

    (status, message) = gyro.set_fifomode (G, 'stream')
    if not status:
        error (lineno(),"Error while setting the fifo mode to stream. Error message was: " + message)                    
        return False

    (status, mode) = gyro.get_fifomode (G)
    if not status:
        error (lineno(),"Error while setting the fifo mode to stream. Error message was: " + out)                    
        return False

    if mode != 'stream':
        error (lineno(),"Error while setting the fifo mode to stream. The function to set succeeded but the bits were not set")
        return False

    (status, message) = gyro.set_fifomode (G, 'stream_to_FIFO')
    if not status:
        error (lineno(),"Error while setting the fifo mode to stream_to_FIFO. Error message was: " + message)                    
        return False

    (status, mode) = gyro.get_fifomode (G)
    if not status:
        error (lineno(),"Error while setting the fifo mode to stream_to_FIFO. Error message was: " + out)                    
        return False

    if mode != 'stream_to_FIFO':
        error (lineno(),"Error while setting the fifo mode to stream_to_FIFO. The function to set succeeded but the bits were not set")
        return False

    (status, message) = gyro.set_fifomode (G, 'bypass_to_stream')
    if not status:
        error (lineno(),"Error while setting the fifo mode to bypass_to_stream. Error message was: " + message)                    
        return False

    (status, mode) = gyro.get_fifomode (G)
    if not status:
        error (lineno(),"Error while setting the fifo mode to bypass_to_stream. Error message was: " + out)                    
        return False

    if mode != 'bypass_to_stream':
        error (lineno(),"Error while setting the fifo mode to bypass_to_stream. The function to set succeeded but the bits were not set")
        return False

    gyro.set_fifomode (G, 'bypass')

    #################
    # FIFO_SRC_REG #
    #################

    # Get the fifo level
    if type (gyro.get_fifolevel (G)[1]) != int:
        error (lineno(),"Error while getting the fifo level")
        return False

    # Check if fifo is empty
    if type (gyro.isfifo_empty(G)[1]) != bool:
        error (lineno(),"Error while checking if fifo is empty")
        return False

    # Check if fifo is full
    if type (gyro.isfifo_full(G)[1]) != bool:
        error (lineno(),"Error while checking if fifo is full")
        return False

    (status, wtm) = gyro.get_watermark_status (G)
    if not status:
        error (lineno(),"Error while getting the watermark status. Error message was: " + wtm)                    
        return False

    # Finished test
    del G
  

    return True

#########
# ACCEL #
#########

def create_a ():
    ''' 
    Check the creation of the accelerometer context
    '''
    # Default parameters
    A = accel.Init (scale = '2G')
    if A['error'][0]:
        error (lineno(),"Error while creating the accelerometer. Error message was: " + A['error'][1])
        return False

    if A['addr'] != 0x19:
        error (lineno(),"Error while creating the accelerometer context. The slave address was incorrect. Correct slave address: " + hex (0x19) + ", storaged slave address: " + hex (A['addr']))
        return False

    if A['cal_iter'] != 20:
        error (lineno(),"Error while creating the accelerometer context. The number of iterations for calibration was incorrect. Correct number: " + str (20) + ", storaged number: " + str (A['cal_iter']))
        return False
    
    if A['gain'] != 0.001:
        error (lineno(),"Error while creating the accelerometer context. The value of the gain was incorrect. Correct value: " + str (0.00875) + ", storaged number: " + str (A['gain']))
        return False

    (status, scale) = accel.get_scale (A)
    if not status:
        error (lineno(),"Error while getting the scale. Error message was: " + scale)        
        return False
    
    if scale != '2G':
        error (lineno(),"Error while setting the scale. The function to set succeeded but the bits were not set")
        return False
    
    # Finished test
    del A
    
    return True

def reset_a (A = None):
    '''
    Reset the values of the registers of the accelerometer device
    '''
    (status, message) = accel.reset (A)
    if not status:
        error (lineno(),"Error while reseting the accelerometer device. The error was %s." % message)
        return False
    
    return True
    
def test_03 ():
    '''
    Check the default values of the accelerometer device
    '''

    A = accel.Init ()    
    if A['error'][0]:
        error (lineno(),"Error while creating the accelerometer. Error message was: " + A['error'][1])
        return False

    # Read the value of the CTRL_REG1 register
    (status, value) = common_i2c.read (A, accel.__CTRL_REG1, 0xff)
    if value != 0x27:
        error (lineno(),"The default value of the register CTRL_REG1 is incorrect. THe value was %s." % value)
        return False

    # Read the value of the CTRL_REG2 register
    (status, value) = common_i2c.read (A, accel.__CTRL_REG2, 0xff)
    if value != 0x0:
        error (lineno(),"The default value of the register CTRL_REG2 is incorrect")
        return False

    # Read the value of the CTRL_REG3 register
    (status, value) = common_i2c.read (A, accel.__CTRL_REG3, 0xff)
    if value != 0x0:
        error (lineno(),"The default value of the register CTRL_REG3 is incorrect")
        return False

    # Read the value of the CTRL_REG4 register
    (status, value) = common_i2c.read (A, accel.__CTRL_REG4, 0xff)
    if value != 0x0:
        error (lineno(),"The default value of the register CTRL_REG4 is incorrect")
        return False
    
    # Read the value of the CTRL_REG5 register
    (status, value) = common_i2c.read (A, accel.__CTRL_REG5, 0xff)
    if value != 0x0:
        error (lineno(),"The default value of the register CTRL_REG5 is incorrect")
        return False

    # Read the value of the REFERENCE register
    (status, value) = common_i2c.read (A, accel.__REFERENCE, 0xff)
    if value != 0x0:
        error (lineno(),"The default value of the register REFERENCE is incorrect")
        return False
    
    # Read the value of the FIFO_CTRL_REG register
    (status, value) = common_i2c.read (A, accel.__FIFO_CTRL_REG, 0xff)
    if value != 0x0:
        error (lineno(),"The default value of the register FIFO_CTRL_REG is incorrect")
        return False

    # Finished test
    del A

    return True

def test_04 ():
    '''
    Check that the read only registers of the accelerometer are not writable
    '''

    A = accel.Init ()    
    if A['error'][0]:
        error (lineno(),"Error while creating the accelerometer. Error message was: " + A['error'][1])
        return False

    # Reset the values of the registers
    reset_a (A)

    # Try to write in the STATUS_REG register
    (status, message) = common_i2c.write (A, accel.__STATUS_REG, 0xff, 0xFF)
    if status:
        error (lineno(),"Success on the attempt to write in the unwritable register STATUS_REG")
        return False
    
    # Try to write in the OUT_X_L register
    (status, message) = common_i2c.write (A, accel.__OUT_X_L, 0xff, 0xFF)
    if status:
        error (lineno(),"Success on the attempt to write in the unwritable register OUT_X_L")
        return False

    # Try to write in the OUT_X_H register
    (status, message) = common_i2c.write (A, accel.__OUT_X_H, 0xff, 0xFF)
    if status:
        error (lineno(),"Success on the attempt to write in the unwritable register OUT_X_H")
        return False

    # Try to write in the OUT_Y_L register
    (status, message) = common_i2c.write (A, accel.__OUT_Y_L, 0xff, 0xFF)
    if status:
        error (lineno(),"Success on the attempt to write in the unwritable register OUT_Y_L")
        return False

    # Try to write in the OUT_Y_H register
    (status, message) = common_i2c.write (A, accel.__OUT_Y_H, 0xff, 0xFF)
    if status:
        error (lineno(),"Success on the attempt to write in the unwritable register OUT_Y_H")
        return False

    # Try to write in the OUT_Z_L register
    (status, message) = common_i2c.write (A, accel.__OUT_Z_L, 0xff, 0xFF)
    if status:
        error (lineno(),"Success on the attempt to write in the unwritable register OUT_Z_L")
        return False

    # Try to write in the OUT_Z_H register
    (status, message) = common_i2c.write (A, accel.__OUT_Z_H, 0xff, 0xFF)
    if status:
        error (lineno(),"Success on the attempt to write in the unwritable register OUT_Z_H")
        return False

    # Try to write in the FIFO_SRC_REG register
    (status, message) = common_i2c.write (A, accel.__FIFO_SRC_REG, 0xff, 0xFF)
    if status:
        error (lineno(),"Success on the attempt to write in the unwritable register FIFO_SRC_REG")
        return False

    # Finished test
    del A
    
    return True

def test_05 ():
    '''
    Check the functions of the accelerometer API
    '''

    A = accel.Init ()    
    if A['error'][0]:
        error (lineno(),"Error while creating the accelerometer. Error message was: " + A['error'][1])
        return False

    # Reset the values of the registers
    reset_a (A)

    #############
    # CTRL_REG1 #
    #############

    # Enable X
    (status, message) = accel.enable_x (A, True)
    if not status:
        error (lineno(),"Error while enabling axis X. Error message was: " + message)
        return False

    (status, enabled) = accel.isenabled_x (A)
    if not status or not enabled:
        error (lineno(),"Error while enabling axis X. The function to enable the axis succeeded but the bit was not set")
        return False

    (status, value) = common_i2c.read (A, accel.__CTRL_REG1, 0x01)
    if value != 0x1:
        error (lineno(),"Error while enabling axis X. The function to enable the axis succeeded but the bit was not set")
        return False

    # Disable X
    (status, message) = accel.enable_x (A, False)
    if not status:
        error (lineno(),"Error while disabling axis X. Error message was: " + message)
        return False

    (status, enabled) = accel.isenabled_x (A)
    if not status or enabled:
        error (lineno(),"Error while disabling axis X. The function to disaable the axis succeeded but the bit was not set")
        return False

    (status, value) = common_i2c.read (A, accel.__CTRL_REG1, 0x01)
    if value != 0x0:
        error (lineno(),"Error while disabling axis X. The function to disaable the axis succeeded but the bit was not set")
        return False

    # Enable Y
    (status, message) = accel.enable_y (A, True)
    if not status:
        error (lineno(),"Error while enabling axis Y. Error message was: " + message)
        return False

    (status, enabled) = accel.isenabled_y (A)
    if not status or not enabled:
        error (lineno(),"Error while enabling axis Y. The function to enable the axis succeeded but the bit was not set")
        return False

    (status, value) = common_i2c.read (A, accel.__CTRL_REG1, 0x02)
    if value != 0x1:
        error (lineno(),"Error while enabling axis Y. The function to enable the axis succeeded but the bit was not set")
        return False

    # Disable Y
    (status, message) = accel.enable_y (A, False)
    if not status:
        error (lineno(),"Error while disabling axis Y. Error message was: " + message)
        return False

    (status, enabled) = accel.isenabled_y (A)
    if not status or enabled:
        error (lineno(),"Error while disabling axis Y. The function to disaable the axis succeeded but the bit was not set")
        return False

    (staus, value) = common_i2c.read (A, accel.__CTRL_REG1, 0x02)
    if value != 0x0:
        error (lineno(),"Error while disabling axis Y. The function to disaable the axis succeeded but the bit was not set")
        return False

    # Enable Z
    (status, message) = accel.enable_z (A, True)
    if not status:
        error (lineno(),"Error while enabling axis Z. Error message was: " + message)
        return False

    (status, enabled) = accel.isenabled_z (A)
    if not status or not enabled:
        error (lineno(),"Error while enabling axis Z. The function to enable the axis succeeded but the bit was not set")
        return False

    (status, value) = common_i2c.read (A, accel.__CTRL_REG1, 0x04)
    if value != 0x1:
        error (lineno(),"Error while enabling axis Z. The function to enable the axis succeeded but the bit was not set")
        return False

    # Disable Z
    (status, message) = accel.enable_z (A, False)
    if not status:
        error (lineno(),"Error while disabling axis Z. Error message was: " + message)
        return False

    (status, enabled) = accel.isenabled_z (A)
    if not status or enabled:
        error (lineno(),"Error while disabling axis Z. The function to disaable the axis succeeded but the bit was not set")
        return False

    (status, value) = common_i2c.read (A, accel.__CTRL_REG1, 0x04)
    if value != 0x0:
        error (lineno(),"Error while disabling axis Z. The function to disaable the axis succeeded but the bit was not set")
        return False

    # Set power-down mode
    (status, message) = accel.set_powerdownmode (A)
    if not status:
        error (lineno(),"Error while setting power-down mode. Error message was: " + message)
        return False

    (status, powermode) = accel.get_powermode (A)
    if not status or powermode != 'power_down':
        error (lineno(),"Error while setting power-down mode. The error was: %s" % powermode)
        return False

    (status, value) = common_i2c.read (A, accel.__CTRL_REG1, 0xF0)
    if value != 0x0:
        error (lineno(),"Error while setting power-down mode. The function to set the mode succeeded but the bit was not set")
        return False

    # Set normal mode
    accel.enable_x (A, True)
    (status, message) = accel.poweron (A, 10, 'normal')
    if not status:
        error (lineno(),"Error while setting normal mode. Error message was: " + message)
        return False

    (status, powermode) = accel.get_powermode (A)
    if not status or powermode != 'normal':
        error (lineno(),"Error while setting normal mode. The function to set the mode succeeded but the bit was not set")
        return False

    (status, value) = common_i2c.read (A, accel.__CTRL_REG1, 0x08)
    if value != 0x0:
        error (lineno(),"Error while setting normal mode. The function to set the mode succeeded but the bit was not set")
        return False

    (status, value) = common_i2c.read (A, accel.__CTRL_REG1, 0xF0)
    if value != 0x2:
        error (lineno(),"Error while setting normal mode. The function to set the mode succeeded but the bit was not set")
        return False

    # Set low power mode
    (status, message) = accel.set_powermode (A, 'low_power')
    if not status:
        error (lineno(),"Error while setting low power mode. Error message was: " + message)
        return False

    (status, powermode) = accel.get_powermode (A)
    if not status or powermode != 'low_power':
        error (lineno(),"Error while setting low power mode. The function to set the mode succeeded but the bit was not set")
        return False

    (status, value) = common_i2c.read (A, accel.__CTRL_REG1, 0x08)
    if value != 0x1:
        error (lineno(),"Error while setting low power mode. The function to set the mode succeeded but the bit was not set")
        return False

    # set data rate
    for datarate in accel.__DR.keys ():
        (status, message) = accel.set_dr (A, datarate)
        if not status:
            error (lineno(),"Error while setting the data rate. Error message was: " + message)
            return False

        (status, dr) = accel.get_dr (A)
        if not status:
            error (lineno(),"Error while getting the data rate. Error message was: " + dr)                
            return False

        # There are two different keys that correspond to the same value
        if dr != datarate and accel.__DR[dr] != accel.__DR[datarate]:
            error (lineno(),"Error while setting the data rate. The function to set succeeded but the bits were not set. The write value was %s and the value obtained was %s." % (datarate, dr))
            return False

    #############
    # CTRL_REG2 #
    #############

    # set high pass filter mode
    for hpm in accel.__HPM.keys ():
        (status, message) = accel.set_hpfm (A, hpm)
        if not status:
            error (lineno(),"Error while setting the high pass filter mode. Error message was: " + message)            
            return False

        (status, mode) = accel.get_hpfm (A)
        if not status:
            error (lineno(),"Error while getting the high pass filter mode. Error message was: " + mode)
            return False

        if mode != hpm:
            error (lineno(),"Error while setting the high pass filter mode. The function to set succeeded but the bits were not set")
            return False

    # set high pass filter cutoff frequency
    for hpcf in accel.__HPCF.keys ():
        (status, message) = accel.set_hpcf (A, hpcf)
        if not status:
            error (lineno(),"Error while setting the high pass filter cutoff frequency. Error message was: " + message)            
            return False

        (status, mode) = accel.get_hpcf (A)
        if not status:
            error (lineno(),"Error while getting the high pass filter cutoff frequency. Error message was: " + mode)
            return False

        if mode != hpcf:
            error (lineno(),"Error while setting the high pass filter cutoff frequency. The function to set succeeded but the bits were not set")
            return False

    #############
    # CTRL_REG4 #
    #############

    gains  = {'2G': 0.001, '4G': 0.002, '8G': 0.004, '16G': 0.012}


    # set the scale
    for scale in accel.__Scales:
        accel.set_scale (A, scale)
        if not status:
            error (lineno(),"Error while setting the scale. Error message was: " + message)            
            return False

        if A['gain'] != gains[scale]:
            error (lineno(),"Error while setting the scale. The gain value should have been %s and it was %s" % (gains[scale], A['gain']))            
            return False

        (status, scale_aux) = accel.get_scale (A)
        if not status:
            error (lineno(),"Error while getting the scale. Error message was: " + scale)        
            return False
        
        if scale != scale_aux:
            error (lineno(),"Error while setting the scale. The function to set succeeded but the bits were not set")
            return False
            
    # set the filtered data selection
    (status, message) = accel.set_fds (A, 'filter_bypassed')
    if not status:
        error (lineno(),"Error while setting the filtered data selection. Error message was: " + message)                    
        return False

    (status, fds) = accel.get_fds (A)
    if not status:
        error (lineno(),"Error while getting the filtered data selection. Error message was: " + fds)                    
        return False        

    if fds != 'filter_bypassed':
        error (lineno(),"Error while setting the filtered data selection. The function to set succeeded but the bits were not set")
        return False        

    (status, message) = accel.set_fds (A, 'data_from_internal_filter')
    if not status:
        error (lineno(),"Error while setting the filtered data selection. Error message was: " + message)                    
        return False

    (status, fds) = accel.get_fds (A)
    if not status:
        error (lineno(),"Error while getting the filtered data selection. Error message was: " + fds)                    
        return False        

    if fds != 'data_from_internal_filter':
        error (lineno(),"Error while setting the filtered data selection. The function to set succeeded but the bits were not set")
        return False        

    # Enable high resolution
    (status, message) = accel.enable_hr (A, True)
    if not status:
        error (lineno(),"Error while enabling high resolution. Error message was: " + message)
        return False

    (status, enabled) = accel.isenabled_hr (A)
    if not status or not enabled:
        error (lineno(),"Error while enabling high resolution. The function to enable succeeded but the bit was not set")
        return False

    # Disable high resolution
    (status, message) = accel.enable_hr (A, False)
    if not status:
        error (lineno(),"Error while disabling high resolution. Error message was: " + message)
        return False

    (status, enabled) = accel.isenabled_hr (A)
    if not status or enabled:
        error (lineno(),"Error while disabling high resolution. The function to enable succeeded but the bit was not set")
        return False

    # set the endianness
    (status, message) = accel.set_endianness (A, 'big_endian')
    if not status:
        error (lineno(),"Error while setting the endianness. Error message was: " + message)                    
        return False

    (status, endianness) = accel.get_endianness (A)
    if not status:
        error (lineno(),"Error while getting the endianness. Error message was: " + endianness)                    
        return False        

    if endianness != 'big_endian':
        error (lineno(),"Error while setting the endianess. The function to set succeeded but the bits were not set")
        return False        

    (status, message) = accel.set_endianness (A, 'little_endian')
    if not status:
        error (lineno(),"Error while setting the endianness. Error message was: " + message)                    
        return False

    (status, endianness) = accel.get_endianness (A)
    if not status:
        error (lineno(),"Error while getting the endianness. Error message was: " + endianness)                    
        return False        

    if endianness != 'little_endian':
        error (lineno(),"Error while setting the endianess. The function to set succeeded but the bits were not set")
        return False        

    accel.set_endianness (A, 'big_endian')

    # set block data update
    (status, message) = accel.set_bdu (A, 'continous_update')
    if not status:
        error (lineno(),"Error while setting the block data update. Error message was: " + message)                    
        return False
        
    (status, bdu) = accel.get_bdu (A)
    if not status:
        error (lineno(),"Error while setting the block data update. Error message was: " + bdu)                    
        return False

    if bdu != 'continous_update':
        error (lineno(),"Error while setting the block data update. The function to set succeeded but the bits were not set")
        return False

    (status, message) = accel.set_bdu (A, 'not_updated_until_reading')
    if not status:
        error (lineno(),"Error while setting the block data update. Error message was: " + message)                    
        return False
        
    (status, bdu) = accel.get_bdu (A)
    if not status:
        error (lineno(),"Error while setting the block data update. Error message was: " + bdu)                    
        return False

    if bdu != 'not_updated_until_reading':
        error (lineno(),"Error while setting the block data update. The function to set succeeded but the bits were not set")
        return False

    #############
    # CTRL_REG5 #
    #############

    # set the boot mode
    (status, message) = accel.set_bootmode (A, 'normal')
    if not status:
        error (lineno(),"Error while setting the boot mode. Error message was: " + message)                    
        return False
        
    (status, bootmode) = accel.get_bootmode (A)
    if not status:
        error (lineno(),"Error while setting the block data update. Error message was: " + bootmode)                    
        return False

    if bootmode != 'normal':
        error (lineno(),"Error while setting the boot mode. The function to set succeeded but the bits were not set")
        return False

    # It is not posible to set the boot mode to reboot memory content,
    # although it is written a value of 1 it does not change.
    # Commented the code to check this to not obtain an error
    # (status, message) = accel.set_bootmode (A, 'reboot_memory_content')
    # if not status:
    #     error (lineno(),"Error while setting the boot mode. Error message was: " + message)                    
    #     return False
        
    # (status, bootmode) = accel.get_bootmode (A)
    # if not status:
    #     error (lineno(),"Error while setting the block data update. Error message was: " + bootmode)                    
    #     return False

    # if bootmode != 'reboot_memory_content':
    #     error (lineno(),"Error while setting the boot mode. The function to set succeeded but the bits were not set")
    #     return False

    # enable FIFO
    (status, message) = accel.enable_fifo (A, True)
    if not status:
        error (lineno(),"Error while enabling fifo. Error message was: " + message)                    
        return False
        
    (status, enable) = accel.isenabled_fifo (A)
    if not status:
        error (lineno(),"Error while enabling fifo. Error message was: " + enable)                    
        return False

    if not enable:
        error (lineno(),"Error while enabling fifo. The function to enable succeeded but the bits were not set")
        return False

    (status, message) = accel.enable_fifo (A, False)
    if not status:
        error (lineno(),"Error while disabling fifo. Error message was: " + message)                    
        return False
        
    (status, enable) = accel.isenabled_fifo (A)
    if not status:
        error (lineno(),"Error while disabling fifo. Error message was: " + enable)                    
        return False

    if enable:
        error (lineno(),"Error while disabling fifo. The function to enable succeeded but the bits were not set")
        return False
    
    #############
    # REFERENCE #
    #############

    # Set reference
    (status, message) = accel.set_reference (A, 0xff)
    if not status:
        error (lineno(),"Error while setting the reference. Error message was: " + message)                    
        return False
        
    (status, reference) = accel.get_reference (A)

    if reference != 0xff:
        error (lineno(),"Error while setting the reference. The function to set succeeded but the bits were not set")
        return False

    ##############
    # STATUS_REG #
    ##############

    # Get the data overrun
    for dor in accel.isdata_overrun (A)[1]:
        if type (dor) != bool:
            error (lineno(),"Error while getting the data overrun. The data was %s." % dor)
            return False

    # Get the data available
    for dav in accel.isdata_available (A)[1]:
        if type (dav) != bool:
            error (lineno(),"Error while getting the data available")
            return False

    #########
    # OUT_X #
    #########

    # Get the x axis accelerometer data
    if type (accel.get_x (A)[1]) != float:
        error (lineno(),"Error while getting the x axis accelerometer data")
        return False

    #########
    # OUT_Y #
    #########

    # Get the y axis accelerometer data
    if type (accel.get_y (A)[1]) != float:
        error (lineno(),"Error while getting the y axis accelerometer data")
        return False

    #########
    # OUT_Z #
    #########

    # Get the z axis accelerometer data
    if type (accel.get_z (A)[1]) != float:
        error (lineno(),"Error while getting the z axis accelerometer data")
        return False

    # Get the three axis accelerometer datas
    (status, xyz) = accel.get_xyz(A)
    if not status:
        error (lineno(),"Error while getting the three axis accelerometer datas. The error was " + xyz)
        return False
    for axis in xyz:        
        if type (axis) != float:
            error (lineno(),"Error while getting the three axis accelerometer datas")
            return False

    ##################
    # PITCH AND ROLL #
    ##################        

    # Reset the values of the registers
    reset_a (A)
    accel.poweron (A, 10, 'normal')
    
    # Get the pitch and roll
    (status, data_available) = accel.isdata_available (A)
    if not status:
        error (lineno(), "Error while checking if there is data available. The error was %s" % data_available)
    # end if

    xda, yda, zda = data_available
    while not xda and not yda and not zda:
        (status, data_available) = accel.isdata_available (A)
        if not status:
            error (lineno(), "Error while checking if there is data available. The error was %s" % data_available)
        # end if

        xda, yda, zda = data_available
    # end while

    (status, angles) = accel.get_pitch_roll (A, imu_ex + "acc/cal_matrix.npz", calibrate = True)

    if not status:
        error (lineno(), "Error while getting the roll and pitch angles from the accelerometer. Ther error was %s" % angles)
    for angle in angles:
        if type (angle) != float:
            error (lineno(),"Error while getting the pitch and roll angles. The value had to be of type float, but is of type " + str (type (angle)))
            return False

    #################
    # FIFO_CTRL_REG #
    #################
    
    # Set fifo threshold value
    (status, message) = accel.set_fifoth (A, 0x1F)
    if not status:
        error (lineno(),"Error while setting the fifo threshold value. Error message was: " + message)                    
        return False

    thr = accel.get_fifoth (A)[1]
    if thr != 0x1f:
        error (lineno(),"Error while setting the fifo threshold value. The function to set succeeded but the bits were not set")
        return False

    # Set fifo mode
    (status, message) = accel.set_fifomode (A, 'bypass')
    if not status:
        error (lineno(),"Error while setting the fifo mode to bypass. Error message was: " + message)                    
        return False

    (status, mode) = accel.get_fifomode (A)
    if not status:
        error (lineno(),"Error while setting the fifo mode to bypass. Error message was: " + out)                    
        return False

    if mode != 'bypass':
        error (lineno(),"Error while setting the fifo mode to bypass. The function to set succeeded but the bits were not set")
        return False

    (status, message) = accel.set_fifomode (A, 'FIFO')
    if not status:
        error (lineno(),"Error while setting the fifo mode to FIFO. Error message was: " + message)                    
        return False

    (status, mode) = accel.get_fifomode (A)
    if not status:
        error (lineno(),"Error while setting the fifo mode to FIFO. Error message was: " + out)                    
        return False

    if mode != 'FIFO':
        error (lineno(),"Error while setting the fifo mode to FIFO. The function to set succeeded but the bits were not set")
        return False

    (status, message) = accel.set_fifomode (A, 'stream')
    if not status:
        error (lineno(),"Error while setting the fifo mode to stream. Error message was: " + message)                    
        return False

    (status, mode) = accel.get_fifomode (A)
    if not status:
        error (lineno(),"Error while setting the fifo mode to stream. Error message was: " + out)                    
        return False

    if mode != 'stream':
        error (lineno(),"Error while setting the fifo mode to stream. The function to set succeeded but the bits were not set")
        return False

    (status, message) = accel.set_fifomode (A, 'trigger')
    if not status:
        error (lineno(),"Error while setting the fifo mode to trigger. Error message was: " + message)                    
        return False

    (status, mode) = accel.get_fifomode (A)
    if not status:
        error (lineno(),"Error while setting the fifo mode to trigger. Error message was: " + out)                    
        return False

    if mode != 'trigger':
        error (lineno(),"Error while setting the fifo mode to trigger. The function to set succeeded but the bits were not set")
        return False

    accel.set_fifomode (A, 'bypass')

    #################
    # FIFO_SRC_REG #
    #################

    # Get the fifo level
    if type (accel.get_fifolevel (A)[1]) != int:
        error (lineno(),"Error while getting the fifo level")
        return False

    # Check if fifo is empty
    if type (accel.isfifo_empty(A)[1]) != bool:
        error (lineno(),"Error while checking if fifo is empty")
        return False

    # Check if fifo is full
    if type (accel.isfifo_full(A)[1]) != bool:
        error (lineno(),"Error while checking if fifo is full")
        return False

    (status, wtm) = accel.get_watermark_status (A)
    if not status:
        error (lineno(),"Error while getting the watermark status. Error message was: " + wtm)                    
        return False

    # Finished test
    del A

    return True


#######
# MAG #
#######

def reset_m (M = None):
    '''
    Reset the values of the registers of the magnetometer device
    '''
    (status, message) = mag.reset (M)
    if not status:
        error (lineno(),"Error while reseting the magnetometer device. The error was %s." % message)
        return False
    
    return True

def create_m ():
    ''' 
    Check the creation of the magnetometer context
    '''
    # Default parameters
    M = mag.Init (scale = 1.3)
    if M['error'][0]:
        error (lineno(),"Error while creating the magnetometer. Error message was: " + M['error'][1])
        return False

    if M['addr'] != 0x1e:
        error (lineno(),"Error while creating the magnetometer context. The slave address was incorrect. Correct slave address: " + hex (0x1e) + ", storaged slave address: " + hex (M['addr']))
        return False

    if M['cal_iter'] != 4:
        error (lineno(),"Error while creating the magnetometer context. The number of iterations for calibration was incorrect. Correct number: " + str (4) + ", storaged number: " + str (M['cal_iter']))
        return False
    
    if M['gainxy'] != 1.0/1100:
        error (lineno(),"Error while creating the magnetometer context. The value of the gain was incorrect. Correct value: " + str (1.0/1100) + ", storaged number: " + str (M['gainxy']))
        return False

    if M['gainz'] != 1.0/980:
        error (lineno(),"Error while creating the magnetometer context. The value of the gain was incorrect. Correct value: " + str (1.0/980) + ", storaged number: " + str (M['gainz']))
        return False

    (status, scale) = mag.get_scale (M)
    if not status:
        error (lineno(),"Error while getting the scale. Error message was: " + scale)        
        return False
    
    if scale != 1.3:
        error (lineno(),"Error while setting the scale. The function to set succeeded but the bits were not set")
        return False
    
    # Finished test
    del M
    
    return True

def test_06 ():
    '''
    Check the default values of the magnetometer device
    '''

    M = mag.Init ()    
    if M['error'][0]:
        error (lineno(),"Error while creating the magnetometer. Error message was: " + M['error'][1])
        return False

    # Read the value of the CRA register
    (status, value) = common_i2c.read (M, mag.__CRA, 0xff)
    if value != 0x14:
        error (lineno(),"The default value of the register CRA is incorrect. THe value was %s." % value)
        return False

    # Read the value of the CRB register
    (status, value) = common_i2c.read (M, mag.__CRB, 0xff)
    if value != 0x20:
        error (lineno(),"The default value of the register CRB is incorrect. The value was %s." %value)
        return False

    # Read the value of the MR register
    (status, value) = common_i2c.read (M, mag.__MR, 0xff)
    if value != 0x0:
        error (lineno(),"The default value of the register MR is incorrect")
        return False

    # Finished test
    del M

    return True

def test_07 ():
    '''
    Check that the read only registers of the magnetometer are not writable
    '''

    M = mag.Init ()    
    if M['error'][0]:
        error (lineno(),"Error while creating the magnetometer. Error message was: " + M['error'][1])
        return False

    # Reset the values of the registers
    reset_m (M)

    # Try to write in the STATUS_REG register
    (status, message) = common_i2c.write (M, mag.__SR, 0xff, 0xFF)
    if status:
        error (lineno(),"Success on the attempt to write in the unwritable register STATUS_REG")
        return False
    
    # Try to write in the OUT_X_L register
    (status, message) = common_i2c.write (M, mag.__OUT_X_L, 0xff, 0xFF)
    if status:
        error (lineno(),"Success on the attempt to write in the unwritable register OUT_X_L")
        return False

    # Try to write in the OUT_X_H register
    (status, message) = common_i2c.write (M, mag.__OUT_X_H, 0xff, 0xFF)
    if status:
        error (lineno(),"Success on the attempt to write in the unwritable register OUT_X_H")
        return False

    # Try to write in the OUT_Y_L register
    (status, message) = common_i2c.write (M, mag.__OUT_Y_L, 0xff, 0xFF)
    if status:
        error (lineno(),"Success on the attempt to write in the unwritable register OUT_Y_L")
        return False

    # Try to write in the OUT_Y_H register
    (status, message) = common_i2c.write (M, mag.__OUT_Y_H, 0xff, 0xFF)
    if status:
        error (lineno(),"Success on the attempt to write in the unwritable register OUT_Y_H")
        return False

    # Try to write in the OUT_Z_L register
    (status, message) = common_i2c.write (M, mag.__OUT_Z_L, 0xff, 0xFF)
    if status:
        error (lineno(),"Success on the attempt to write in the unwritable register OUT_Z_L")
        return False

    # Try to write in the OUT_Z_H register
    (status, message) = common_i2c.write (M, mag.__OUT_Z_H, 0xff, 0xFF)
    if status:
        error (lineno(),"Success on the attempt to write in the unwritable register OUT_Z_H")
        return False

    # Try to write in the OUT_TEMP_H register
    (status, message) = common_i2c.write (M, mag.__OUT_TEMP_H, 0xff, 0xFF)
    if status:
        error (lineno(),"Success on the attempt to write in the unwritable register OUT_TEMP_H")
        return False

    # Try to write in the OUT_TEMP_L register
    (status, message) = common_i2c.write (M, mag.__OUT_TEMP_L, 0xff, 0xFF)
    if status:
        error (lineno(),"Success on the attempt to write in the unwritable register OUT_TEMP_L")
        return False

    # Finished test
    del M
    

    return True

def test_08 ():
    '''
    Check the functions of the magnetometer API
    '''

    M = mag.Init ()    
    if M['error'][0]:
        error (lineno(),"Error while creating the magnetometer. Error message was: " + M['error'][1])
        return False

    # Reset the values of the registers
    reset_m (M)

    #######
    # CRA #
    #######

    # Enable temperature
    (status, message) = mag.enable_temp (M, True)
    if not status:
        error (lineno(),"Error while enabling temperature. Error message was: " + message)
        return False

    (status, enabled) = mag.isenabled_temp (M)
    if not status or not enabled:
        error (lineno(),"Error while enabling temperature. The function to enable the temperature succeeded but the bit was not set")
        return False

    (status, value) = common_i2c.read (M, mag.__CRA, 0x80)
    if value != 0x1:
        error (lineno(),"Error while enabling temperature. The function to enable the temperature succeeded but the bit was not set")
        return False

    # Disable temperature
    (status, message) = mag.enable_temp (M, False)
    if not status:
        error (lineno(),"Error while disabling temperature. Error message was: " + message)
        return False

    (status, enabled) = mag.isenabled_temp (M)
    if not status or enabled:
        error (lineno(),"Error while disbling temperature. The function to disable the temperature succeeded but the bit was not set")
        return False

    (status, value) = common_i2c.read (M, mag.__CRA, 0x80)
    if value != 0x0:
        error (lineno(),"Error while disabling temperature. The function to disable the temperature succeeded but the bit was not set")
        return False

    # set data rate
    for datarate in mag.__DR.keys ():
        (status, message) = mag.set_dr (M, datarate)
        if not status:
            error (lineno(),"Error while setting the data rate. Error message was: " + message)
            return False

        (status, dr) = mag.get_dr (M)
        if not status:
            error (lineno(),"Error while getting the data rate. Error message was: " + dr)                
            return False

        if dr != datarate:
            error (lineno(),"Error while setting the data rate. The function to set succeeded but the bits were not set. The write value was %s and the value obtained was %s." % (datarate, dr))
            return False

    #######
    # CRB #
    #######

    # set the scale
    for scale in mag.__Scales:
        mag.set_scale (M, scale)
        if not status:
            error (lineno(),"Error while setting the scale. Error message was: " + message)            
            return False

        if M['gainxy'] != mag.__Gains_XY[scale]:
            error (lineno(),"Error while setting the scale. The gain value should have been %s and it was %s" % (gains[scale], mag.__Gains_XY[scale]))            
            return False

        if M['gainz'] != mag.__Gains_Z[scale]:
            error (lineno(),"Error while setting the scale. The gain value should have been %s and it was %s" % (mag.__Gains_Z[scale], M['gainz']))            
            return False

        (status, scale_aux) = mag.get_scale (M)
        if not status:
            error (lineno(),"Error while getting the scale. Error message was: " + scale)        
            return False
        
        if scale != scale_aux:
            error (lineno(),"Error while setting the scale. The function to set succeeded but the bits were not set")
            return False
            
    ######
    # MR #
    ######
    # Set sleep mode
    (status, message) = mag.set_powermode (M, 'sleep')
    if not status:
        error (lineno(),"Error while setting sleep mode. Error message was: " + message)
        return False

    (status, powermode) = mag.get_powermode (M)
    if not status or powermode != 'sleep':
        error (lineno(),"Error while setting sleep mode. The error was: %s" % powermode)
        return False

    (status, value) = common_i2c.read (M, mag.__MR, 0x3)
    if value != 0x3:
        error (lineno(),"Error while setting sleep mode. The function to set the mode succeeded but the bit was not set")
        return False

    # Set sleep mode
    (status, message) = mag.set_powermode (M, 'sleep_bis')
    if not status:
        error (lineno(),"Error while setting sleep mode. Error message was: " + message)
        return False

    (status, powermode) = mag.get_powermode (M)
    if not status or powermode != 'sleep_bis':
        error (lineno(),"Error while setting sleep mode. The error was: %s" % powermode)
        return False

    (status, value) = common_i2c.read (M, mag.__MR, 0x3)
    if value != 0x2:
        error (lineno(),"Error while setting sleep mode. The function to set the mode succeeded but the bit was not set")
        return False

    # Set continuous-conversion mode
    (status, message) = mag.set_powermode (M, 'continuous')
    if not status:
        error (lineno(),"Error while setting continuous mode. Error message was: " + message)
        return False

    (status, powermode) = mag.get_powermode (M)
    if not status or powermode != 'continuous':
        error (lineno(),"Error while setting continuous mode. The error was: %s" % powermode)
        return False

    (status, value) = common_i2c.read (M, mag.__MR, 0x3)
    if value != 0x0:
        error (lineno(),"Error while setting continuous mode. The function to set the mode succeeded but the bit was not set")
        return False

    #########
    # OUT_X #
    #########

    # Get the x axis magnetic field data
    if type (mag.get_raw_x (M)[1]) != float:
        error (lineno(),"Error while getting the x axis magnetic field data")
        return False

    #########
    # OUT_Y #
    #########

    # Get the y axis magnetic field data
    if type (mag.get_raw_y (M)[1]) != float:
        error (lineno(),"Error while getting the y axis magnetic field data")
        return False

    #########
    # OUT_Z #
    #########

    # Get the z axis magnetic field data
    if type (mag.get_raw_z (M)[1]) != float:
        error (lineno(),"Error while getting the z axis magnetic field data")
        return False

    # Get the three axis magnetic field datas
    (status, xyz) = mag.get_raw_xyz(M)
    if not status:
        error (lineno(),"Error while getting the three axis magnetic field datas. The error was " + xyz)
        return False
    for axis in xyz:        
        if type (axis) != float:
            error (lineno(),"Error while getting the three axis magnetic field datas")
            return False

    ###########
    # HEADING #
    ###########        

    # Reset the values of the registers
    reset_m (M)
    mag.poweron (M, 'continuous')
    A = accel.Init ()
    if A['error'][0]:
        print "Error creating the accelerometer. The error was: %s" % (A['error'][1])
        sys.exit (0)

    
    # Get the heading 
    for value in mag.get_heading (M, A, imu_ex + "mag/max_min.npz", imu_ex + "acc/cal_matrix.npz")[1]:
        if type (value) != float:
            error (lineno(),"Error while getting the heading angle. The value had to be of type float, but is of type " + str (type (value)))
            return False

    ##############
    # STATUS_REG #
    ##############

    # Get the data ready
    if type (mag.isdata_ready (M)[1]) != bool:
        error (lineno(),"Error while getting the data ready. The data was %s." % dor)
        return False

    ############
    # TEMP_OUT #
    ############

    # Get the temperature
    if type (mag.get_temp (M)[1]) != float:
        error (lineno(),"Error while getting the temperature of the magnetometer")
        return False

    # Finished test
    del M

    return True

#######
# ALT #
#######

def reset_p (P = None):
    '''
    Reset the values of the registers of the pressure sensor
    '''
    (status, message) = alt.reset (P)
    if not status:
        error (lineno(),"Error while reseting the pressure sensor. The error was %s." % message)
        return False
    
    return True

def create_p ():
    ''' 
    Check the creation of the altimeter context
    '''
    # Default parameters
    P = alt.Init ()
    if P['error'][0]:
        error (lineno(),"Error while creating the altimeter. Error message was: " + P['error'][1])
        return False

    if P['addr'] != 0x5d:
        error (lineno(),"Error while creating the altimeter context. The slave address was incorrect. Correct slave address: " + hex (0x5d) + ", storaged slave address: " + hex (P['addr']))
        return False

    # Finished test
    del P
    
    return True

def test_09 ():
    '''
    Check the default values of the pressure sensor
    '''

    P = alt.Init ()    
    if P['error'][0]:
        error (lineno(),"Error while creating the altimeter. Error message was: " + P['error'][1])
        return False

    # Read the value of the REF_P register
    (status, xl) = common_i2c.read(P, alt.__REF_P_XL, 0xff)
    (status, l) = common_i2c.read(P, alt.__REF_P_L, 0xff)
    (status, h) = common_i2c.read(P, alt.__REF_P_H, 0xff)

    value = (h << 16) + (l << 8) + xl
    if value != 0x0:
        error (lineno(),"The default value of the register REF_P is incorrect. THe value was %s." % value)
        return False

    # Read the value of the WHO_AM_I register
    (status, value) = common_i2c.read (P, alt.__WHO_AM_I, 0xff)
    if value != 0xbb:
        error (lineno(),"The default value of the register WHO_AM_I is incorrect. The value was %s." %value)
        return False

    # Read the value of the RES_CONF register
    (status, value) = common_i2c.read (P, alt.__RES_CONF, 0xff)
    if value != 0x7a:
        error (lineno(),"The default value of the register RES_CONF is incorrect. The value was %s." %value)
        return False

    # Read the value of the CTRL_REG1 register
    (status, value) = common_i2c.read (P, alt.__CTRL_REG1, 0xff)
    if value != 0xe0:
        error (lineno(),"The default value of the register CTRL_REG1 is incorrect. The value was %s." %value)
        return False

    # Read the value of the CTRL_REG2 register
    (status, value) = common_i2c.read (P, alt.__CTRL_REG2, 0xff)
    if value != 0x0:
        error (lineno(),"The default value of the register CTRL_REG2 is incorrect. The value was %s." %value)
        return False

    # Read the value of the CTRL_REG3 register
    (status, value) = common_i2c.read (P, alt.__CTRL_REG3, 0xff)
    if value != 0x0:
        error (lineno(),"The default value of the register CTRL_REG3 is incorrect. The value was %s." %value)
        return False

    # Read the value of the INT_CFG register
    (status, value) = common_i2c.read (P, alt.__INT_CFG, 0xff)
    if value != 0x0:
        error (lineno(),"The default value of the register INT_CFG is incorrect. The value was %s." %value)
        return False

    # Read the value of the INT_SOURCE register
    (status, value) = common_i2c.read (P, alt.__INT_SOURCE, 0xff)
    if value != 0x0:
        error (lineno(),"The default value of the register INT_SOURCE is incorrect. The value was %s." %value)
        return False

    # Read the value of the INT_SOURCE register
    (status, value) = common_i2c.read (P, alt.__INT_SOURCE, 0xff)
    if value != 0x0:
        error (lineno(),"The default value of the register INT_SOURCE is incorrect. The value was %s." %value)
        return False

    # Finished test
    del P

    return True

def test_10 ():
    '''
    Check that the read only registers of the altimeter are not writable
    '''

    P = alt.Init ()    
    if P['error'][0]:
        error (lineno(),"Error while creating the altimeter. Error message was: " + P['error'][1])
        return False

    # Reset the values of the registers
    reset_p (P)

    # Try to write in the WHO_AM_I register
    (status, message) = common_i2c.write (P, alt.__WHO_AM_I, 0xff, 0xFF)
    if status:
        error (lineno(),"Success on the attempt to write in the unwritable register WHO_AM_I")
        return False
    
    # Try to write in the INT_SOURCE register
    (status, message) = common_i2c.write (P, alt.__INT_SOURCE, 0xff, 0xFF)
    if status:
        error (lineno(),"Success on the attempt to write in the unwritable register INT_SOURCE")
        return False

    # Try to write in the STATUS register
    (status, message) = common_i2c.write (P, alt.__STATUS, 0xff, 0xFF)
    if status:
        error (lineno(),"Success on the attempt to write in the unwritable register STATUS")
        return False

    # Try to write in the PRESS_OUT_XL register
    (status, message) = common_i2c.write (P, alt.__PRESS_OUT_XL, 0xff, 0xFF)
    if status:
        error (lineno(),"Success on the attempt to write in the unwritable register PRESS_OUT_XL")
        return False

    # Try to write in the PRESS_OUT_L register
    (status, message) = common_i2c.write (P, alt.__PRESS_OUT_L, 0xff, 0xFF)
    if status:
        error (lineno(),"Success on the attempt to write in the unwritable register PRESS_OUT_L")
        return False

    # Try to write in the PRESS_OUT_H register
    (status, message) = common_i2c.write (P, alt.__PRESS_OUT_H, 0xff, 0xFF)
    if status:
        error (lineno(),"Success on the attempt to write in the unwritable register PRESS_OUT_H")
        return False

    # Try to write in the TEMP_OUT_L register
    (status, message) = common_i2c.write (P, alt.__TEMP_OUT_L, 0xff, 0xFF)
    if status:
        error (lineno(),"Success on the attempt to write in the unwritable register TEMP_OUT_L")
        return False

    # Try to write in the TEMP_OUT_H register
    (status, message) = common_i2c.write (P, alt.__TEMP_OUT_H, 0xff, 0xFF)
    if status:
        error (lineno(),"Success on the attempt to write in the unwritable register TEMP_OUT_H")
        return False

    # Finished test
    del P
    

    return True

def test_11 ():
    '''
    Check the functions of the altimeter API
    '''

    P = alt.Init ()    
    if P['error'][0]:
        error (lineno(),"Error while creating the altimeter. Error message was: " + P['error'][1])
        return False

    # Reset the values of the registers
    reset_p (P)

    ############
    # RES_CONF #
    ############

    for res in alt.__PRES.keys ():

        (status, message) = alt.set_pres (P, res)
        if not status:
            error (lineno(),"Error while setting the pressure resolution. Error message was: " + message)
            return False

        (status, resolution) = alt.get_pres (P)
        if not status:
            error (lineno(),"Error while getting the pressure resolution. Error message was: " + resolution)                
            return False

        if res != resolution:
            error (lineno(),"Error while setting the pressure resolution. The function to set succeeded but the bits were not set. The write value was %s and the value obtained was %s." % (res, resolution))
            return False
    # Case 25Hz/25Hz and pressure resolution = 0.02
    alt.set_dr (P, 25, 25)
    (status, message) = alt.set_pres (P, 0.02)
    if not status:
        error (lineno(),"Error while setting the pressure resolution. Error message was: " + message)
        return False

    time.sleep (0.01)

    (status, resolution) = alt.get_pres (P)
    if not status:
        error (lineno(),"Error while getting the pressure resolution. Error message was: " + resolution)                
        return False

    if 0.02 != resolution:
        error (lineno(),"Error while setting the pressure resolution. The function to set succeeded but the bits were not set. The write value was %s and the value obtained was %s." % (resolution, res))
        return False

    # Restoring values
    alt.set_dr (P, 15, 15)
    alt.set_pres (P, 0.02)

    ############
    # WHO_AM_I #
    ############

    (status, value) = alt.get_deviceid (P)
    if value != 0xbb:
        error (lineno(),"The value of the register WHO_AM_I is incorrect")
        return False

    #############
    # CTRL_REG1 #
    #############
    # Power down the altimeter
    (status, message) = alt.powerdown (P)
    if not status:
        error (lineno(),"Error while powering down the altimeter. Error message was: " + message)
        return False

    (status, powermode) = alt.get_powermode (P)
    if not status or powermode != 'power_down':
        error (lineno(),"Error while powering down the altimeter. The function to set the mode succeeded but the bit was not set")
        return False

    (status, value) = common_i2c.read (P, alt.__CTRL_REG1, 0x80)
    if value != 0x0:
        error (lineno(),"Error while powering down the altimeter. The function to set the mode succeeded but the bit was not set")
        return False
    
    # Power on the altimeter
    (status, message) = alt.poweron (P)
    if not status:
        error (lineno(),"Error while powering on the altimeter. Error message was: " + message)
        return False

    (status, powermode) = alt.get_powermode (P)
    if not status or powermode != 'active':
        error (lineno(),"Error while powering on the altimeter. The function to set the mode succeeded but the bit was not set")
        return False

    (status, value) = common_i2c.read (P, alt.__CTRL_REG1, 0x80)
    if value != 0x1:
        error (lineno(),"Error while powering on the altimeter. The function to set the mode succeeded but the bit was not set")
        return False

    # set data rate
    for p_dr in alt.__DR.keys ():
        for t_dr in alt.__DR[p_dr].keys():
            (status, message) = alt.set_dr (P, p_dr, t_dr)
            if not status:
                error (lineno(),"Error while setting the pressure and the temperature data rate. Error message was: " + message)
                return False

            (status, values) = alt.get_dr (P)
            if not status:
                error (lineno(),"Error while getting the pressure and the temperature data rate. Error message was: " + values)                
                return False

            if values != (p_dr, t_dr):
                error (lineno(),"Error while setting the pressure and the temperature data rate. The function to set succeeded but the bits were not set")
                return False

    # set block data update
    (status, message) = alt.set_bdu (P, 'not_updated_until_reading')
    if not status:
        error (lineno(),"Error while setting the block data update. Error message was: " + message)                    
        return False
        
    (status, bdu) = alt.get_bdu (P)
    if not status:
        error (lineno(),"Error while setting the block data update. Error message was: " + bdu)                    
        return False

    if bdu != 'not_updated_until_reading':
        error (lineno(),"Error while setting the block data update. The function to set succeeded but the bits were not set")
        return False

    (status, message) = alt.set_bdu (P, 'continous_update')
    if not status:
        error (lineno(),"Error while setting the block data update. Error message was: " + message)                    
        return False
        
    (status, bdu) = alt.get_bdu (P)
    if not status:
        error (lineno(),"Error while setting the block data update. Error message was: " + bdu)                    
        return False

    if bdu != 'continous_update':
        error (lineno(),"Error while setting the block data update. The function to set succeeded but the bits were not set")
        return False

    # Enable delta pressure
    (status, message) = alt.enable_delta (P, True)
    if not status:
        error (lineno(),"Error while enabling delta pressure. Error message was: " + message)
        return False

    (status, enabled) = alt.isenabled_delta (P)
    if not status or not enabled:
        error (lineno(),"Error while enabling delta pressure. The function to enable the delta pressure succeeded but the bit was not set")
        return False

    (status, value) = common_i2c.read (P, alt.__CTRL_REG1, 0x2)
    if value != 0x1:
        error (lineno(),"Error while enabling delta pressure. The function to enable the delta pressure succeeded but the bit was not set. Line: %s" %lineno())
        return False

    # Disable delta pressure
    (status, message) = alt.enable_delta (P, False)
    if not status:
        error (lineno(),"Error while disabling delta pressure. Error message was: " + message)
        return False

    (status, enabled) = alt.isenabled_delta (P)
    if not status or enabled:
        error (lineno(),"Error while disabling delta pressure. The function to disable the delta pressure succeeded but the bit was not set")
        return False

    (status, value) = common_i2c.read (P, alt.__CTRL_REG1, 0x2)
    if value != 0x0:
        error (lineno(),"Error while disabling delta pressure. The function to disable the delta pressure succeeded but the bit was not set")
        return False

    #############
    # CTRL_REG2 #
    #############

    # set the boot mode
    (status, message) = alt.set_bootmode (P, 'normal')
    if not status:
        error (lineno(),"Error while setting the boot mode. Error message was: " + message)                    
        return False
        
    (status, bootmode) = alt.get_bootmode (P)
    if not status:
        error (lineno(),"Error while setting the block data update. Error message was: " + bootmode)                    
        return False

    if bootmode != 'normal':
        error (lineno(),"Error while setting the boot mode. The function to set succeeded but the bits were not set")
        return False

    # get pressure
    (status, p) = alt.get_p (P)
    if type (p) != float:
        error (lineno(),"Error while getting the pressure data. The value is not of type float is of type : %s" + type (p))                    
        return False

    # get reference
    (status, ref) = alt.get_ref (P)
    if not status:
        error (lineno(),"Error while getting the reference. Error message was: " + message)                    
        return False

    if ref != 0x0:
        error (lineno(),"Error while getting the reference. The otained reference value is incorrect")                    
        return False

    # set reference
    (status, message) = alt.set_ref (P)
    if not status:
        error (lineno(),"Error while setting the reference. Error message was: " + message)                    
        return False

    time.sleep (0.25)

    (status, r) = alt.get_ref (P)
    
    if p == 0:
        error (lineno(),"Error while setting the reference. The function to set succed but the set reference is incorrect")                    
        return False

    # start conversion
    (status, values) = alt.start_conversion (P)
    if not status:
        error (lineno(),"Error while starting one shot conversion. The error was: %s" % values)
        return False
    
    p,t = values
    if type (p) != float and type (t) != float:
        error (lineno(),"Error while starting one shot conversion. The obtained values are incorrect")
        return False        

    ##############
    # STATUS REG #
    ##############

    # Get the data overrun
    for dor in alt.isdata_overrun (P)[1]:
        if type (dor) != bool:
            error (lineno(),"Error while getting the data overrun")
            return False

    # Get the data available
    for dav in alt.isdata_available (P)[1]:
        if type (dav) != bool:
            error (lineno(),"Error while getting the data available")
            return False

    #############
    # PRESS_OUT #
    #############
    # Get the pressure data
    if type (alt.get_p (P)[1]) != float:
        error (lineno(),"Error while getting the pressure data")
        return False

    ############
    # TEMP_OUT #
    ############
    # Get the temperature data
    if type (alt.get_t (P)[1]) != float:
        error (lineno(),"Error while getting the temperature data")
        return False

    alt.reset ()
    P = alt.Init ()
    
    # Get the altitude data
    for data in alt.get_alt (P)[1]:
        if type (data) != float:
            error (lineno(),"Error while getting the altitude data")
            return False

    # Finished test
    del P

    return True

###########################
# infrastructure support #
###########################

def lineno():
    """Returns the current line number in our program."""
    return inspect.currentframe().f_back.f_lineno

def info (msg):
    print "[ INFO  ] : " + msg

def error (line, msg):
    if line != None:
        print "[ ERROR ] line %s: %s"  % (line, msg)
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
            error (None, "detected test failure at: " + test[1])
            return False

        # next test
        test_count += 1
    
    ok ("All tests ok!")
    return True

# declare list of tests available
tests = [
   (reset_g,   "Reset the values of the registers of the gyro device"),
   (create_g,  "Check the creation of the gyro context"),
   (test_00,   "Check the defaults values of each register of the gyro device"),
   (test_01,   "Check that the read only registers of the gyro device are not writable"),
   (test_02,   "Check the functions of the gyro API"),
   (reset_g,   "Reset the values of the registers of the gyro device"),
   (reset_a,   "Reset the values of the registers of the accelerometer device"),
   (create_a,  "Check the creation of the accelerometer context"),
   (test_03,   "Check the defaults values of each register of the accelerometer device"),
   (test_04,   "Check that the read only registers of the accelerometer device are not writable"),
   (test_05,   "Check the functions of the accelerometer API"),
   (reset_a,   "Reset the values of the registers of the accelerometer device"),
   (reset_m,   "Reset the values of the registers of the magnetometer device"),
   (create_m,  "Check the creation of the magnetometer context"),
   (test_06,   "Check the defaults values of each register of the magnetometer device"),
   (test_07,   "Check that the read only registers of the magnetometer device are not writable"),
   (test_08,   "Check the functions of the magnetometer API"),
   (reset_m,   "Reset the values of the registers of the magnetometer device"),
   (reset_p,   "Reset the values of the registers of the pressure sensor"),
   (create_p,  "Check the creation of the magnetometer context"),
   (test_09,   "Check the defaults values of each register of the pressure sensor"),
   (test_10,   "Check that the read only registers of the pressure sensor are not writable"),
   (test_11,   "Check the functions of the altimeter API"),
   (reset_p,   "Reset the values of the registers of the pressure sensor")

]

if __name__ == '__main__':

    # call to run all tests
    run_all_tests ()



# TODO:
# Check the consistency of all registers when one of them is modified
# Check errors on the parameters of the API
# Check parameters in Init function
