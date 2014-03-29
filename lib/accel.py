# -*- coding: utf-8 -*-

# Library for the Raspberry Pi that interfaces with LSM303DLHC
# accelerometer on Polulu boards

from smbus import SMBus
from common_i2c import *
import bitOps
import numpy
import time
import os

DEV_SLAVE_ADDR = 0x19         # Default device slave address
DEV_BUS_ID     = 1            # Default device bus id
CALIBRATION_ITERATIONS = 20   # Default number for calibration operations

# Sign definition fot the accelerometer in the AltIMU-10
# Invert the X axis to make the sensing values correspond to the device body axes.   
SIGN_DEFINITION = {'x': -1.0, 'y': 1.0, 'z': 1.0}

def Init (bus = None, 
          slaveAddr = DEV_SLAVE_ADDR, 
          cal_iter = CALIBRATION_ITERATIONS,
          scale = None, dr = 10, mode = 'normal',
          sign_def = SIGN_DEFINITION):
    '''
    bus:       indicates the bus associated to the i2c device /dev/i2cX
    slaveAddr: indicates the address of the slave device
    cal_iter:  indicates the number of iterations to calibrate each axis
    scale:     indicates the measurement range
    dr:        indicates the data rate
    mode:      indicates the mode to power the device
    sign_def:  indicates the sign definition to orientate the device correctly
    '''
    A = {}
    A['bus'] = bus
    if bus == None:
        try:
            A['bus'] = SMBus(DEV_BUS_ID)
        except IOError as e:
            A['error'] = (True, "Unable to create the accelerometer. The error was: I/O error (%s): %s" %(e.errno, e.strerror))
            return A
    A['addr'] = DEV_SLAVE_ADDR
    A['cal_iter'] = cal_iter
    A['sign_def'] = sign_def
    A['error'] = (False, None)

    # Set the gain for the out values
    if scale == None:
        (status, scale) = get_scale(A)
        if not status:
            A['error'] = (True, "Unable to create the accelerometer. The error was %s " %scale)
            return A
    elif not scale in __Scales.keys():
        A['error'] = (True, "Unable to create the accelerometer. The scale %s is not in the range." % scale)
        return A
    else:
        set_scale (A, scale)

                
    A['gain'] = __Gains [scale]    

    # Power on the device
    (status, error) = poweron (A, dr, mode)
    if not status:
        A['error'] = (True, "Unable to power on the accelerometer. The error was: %s" % error)
        return A

    return A


#############
# REGISTERS #
#############

__CTRL_REG1       = Register ('CTRL_REG1', 0x20, mode = 'rw')
__CTRL_REG2       = Register ('CTRL_REG2', 0x21, mode = 'rw')
__CTRL_REG3       = Register ('CTRL_REG3', 0x22, mode = 'rw')
__CTRL_REG4       = Register ('CTRL_REG4', 0x23, mode = 'rw')
__CTRL_REG5       = Register ('CTRL_REG5', 0x24, mode = 'rw')
__CTRL_REG6       = Register ('CTRL_REG6', 0x25, mode = 'rw')
__REFERENCE       = Register ('REFERENCE', 0x26, mode = 'rw')
__STATUS_REG      = Register ('STATUS_REG', 0x27, mode = 'r')
__OUT_X_L         = Register ('OUT_X_L', 0x28, mode = 'r')
__OUT_X_H         = Register ('OUT_X_H', 0x29, mode = 'r')
__OUT_Y_L         = Register ('OUT_Y_L', 0x2a, mode = 'r')
__OUT_Y_H         = Register ('OUT_Y_H', 0x2b, mode = 'r')
__OUT_Z_L         = Register ('OUT_Z_L', 0x2c, mode = 'r')
__OUT_Z_H         = Register ('OUT_Z_H', 0x2d, mode = 'r')
__FIFO_CTRL_REG   = Register ('FIFO_CTRL_REG', 0x2e, mode = 'rw')
__FIFO_SRC_REG    = Register ('FIFO_SRC_REG', 0x2f, mode = 'r')
__INT1_CFG        = Register ('INT1_CFG', 0x30, mode = 'rw')

# Is not posible to manage the device through interrupts (Only here for completeness)
__INT1_SOURCE     = Register ('INT1_SOURCE', 0x31, mode = 'r')
__INT1_THS        = Register ('INT1_THS', 0x32, mode = 'rw')
__INT1_DURATION   = Register ('INT1_DURATION', 0x33, mode = 'rw')
__INT2_CFG        = Register ('INT2_CFG', 0x34, mode = 'rw')
__INT2_SOURCE     = Register ('INT2_SOURCE', 0x35, mode = 'r')
__INT2_THS        = Register ('INT2_THS', 0x36, mode = 'rw') 
__INT2_DURATION   = Register ('INT2_DURATION', 0x37, mode = 'rw')

# Click recognition not used (Only here for completeness)
__CLICK_CFG       = Register ('CLICK_CFG', 0x38, mode = 'rw')
__CLICK_SRC       = Register ('CLICK_SRC', 0x39, mode = 'rw')
__CLICK_THS       = Register ('CLICK_THS', 0x3a, mode = 'rw')
__TIME_LIMIT      = Register ('TIME_LIMIT', 0x3b, mode = 'rw')
__TIME_LATENCY    = Register ('TIME_LATENCY', 0x3c, mode = 'rw')
__TIME_WINDOW     = Register ('TIME_WINDOW', 0x3d, mode = 'rw')

#################################
# MASKS TO MODIFY THE REGISTERS #
#################################

# CTRL_REG1
__MASK_Xen        = 0x01      # X enable
__MASK_Yen        = 0x02      # Y enable
__MASK_Zen        = 0x04      # Z enable
__MASK_LPen       = 0x08      # Low-power mode enable
__MASK_DR        = 0xF0      # Data rate selection

# CTRL_REG2
__MASK_FDS        = 0x08      # Filtered data selection
__MASK_HPCF        = 0x30      # High pass filter cut-off frequency selection
__MASK_HPM        = 0xC0      # High pass filter mode selection

# CTRL_REG4
__MASK_HR         = 0x04      # High resolution output mode
__MASK_FS         = 0x30      # Full scale selection
__MASK_BLE        = 0x40      # Big/Little endian data selection
__MASK_BDU        = 0x80      # Block data update

# CTRL_REG5
__MASK_FIFO_EN    = 0x40      # FIFO enable
__MASK_BOOT       = 0x80      # Reboot memory content

# STATUS_REG
__MASK_ST_H       = 0xF0      # high part of the status register
__MASK_ST_L       = 0x0F       # low part of the status register
__MASK_XDA        = 0x01      # X axis new data available
__MASK_YDA        = 0x02      # Y axis new data available
__MASK_ZDA        = 0x04      # Z axis new data available
__MASK_ZYXDA      = 0x08      # X, Y and Z axis new data available
__MASK_XOR        = 0x10      # X axis data overrun
__MASK_YOR        = 0x20      # Y axis data overrun
__MASK_ZOR        = 0x40      # Z axis data overrun
__MASK_ZYXOR      = 0x80      # X, Y and Z axis data overrun

# FIFO_CTRL_REG
__MASK_THR        = 0x1F      # FIFO threshold value
__MASK_TR         = 0x20      # Trigger selection
__MASK_FM         = 0xC0      # FIFO mode selection

# FIFO_SRC_REG
__MASK_FSS        = 0x1F      # FIFO stored data level
__MASK_EMPTY      = 0x20      # FIFO empty bit
__MASK_OVRN       = 0x40      # Overrun bit status
__MASK_WTM        = 0x80      # Watermark status

##########################
# VALUE REGISTER MAPPING #
##########################

# Output data rate selection 
__DR = { 
    'power_down': 0x0,
    1: 0x1, 10: 0x2, 25: 0x3, 50: 0x4, 100: 0x5, 200: 0x6, 400:0x7,
    1620: 0x8, 1344: 0x9, 5376: 0x9
    }

__PowerMode = {'normal': 0x0, 'low_power': 0x1}

# Enable/disable
__Enable = {True: 0x1, False: 0x0}

# High pass filter mode selection
__HPM = {
    'normal_with_reset':0x0,
    'reference_signal_for_filtering':0x1,
    'normal':0x2,
    'autoreset_on_interrupt':0x3
    }

# High pass filter cut-off frequency selection
__HPCF = {8: 0x0, 16: 0x1, 32: 0x2, 64: 0x3}

# Filtered data selection
__FDS = {'filter_bypassed': 0x1, 'data_from_internal_filter': 0x0}

# Block data update mode
__BlockDataUpdate = {'continous_update': 0x00, 'not_updated_until_reading': 0x01}

# Big/little endian data selection
__Endianness = {'big_endian': 0x00, 'little_endian': 0x01}

# Full-scale data selection
__Scales = {'2G': 0x00, '4G': 0x01, '8G': 0x02, '16G': 0x03}
__Gains  = {'2G': 0.001, '4G': 0.002, '8G': 0.004, '16G': 0.012}

# Reboot memory content mode
__BootMode = {'normal': 0x00, 'reboot_memory_content': 0x01}

# FIFO mode selection
__FIFOMode = {
    'bypass': 0x00,
    'FIFO': 0x01,
    'stream': 0x02,
    'trigger': 0x03,
    }

# Watermark status
__WTM = {'lower': 0x0, 'equal_greater': 0x1}

#############
# FUNCTIONS #
#############

###################
# Print functions #
###################
        
def print_ctrl_reg1 (A):
    print "CTRL_REG1:"
    print "\tXen: " + str (isenabled_x(A)[1])
    print "\tYen: " + str (isenabled_y(A)[1])
    print "\tZen: " + str (isenabled_z(A)[1])
    print "\tLPen: " + str (get_powermode(A)[1])
    print "\tDR: " + str (get_dr(A)[1])

    return
    
def print_ctrl_reg2 (A):
    print "CTRL_REG2:"
    print "\tHPM: " + str (get_hpfm(A)[1])
    print "\tHPCF: " + str (get_hpcf(A)[1])
    print "\tFDS: " + str (get_fds(A)[1])

    return

def print_ctrl_reg4 (A):
    print "CTRL_REG4:"
    print "\tHR: " + str (isenabled_hr(A)[1])
    print "\tFS1-FS0: " + str (get_scale(A)[1])
    print "\tBLE: " + str (get_endianness(A)[1])
    print "\tBDU: " + str (get_bdu(A)[1])

    return

def print_ctrl_reg5 (A):
    print "CTRL_REG5:"
    print "\tFIFO_EN: " + str (isenabled_fifo(A)[1])
    print "\tBOOT: " + str (get_bootmode(A)[1])

    return

def print_reference (A):
    print "REFERENCE: " + str (get_reference(A)[1])

    return

def print_fifo_ctrl (A):
    print "FIFO_CTRL:"
    print "\tFTH4:0: " + str (get_fifoth(A)[1])
    print "\tFM1-FM0: " + str (get_fifomode(A)[1])
    
    return
    
def print_configuration(A):
    ''' Print the configuration '''
    print_ctrl_reg1 (A)
    print_ctrl_reg2 (A)
    print_ctrl_reg4 (A)
    print_ctrl_reg5 (A)
    print_reference (A)
    print_fifo_ctrl (A)
    
    return


def enable_x(A, enabled):
    ''' Enable x Axis '''
    (status, message) = write(A, __CTRL_REG1, __MASK_Xen, __Enable[enabled])
    if not status:
        return (False, 'Unable to enable or disable x axis for the accelerometer device. The error was %s' % message)
    return (True, None)
    
def isenabled_x(A):
    ''' Check if x axis is enabled '''
    (status, enabled) = read_match(A, __CTRL_REG1, __MASK_Xen, __Enable)
    if not status:
        return (False, 'Unable to check if x axis is enabled for the accelerometer device. The error was %s' % enabled)
    return (True, enabled)

def enable_y(A, enabled):
    ''' Enable y Axis '''
    (status, message) = write(A, __CTRL_REG1, __MASK_Yen, __Enable[enabled])
    if not status:
        return (False, 'Unable to enable or disable y axis for the accelerometer device. The error was %s' % message)
    return (True, None)

def isenabled_y(A):
    ''' Check if y axis is enabled '''
    (status, enabled) = read_match(A, __CTRL_REG1, __MASK_Yen, __Enable)
    if not status:
        return (False, 'Unable to check if y axis is enabled for the accelerometer device. The error was %s' % enabled)
    return (True, enabled)

def enable_z(A, enabled):
    ''' Enable z Axis '''
    (status, message) = write(A, __CTRL_REG1, __MASK_Zen, __Enable[enabled])
    if not status:
        return (False, 'Unable to enable or disable z axis for the accelerometer device. The error was %s' % message)
    return (True, None)

def isenabled_z(A):
    ''' Check if z axis is enabled '''
    (status, enabled) = read_match(A, __CTRL_REG1, __MASK_Zen, __Enable)
    if not status:
        return (False, 'Unable to check if z axis is enabled for the accelerometer device. The error was %s' % enabled)
    return (True, enabled)

def set_powerdownmode(A):
    ''' Set power-down mode '''
    (status, message) = write(A, __CTRL_REG1, __MASK_DR, __DR['power_down'])
    if not status:
        return (False, 'Unable to power down the accelerometer device. The error was %s' % message)

    return (True, None)

def set_powermode (A, mode):
    ''' Set power mode '''
    if mode not in __PowerMode.keys():
        return (False, 'Mode %s not in range of allowed power mode values for the accelerometer device' % mode)
    
    (status, message) = write (A, __CTRL_REG1, __MASK_LPen, __PowerMode[mode])
    if not status:
        return (False, 'Unable to set the power mode on the accelerometer device. The error was %s' % message)

    return (True, None)

def poweron(A, dr, mode):
    ''' Power on the device '''
    (status, message) = set_dr (A, dr)
    if not status:
        return (False, 'Unable to power on the accelerometer device. The error was %s' % message)

    return set_powermode (A, mode)

def get_powermode(A):
    ''' Get power mode '''
    # Check power-down mode
    (status, powermode) = read_match(A, __CTRL_REG1, __MASK_DR, __DR)

    # Check power-down or normal mode
    if powermode != 'power_down':
        (status, powermode) = read_match(A, __CTRL_REG1, __MASK_LPen, __PowerMode)            
        
    if not status:
        return (False, 'Unable to get the power mode on the accelerometer device. The error was %s' % powermode)

    return (True, powermode)

def set_dr(A, datarate):
    ''' Set data rate '''
    if datarate not in __DR.keys():
        return (False, 'Data rate %s not in range of data rate values for the accelerometer device.' % datarate)

    bits = __DR[datarate]
    (status, message) = write(A, __CTRL_REG1, __MASK_DR, bits)
    if not status:
        return (False, 'Unable to set the data rate on the accelerometer device. The error was %s' % message)        
    return (True, None)

def get_dr(A):
    ''' Get data rate '''
    (status, current) = read(A, __CTRL_REG1, __MASK_DR)
    if not status:
        return (False, "Unable to get the configurated data rate for the accelerometer device. The error was: %s." % current)
    
    # Convert to the proper value in Hz or 'power_down' value
    for dr in __DR.keys():
        if __DR[dr] == current:
            return (True, dr)

    # never reached
    return (False, "Unable to get the configurated data rate for the accelerometer device")

def set_hpfm(A, mode):
    ''' Set the high-pass filter mode '''
    if mode not in __HPM.keys():
        return (False, 'High pass filter mode %s is not in range of high pass frequency modes for the accelerometer device.' % mode)

    # Obtain the value and write it
    bits = __HPM[mode]
    (status, message) = write(A, __CTRL_REG2, __MASK_HPM, bits)
    if not status:
        return (False, 'Unable to set the high pass filter mode on the accelerometer device. The error was %s' % message)        
    return (True, None)

def get_hpfm(A):
    ''' Get the high-pass filter mode '''
    (status, current) = read(A, __CTRL_REG2, __MASK_HPM)
    if not status:
        return (False, "Unable to get the configurated high pass filter mode on the accelerometer device. The error was: %s." % current)

    for mode in __HPM.keys():
        if __HPM[mode] == current:
            return (True, mode)

    # never reached    
    return (False, "Unable to get the configurated high pass filter mode on the accelerometer device")

def set_hpcf(A, freq):
    ''' Set the high-pass filter cutoff frequency '''
    # Check frequency value
    if freq not in __HPCF.keys():
        return (False, "Frequency: %s is not in range of high pass frequency cut off values for the accelerometer device." % freq)

    # Obtain the value and write it
    bits = __HPCF[freq]
    (status, message) = write(A, __CTRL_REG2, __MASK_HPCF, bits)
    if not status:
        return (status, "Unable to set high-pass filter cutoff frequency on the accelerometer device. The error was %s" % message)

    return (True, None)

def get_hpcf(A):
    ''' Get the high-pass filter cutoff frequency '''
    (status, current) = read(A, __CTRL_REG2, __MASK_HPCF)
    if not status:
        return (False, "Unable to get the high pass filter cutoff frequency on the accelerometer device. The error was: %s" % current)
    
    # Convert to the proper value in Hz
    for freq in __HPCF.keys():
        if __HPCF[freq] == current:
            return (True, freq)

    # never reached    
    return (False, "Unable to get the high pass filter cutoff frequency on the accelerometer device")

def set_fds (A, selection):
    ''' Set filtered data selection '''
    if selection not in __FDS.keys():
        return (False, 'Filtered data selection %s not in range of filtered data selection values for the accelerometer device.' % selection)
    bits = __FDS[selection]
    (status, message) = write(A, __CTRL_REG2, __MASK_FDS, bits)
    if not status:
        return (False, 'Unable to set the filtered data selection on the accelerometer device. The error was %s' % message)        
    return (True, None)

def get_fds (A):
    ''' Get filtered data selection '''
    (status, fds) = read_match(A, __CTRL_REG2, __MASK_FDS, __FDS)
    if not status:
        return (False, 'Unable to get the filtered data selection on the accelerometer device. The error was %s' % fds)        

    return (True, fds)

def enable_hr (A, enabled):
    ''' Enable high resolution '''
    (status, message) = write(A, __CTRL_REG4, __MASK_HR, __Enable[enabled])
    if not status:
        return (False, 'Unable to enable or disable high resolution for the accelerometer device. The error was %s' % message)
    return (True, None)
        
def isenabled_hr (A):
    ''' Check if high resolution is enabled '''
    (status, enabled) = read_match(A, __CTRL_REG4, __MASK_HR, __Enable)
    if not status:
        return (False, 'Unable to check if high resolution is enabled on the accelerometer device. The error was %s' % enabled)

    return (True, enabled)

def set_scale(A, scale):
    ''' Set full scale '''
    if not scale in __Scales.keys():
        return (False, 'Unable to set the scale %s for the accelerometer device, the value is not in the range.' % scale)

    (status, message) = write(A, __CTRL_REG4, __MASK_FS, __Scales[scale]) 

    if not status:
        return (False, "Unable to set the scale %s for the accelerometer device. The error was %s" % (scale, message))

    # Set the gain for the out values
    A['gain'] = __Gains [scale]    
    
    return (True, None)

def get_scale(A):
    ''' Get full scale '''
    (status, scale) = read_match(A, __CTRL_REG4, __MASK_FS, __Scales)
    if not status:
        return (False, "Unable to get the scale of the accelerometer device. The error was %s." % scale)

    return (True, scale)
    
def set_endianness(A, value):
    ''' Set endianness '''
    (status, message) = write(A, __CTRL_REG4, __MASK_BLE, __Endianness[value]) 
    if not status:
        return (False, "Unable to set the endianness of the accelerometer device. The error was %s." % message)

    return (True, None)
    
def get_endianness(A):
    ''' Get endianness '''
    (status, endianness) = read_match(A, __CTRL_REG4, __MASK_BLE, __Endianness)
    if not status:
        return (False, "Unable to get the endianness of the accelerometer device. The error was %s." % endianness)
    
    return (True, endianness)

def set_bdu(A, value):
    ''' Set the block data update '''
    (status, message) = write(A, __CTRL_REG4, __MASK_BDU, __BlockDataUpdate[value]) 
    if not status:
        return (False, "Unable to set the block data update of the accelerometer device. The error was %s." % message)

    return (True, None)
        
def get_bdu(A):
    ''' Get the block data update '''
    (status, bdu) = read_match(A, __CTRL_REG4, __MASK_BDU, __BlockDataUpdate)

    if not status:
        return (False, "Unable to get the block data update of the accelerometer device. The error was %s." % bdu)
    
    return (True, bdu)

def set_bootmode(A, value):
    ''' Set the boot mode '''
    (status, message) = write(A, __CTRL_REG5, __MASK_BOOT, __BootMode[value]) 
    if not status:
        return (False, "Unable to set the boot mode of the accelerometer device. The error was %s." % message)

    return (True, None)
        

def get_bootmode(A):
    ''' Get the boot mode '''
    (status, bootmode) = read_match(A, __CTRL_REG5, __MASK_BOOT, __BootMode)
    if not status:
        return (False, "Unable to get the bootmode of the accelerometer device. The error was %s." % bootmode)
    
    return (True, bootmode)

def enable_fifo(A, enabled):
    ''' Enable FIFO '''
    (status, message) = write(A, __CTRL_REG5, __MASK_FIFO_EN, __Enable[enabled]) 
    if not status:
        return (False, "Unable to enable or disable the FIFO on the accelerometer device. The error was %s." % message)

    return (True, None)

def isenabled_fifo(A):
    ''' Check if FIFO is enabled '''
    (status, enabled) = read_match(A, __CTRL_REG5, __MASK_FIFO_EN, __Enable)
    if not status:
        return (False, "Unable to check if the FIFO is enabled on the accelerometer device. The error was %s." % enabled)
    
    return (True, enabled)

def set_reference(A, value):
    ''' Set the reference value '''
    (status, message) = write(A, __REFERENCE, 0xff, value) 
    if not status:
        return (False, "Unable to set the reference on the accelerometer device. The error was %s." % message)

    return (True, None)
    
def get_reference(A):
    ''' Get the reference value '''
    (status, reference) = read(A, __REFERENCE, 0xff)
    if not status:
        return (False, "Unable to get the reference value on the accelerometer device. The error was: %s" % reference)

    return (True, reference)
    
def isdata_overrun(A):
    ''' Check if there was data overrun on each axis '''
    zor = False
    yor = False
    xor = False

    (status, zyx) = read(A, __STATUS_REG, 0xFF)
    if not status:
        return (False, "Unable to check if there was data overrun on the accelerometer device. The error was: %s" % zyx)

    # Check if there is overrun on at least one axis
    zyx_ov = bitOps.GetValueUnderMaskDictMatch(zyx, __MASK_ZYXOR, __Enable)
    if zyx_ov:
        zor = bitOps.GetValueUnderMaskDictMatch(zyx, __MASK_ZOR, __Enable)
        yor = bitOps.GetValueUnderMaskDictMatch(zyx, __MASK_YOR, __Enable)
        xor = bitOps.GetValueUnderMaskDictMatch(zyx, __MASK_XOR, __Enable)
    return (True, (xor, yor, zor))
    
def isdata_available(A):
    ''' Check if there is data available on each axis '''
    zda = False
    yda = False
    xda = False

    (status, zyx) = read(A, __STATUS_REG, 0xFF)
    if not status:
        return (False, "Unable to check if there is data available on the accelerometer device. The error was: %s" % zyx)

    # Check if there is overrun on at least one axis
    zyx_da = bitOps.GetValueUnderMaskDictMatch(zyx, __MASK_ZYXDA, __Enable)
    if zyx_da:
        zda = bitOps.GetValueUnderMaskDictMatch(zyx, __MASK_ZDA, __Enable)
        yda = bitOps.GetValueUnderMaskDictMatch(zyx, __MASK_YDA, __Enable)
        xda = bitOps.GetValueUnderMaskDictMatch(zyx, __MASK_XDA, __Enable)
    return (True, (xda, yda, zda))

def to_g (gain, h, l, sign):
    ''' 
    Convert the bytes to g
    h: high part of the information in two's complement
    l: low part of the information in two's complement
    '''
    # Convert to int format
    data_u2 = ((h << 8) + l) >> 4
    data = bitOps.TwosComplementToCustom(data_u2, 11)

    value = data * gain * sign

    return value

def get_x(A):        
    ''' Get the x axis acceleration data '''
    # Get the high and the low parts
    (status, l) = read(A, __OUT_X_L, 0xff)
    if not status:
        return (False, "Unable to get the x axis acceleration data on the accelerometer device. The error was: %s" % l)
    (status, h) = read(A, __OUT_X_H, 0xff)
    if not status:
        return (False, "Unable to get the x axis acceleration data on the accelerometer device. The error was: %s" % h)

    return (True, to_g (A['gain'], h, l, A['sign_def']['x']))

def get_y(A):        
    ''' Get the y axis acceleration data '''
    # Get the high and the low parts
    (status, l) = read(A, __OUT_Y_L, 0xff)
    if not status:
        return (False, "Unable to get the y axis acceleration data on the accelerometer device. The error was: %s" % l)
    (status, h) = read(A, __OUT_Y_H, 0xff)
    if not status:
        return (False, "Unable to get the y axis acceleration data on the accelerometer device. The error was: %s" % h)

    return (True, to_g (A['gain'], h, l, A['sign_def']['y']))

def get_z(A):        
    ''' Get the z axis acceleration data '''
    # Get the high and the low parts
    (status, l) = read(A, __OUT_Z_L, 0xff)
    if not status:
        return (False, "Unable to get the z axis acceleration data on the accelerometer device. The error was: %s" % l)
    (status, h) = read(A, __OUT_Z_H, 0xff)
    if not status:
        return (False, "Unable to get the z axis acceleration data on the accelerometer device. The error was: %s" % h)

    return (True, to_g (A['gain'], h, l, A['sign_def']['z']))

def get_xyz(A):
    ''' Get the three axis acceleration data '''
    (statusx, x) = get_x(A)
    (statusy, y) = get_y(A)
    (statusz, z) = get_z(A)
    if not statusx or not statusy or not statusz:
        return (False, "Unable to get the axis acceleration data")
    return (True, (x,y,z))

def get_block_xyz(A):
    ''' Get the three axis acceleration data '''
    # Get the six values for x, y and z low and high parts
    (status, values) = read_block (A, __OUT_X_L.get_addr()+0x80, 6)
    if not status:
        return (False, "Unable to get the axis acceleration data")

    # X
    l = values[0]
    h = values[1]
    x = to_g (A['gain'], h, l, A['sign_def']['x'])

    # Y
    l = values[2]
    h = values[3]
    y = to_g (A['gain'], h, l, A['sign_def']['y'])

    # Z
    l = values[4]
    h = values[5]
    z = to_g (A['gain'], h, l, A['sign_def']['z'])

    return (True, (x,y,z))

def calibrate (A, save = False):
    ''' Calibrate the accelerometer '''
    raw = None
    axis = 0
    taken_meassures = 0
    request_op = {0: "Put the device in Zb down position, and press enter to continue, p to print the meassures or c to stop the calibration: ",
                1: "Put the device in Zb up position, and press enter to continue or c to stop the calibration: ",
                2: "Put the device in Yb down position, and press enter to continue or c to stop the calibration: ",
                3: "Put the device in Yb up position, and press enter to continue or c to stop the calibration: ",
                4: "Put the device in Xb down position, and press enter to continue or c to stop the calibration: ",
                5: "Put the device in Xb up position, and press enter to continue or c to stop the calibration: "}

    collected_data = numpy.zeros ((6,), dtype = [('y', object),
                                                 ('w', object)])

    y_values = {0: [0,0,1.0], 1: [0,0,-1.0], 2: [0,1.0,0],
                3: [0,-1.0,0], 4: [1.0,0,0], 5: [-1.0,0,0]}

    while (raw != 'c' and axis < 6):
        # Set the right position and waiting for the confirmation to
        # do the calibration
        raw = 'p'
        while (raw == 'p'):
            raw = raw_input(request_op[axis])
            if raw == 'p':
                for i in xrange (10):
                    time.sleep(0.25)
                    x = get_x (A)
                    y = get_y (A)
                    z = get_z (A)
                    print("{:7.2f} {:7.2f} {:7.2f}".format(x, y, z))

        if raw == '':
            print "Collecting the axis raw data"
            collected_data[axis]['y'] = numpy.zeros ((A['cal_iter'], 3), dtype=float)
            collected_data[axis]['w'] = numpy.zeros ((A['cal_iter'], 4), dtype=float)

        while(raw == '' and taken_meassures < A['cal_iter']):
            
            time.sleep(0.25)
            x = get_x (A)
            y = get_y (A)
            z = get_z (A)
            print("{:7.2f} {:7.2f} {:7.2f}".format(x, y, z))
            collected_data[axis]['y'].put
            ([0+ (3*taken_meassures),
              1+ (3*taken_meassures),
              2+ (3*taken_meassures)], y_values[axis])

            collected_data[axis]['y'].put ([0+ (3*taken_meassures), 
                                            1+ (3*taken_meassures), 
                                            2+ (3*taken_meassures)], 
                                           y_values[axis])

            collected_data[axis]['w'].put ([0+ (4*taken_meassures),
                                            1+ (4*taken_meassures),
                                            2+ (4*taken_meassures),
                                            3+ (4*taken_meassures)],
                                           [x,y,z,1])

            taken_meassures += 1

        if raw == '':
            taken_meassures = 0
            axis += 1

    if axis < 6:
        return None

    if save:
        # Save the collected data in a file
        numpy.savez ("raw_accel_data", data=collected_data) 
        print "Collected data saved in the file raw_accel_data.npz"

    return get_params (collected_data, save)

def get_params (data, save = False):
    '''
    Get the calibration parameters matrix
    data: npz file or numpy array with the collected data
    save: indicates if the x_matrix has to be saved
    '''

    if type (data) == str:
        if os.path.isfile(data):
            try:
                collected_data = numpy.load (data)
                if 'data' in collected_data.keys ():
                    collected_data = collected_data['data']
                else:
                    return (False, "Unable read data from file %s. The file does not contain the 'data' key")
            except IOError as e:
                return (False, "Unable read data from file %s. The error was: I/O error (%s): %s" %(data, e.errno, e.strerror))
            
        else:
            return (False, "Unable to get the calibration parameters matrix for the accelerometer device. The npz file %s does not exist." % data)   
    elif type (data) != numpy.ndarray:
        return (False, "Unable to get the calibration parameters matrix for the accelerometer device. The data passed by argument is not correct")
    else:
        collected_data = data

    y_matrix = collected_data[0]['y']
    w_matrix = collected_data[0]['w']
    for i in xrange (1,6):
        y_matrix = numpy.concatenate ((y_matrix,collected_data[i]['y']), axis=0)
        w_matrix = numpy.concatenate ((w_matrix,collected_data[i]['w']), axis=0)

    # Calculate the calibration parameter matrix
    matrix_inv = numpy.linalg.inv (numpy.dot (w_matrix.T, w_matrix))
    x_matrix = numpy.dot (numpy.dot (matrix_inv, w_matrix.T), y_matrix)

    if save:
        # Save the collected data in a file
        numpy.savez ("x_matrix", data=x_matrix) 
        print "Calibration parameter matrix saved in the file x_matrix.npz"    
    
    return (True, x_matrix)
    

def rad_to_degrees (value):
    ''' Convert the value in radians to degrees '''
    temp = 180/numpy.pi
    return value*temp

def get_normalized_values (A, x_matrix, calibrate = False):
    ''' 
    Get the corresponding normalized values of each axis 
    x_matrix: path to the file with the calibration parameter matrix
    '''
    (status, xyz) = get_block_xyz (A)
    if not status:
        return (False, "Unable to get the normalized values. The error was: %s" % xyz)

    (x,y,z) = xyz

    if calibrate and not(os.path.isfile(x_matrix)):
        return (False, "Unable to get the normalize values of each axis. The file with the calibration parameter matrix does not exist")
        
    # Calibrate values to provide more accurate
    if calibrate:
        cal_matrix = numpy.load (x_matrix)['data']
        (x, y, z) = numpy.dot (numpy.array ([x,y,z,1]), cal_matrix)

    # Normalize the acceleration values
    norm = numpy.sqrt (x**2 + y**2 + z**2)
    if norm == 0:
        return (False, "Unable to get the normalized values. The norm is equal to 0")
    x_n = x/norm
    y_n = y/norm
    z_n = z/norm

    return (True, (x_n, y_n, z_n, norm))

def get_pitch_roll (A, x_matrix = None, calibrate = False, normalize = True, values = None):
    ''' 
    Get the pitch and the roll in degrees
    x_matrix: path to the file with the calibration parameter matrix
    '''
    if normalize:
        (status, values) = get_normalized_values (A,x_matrix, calibrate)    
        if not status:
            return (False, "Unable to get the pitch and the roll angles. The error was: \n%s" % values)
        # end if
    # end if
    (x,y,z,A_abs) = values

    p_rad = numpy.arcsin (-x)

    # Avoid singularity
    if abs (p_rad) == (numpy.pi/2):
        r_rad = 0.0
    elif abs (y/numpy.cos (p_rad)) > 1:
        # Maintain the sign
        angle = 1.0 * (y/numpy.cos (p_rad))/abs ((y/numpy.cos (p_rad)))
        r_rad = numpy.arcsin (angle)        
    else:
        r_rad = numpy.arcsin (y/numpy.cos (p_rad))
        
    p = rad_to_degrees (p_rad)
    r = rad_to_degrees (r_rad)
    return (True, (float (p), float (r), float (A_abs)))

def set_fifoth(A, value):
    ''' Set the FIFO threshold value '''
    (status, message) = write(A, __FIFO_CTRL_REG, __MASK_THR, value) 
    if not status:
        return (False, "Unable to set the FIFO threshold value on the accelerometer device. The error was: %s." % message)

    return (True, None)
    
def get_fifoth(A):
    ''' Get the FIFO threshold value '''
    (status, fifoth) = read(A, __FIFO_CTRL_REG, __MASK_THR)
    if not status:
        return (False, "Unable to get the FIFO threshold value on the accelerometer device. The error was %s." % fifoth)
    return (True, fifoth)
    
def set_fifomode(A, value):
    ''' Set FIFO mode '''
    (status, message) = write(A, __FIFO_CTRL_REG, __MASK_FM, __FIFOMode[value])         
    if not status:
        return (False, "Unable to set the FIFO mode on the accelerometer device. The error was %s." % message)

    return (True, None)

def get_fifomode(A):
    ''' Get FIFO mode '''
    (status, fifomode) = read_match(A, __FIFO_CTRL_REG, __MASK_FM, __FIFOMode)
    if not status:
        return (False, "Unable to get the fifo mode on the accelerometer device. The error was %s." % fifomode)

    return (True, fifomode)

def get_fifolevel(A):
    ''' Get FIFO stored data level '''
    (status, fifolevel) = read(A, __FIFO_SRC_REG, __MASK_FSS)
    if not status:
        return (False, "Unable to get the FIFO stored data level on the accelerometer device. The error was %s." % fifolevel)
    return (True, fifolevel)
    
def isfifo_empty(A):
    ''' FIFO empty '''
    (status, fifoempty) = read_match(A, __FIFO_SRC_REG, __MASK_EMPTY, __Enable)
    if not status:
        return (False, "Unable to check if FIFO is empty on the accelerometer device. The error was %s." % fifoempty)
    
    return (True, fifoempty)
    
def isfifo_full(A):
    ''' FIFO full '''
    (status, fifofull) = read_match(A, __FIFO_SRC_REG, __MASK_OVRN, __Enable)
    if not status:
        return (False, "Unable to check if FIFO is full on the accelerometer device. The error was %s." % fifofull)
    
    return (True, fifofull)
    
def get_watermark_status(A):
    ''' Get the watermark status (FIFO filling is greater or equal than watermark level) '''
    (status, wtm) = read_match(A, __FIFO_SRC_REG, __MASK_WTM, __WTM)
    if not status:
        return (False, "Unable to get the watermark status on the accelerometer device. The error was %s." % wtm)
    
    return (True, wtm)

def get_conf (A):

    A['conf'] = {}

    # CTRL_REG1
    A['conf']['ctrl1'] = {}
    ctrl1 = A['conf']['ctrl1']

    (status, value) = isenabled_x(A)
    if not status:
        return (False, "Unable to get the accelerometer configuration. The error was %s" % value)
    ctrl1['xen'] = value

    (status, value) = isenabled_y(A)
    if not status:
        return (False, "Unable to get the accelerometer configuration. The error was %s" % value)
    ctrl1['yen'] = value

    (status, value) = isenabled_z(A)
    if not status:
        return (False, "Unable to get the accelerometer configuration. The error was %s" % value)
    ctrl1['zen'] = value

    (status, value) = get_powermode(A)
    if not status:
        return (False, "Unable to get the accelerometer configuration. The error was %s" % value)
    ctrl1['lpen'] = value

    (status, value) = get_dr(A)
    if not status:
        return (False, "Unable to get the accelerometer configuration. The error was %s" % value)
    ctrl1['dr'] = value

    # CTRL_REG2
    A['conf']['ctrl2'] = {}
    ctrl2 = A['conf']['ctrl2']

    (status, value) = get_fds(A)
    if not status:
        return (False, "Unable to get the accelerometer configuration. The error was %s" % value)
    ctrl2['fds'] = value

    (status, value) = get_hpcf(A)
    if not status:
        return (False, "Unable to get the accelerometer configuration. The error was %s" % value)
    ctrl2['hpcf'] = value

    (status, value) = get_hpfm(A)
    if not status:
        return (False, "Unable to get the accelerometer configuration. The error was %s" % value)
    ctrl2['hpfm'] = value

    # CTRL_REG4
    A['conf']['ctrl4'] = {}
    ctrl4 = A['conf']['ctrl4']

    (status, value) = isenabled_hr(A)
    if not status:
        return (False, "Unable to get the accelerometer configuration. The error was %s" % value)
    ctrl4['hr'] = value

    (status, value) = get_scale(A)
    if not status:
        return (False, "Unable to get the accelerometer configuration. The error was %s" % value)
    ctrl4['fs'] = value

    (status, value) = get_endianness(A)
    if not status:
        return (False, "Unable to get the accelerometer configuration. The error was %s" % value)
    ctrl4['ble'] = value

    (status, value) = get_bdu(A)
    if not status:
        return (False, "Unable to get the accelerometer configuration. The error was %s" % value)
    ctrl4['bdu'] = value

    # CTRL_REG5
    A['conf']['ctrl5'] = {}
    ctrl5 = A['conf']['ctrl5']

    (status, value) = isenabled_fifo(A)
    if not status:
        return (False, "Unable to get the accelerometer configuration. The error was %s" % value)
    ctrl5['fifo_en'] = value

    (status, value) = get_bootmode(A)
    if not status:
        return (False, "Unable to get the accelerometer configuration. The error was %s" % value)
    ctrl5['boot'] = value

    # REFERENCE
    (status, value) = get_reference(A)
    if not status:
        return (False, "Unable to get the accelerometer configuration. The error was %s" % value)
    A['conf']['referene'] = value

    # FIFO_CTRL
    A['conf']['fifo_ctrl'] = {}
    fifo_ctrl = A['conf']['fifo_ctrl']

    (status, value) = get_fifoth(A)
    if not status:
        return (False, "Unable to get the accelerometer configuration. The error was %s" % value)
    fifo_ctrl['fifo_th'] = value

    (status, value) = get_fifomode(A)
    if not status:
        return (False, "Unable to get the accelerometer configuration. The error was %s" % value)
    fifo_ctrl['fifo_mode'] = value

    return (True, False)


def reset (A = None):
    '''
    Reset the values of the registers
    '''
    
    delete_accel = False
    # Create the accel
    if A == None:
        delete_accel = True
        A = Init ()
        if A['error'][0]:
            return (False, "Error while creating the accelerometer. Error message was: " + A['error'][1])

    # Write the default value of the CTRL_REG1 register
    (status, message) = write (A, __CTRL_REG1, 0xff, 0x7)
    if not status:
        return (False, "Error while writing the default value of the CTRL_REG1 register of the accelerometer device. Error message was: " + message)

    # Write the default value of the CTRL_REG2 register
    (status, message) = write (A, __CTRL_REG2, 0xff, 0x0)
    if not status:
        return (False, "Error while writing the default value of the CTRL_REG2 register of the accelerometer device. Error message was: " + message)

    # Write the default value of the CTRL_REG3 register
    (status, message) = write (A, __CTRL_REG3, 0xff, 0x0)
    if not status:
        return (False, "Error while writing the default value of the CTRL_REG3 register of the accelerometer device. Error message was: " + message)

    # Write the default value of the CTRL_REG4 register
    (status, message) = write (A, __CTRL_REG4, 0xff, 0x0)
    if not status:
        return (False, "Error while writing the default value of the CTRL_REG4 register of the accelerometer device. Error message was: " + message)
    
    # Write the default value of the CTRL_REG5 register
    (status, message) = write (A, __CTRL_REG5, 0xff, 0x0)
    if not status:
        return (False, "Error while writing the default value of the CTRL_REG5 register of the accelerometer device. Error message was: " + message)

    # Write the default value of the CTRL_REG6 register
    (status, message) = write (A, __CTRL_REG6, 0xff, 0x0)
    if not status:
        return (False, "Error while writing the default value of the CTRL_REG6 register of the accelerometer device. Error message was: " + message)

    # Write the default value of the REFERENCE register
    (status, message) = write (A, __REFERENCE, 0xff, 0x0)
    if not status:
        return (False, "Error while writing the default value of the REFERENCE register of the accelerometer device. Error message was: " + message)

    # Write the default value of the FIFO_CTRL_REG register
    (status, message) = write (A, __FIFO_CTRL_REG, 0xff, 0x0)
    if not status:
        return (False, "Error while writing the default value of the FIFO_CTRL_REG register of the accelerometer device. Error message was: " + message)

    # Finished test
    if delete_accel:
        del A
    
    return (True, None)


