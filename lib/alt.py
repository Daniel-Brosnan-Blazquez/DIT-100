# -*- coding: utf-8 -*-

# Library for the Raspberry Pi that interfaces with LPS331AP
# pressure sensor on Polulu boards

from smbus import SMBus
from common_i2c import *
import bitOps
import numpy
import time
import os

DEV_SLAVE_ADDR = 0x5d           # Default device slave address
DEV_BUS_ID     = 1              # Default device bus id
PRESSURE_AT_SEA_LEVEL = 1013.25 # Pressure at sea level in mbars
GAS_CONSTANT = 287.04           # Specific gas constant for dry air
GRAVITY = 9.8                   # Gravitational acceleration

def Init (bus = None, 
          slaveAddr = DEV_SLAVE_ADDR, 
          pdr = 12.5, tdr = 12.5):
    '''
    bus:       indicates the bus associated to the i2c device /dev/i2cX
    slaveAddr: indicates the address of the slave device
    pdr:       indicates the data rate for the pressure sensor
    tdr:       indicates the data rate for the temperature sensor
    '''
    P = {}
    P['bus'] = bus
    if bus == None:
        try:
            P['bus'] = SMBus(DEV_BUS_ID)
        except IOError as e:
            P['error'] = (True, "Unable to power on the pressure sensor. The error was: I/O error (%s): %s" %(e.errno, e.strerror))
            return P            
    
    P['addr'] = DEV_SLAVE_ADDR
    P['error'] = (False, None)

    # Power on the device
    (status, error) = set_dr (P, pdr, tdr)
    if not status:
        P['error'] = (True, "Unable to power on the pressure sensor. The error was: %s" % error)
        return P

    (status, error) = poweron (P)
    if not status:
        P['error'] = (True, "Unable to power on the pressure sensor. The error was: %s" % error)
        return P

    return P


#############
# REGISTERS #
#############

__REF_P_XL        = Register ('REF_P_XL', 0x08, mode = 'rw')
__REF_P_L         = Register ('REF_P_L', 0x09, mode = 'rw')
__REF_P_H         = Register ('REF_P_H', 0x0A, mode = 'rw')
__WHO_AM_I        = Register ('WHO_AM_I', 0x0F, mode = 'r')
__RES_CONF        = Register ('RES_CONF', 0x10, mode = 'rw')
__CTRL_REG1       = Register ('CTRL_REG1', 0x20, mode = 'rw')
__CTRL_REG2       = Register ('CTRL_REG2', 0x21, mode = 'rw')
__CTRL_REG3       = Register ('CTRL_REG3', 0x22, mode = 'rw')

# Is not posible to manage the device through interrupts (Only here for completeness)
__INT_CFG         = Register ('INT_CFG', 0x23, mode = 'rw')
__INT_SOURCE      = Register ('INT_SOURCE', 0x24, mode = 'r')
__THS_P_L         = Register ('THS_P_L', 0x25, mode = 'rw')
__THS_P_H         = Register ('THS_P_H', 0x26, mode = 'rw')

__STATUS          = Register ('STATUS', 0x27, mode = 'r')
__PRESS_OUT_XL    = Register ('PRESS_OUT_XL', 0x28, mode = 'r')
__PRESS_OUT_L     = Register ('PRESS_OUT_L', 0x29, mode = 'r')
__PRESS_OUT_H     = Register ('PRESS_OUT_H', 0x2A, mode = 'r')
__TEMP_OUT_L      = Register ('TEMP_OUT_L', 0x2B, mode = 'r')
__TEMP_OUT_H      = Register ('TEMP_OUT_H', 0x2C, mode = 'r')
__AMP_CTRL        = Register ('AMP_CTRL', 0x2D, mode = 'rw')
__DELTA_XL        = Register ('DELTA_XL', 0x3C, mode = 'rw')
__DELTA_L         = Register ('DELTA_L', 0x3D, mode = 'rw')
__DELTA_H         = Register ('DELTA_H', 0x3E, mode = 'rw')


#################################
# MASKS TO MODIFY THE REGISTERS #
#################################

# RES_CONF
__MASK_AVGP       = 0x0F      # Pressure resolution mode
__MASK_AVGT       = 0x7F      # Termperature resolution mode

# CTRL_REG1
__MASK_SIM        = 0x01      # SPI Serial Mode Selection
__MASK_DELTA_EN   = 0x02      # Delta pressure enable
__MASK_BDU        = 0x04      # Block data update
__MASK_DIFF_EN    = 0x08      # Interrupt circuit enable
__MASK_DR         = 0x70      # Output data rate selection
__MASK_PD         = 0x80      # Power down control

# CTRL_REG2
__MASK_ONE_SHOT   = 0x01      # One shot enable
__MASK_AUTO_ZERO  = 0x02      # Autozero enable
__MASK_SWRESET    = 0x04      # Software reset
__MASK_BOOT       = 0x80      # Reboot memory content

# STATUS_REG
__MASK_T_DA       = 0x01      # Temperature data available
__MASK_P_DA       = 0x02      # Pressure data available
__MASK_T_OR       = 0x10      # Temperature data overrun
__MASK_P_OR       = 0x20      # Pressure data overrun

##########################
# VALUE REGISTER MAPPING #
##########################

# Pressure resolution configuration
__AVGP = { 
    1: 0x0, 2: 0x1, 4: 0x2, 8: 0x3, 16: 0x4, 32: 0x5, 
    64: 0x6, 128:0x7, 256: 0x8, 384: 0x9, 512: 0xA
    }

# Temperature resolution configuration
__AVGT = { 
    1: 0x0, 2: 0x1, 4: 0x2, 8: 0x3, 16: 0x4, 32: 0x5, 
    64: 0x6, 128:0x7
    }

# Pressure resolution
__PRES = {
    0.45: 0x70, 0.32: 0x71, 0.230: 0x72, 0.160: 0x73, 
    0.11: 0x74, 0.08: 0x75, 0.06: 0x76, 0.04:0x77,
    0.03: 0x78, 0.025: 0x79, 0.02: 0x7A
    }

# Power mode
__PowerMode = {'power_down': 0x0, 'active': 0x1}

# Output data rate selection 
__DR = {
    'one_shot': {'one_shot': 0x0},
    1:    {1:0x01},
    7:    {1:0x02,    7:0x05},
    12.5: {1:0x03,    12.5:0x06},
    25:   {1:0x04,    25:0x07}
    }

# Enable/disable
__Enable = {True: 0x1, False: 0x0}

__BlockDataUpdate = {'continous_update': 0x00, 'not_updated_until_reading': 0x01}

# Reboot memory content mode
__BootMode = {'normal': 0x00, 'reboot_memory_content': 0x01}

#############
# FUNCTIONS #
#############

###################
# Print functions #
###################
        
def print_res_conf (P):
    print "RES_CONF:" + str (get_pres(P)[1])

    return
    
def print_ctrl_reg1 (P):
    print "CTRL_REG1:"
    print "\tPD: " + str (get_powermode(P)[1])
    print "\tDR: " + str (get_dr(P)[1])
    print "\tBDU: " + str (get_bdu(P)[1])
    print "\tDELTA_EN: " + str (isenabled_delta(P)[1])

    return

def print_ctrl_reg2 (P):
    print "CTRL_REG2:"
    print "\tBOOT: " + str (get_bootmode(P)[1])

    return

def print_reference (P):
    print "REFERENCE: " + str (get_ref(P)[1])

    return
    
def print_configuration(P):
    ''' Print the configuration '''
    print_res_conf (P)
    print "WHO_AM_I: " + hex (get_deviceid(G)[1])
    print_ctrl_reg1 (P)
    print_ctrl_reg2 (P)
    print_reference (P)
    
    return

def set_pres(P, resolution):
    ''' Set pressure resolution '''
    value = None

    if resolution not in __PRES.keys():
        return (False, 'Press resolution %s not in range of pressure resolution values on the pressure sensor.' % resolution)

    # Check incompatibility with output data rate 
    if resolution == 0.02:
        (status, dr) = get_dr (P)
        if not status:
            return (False, "Unable to set the pressure resolution. The error was: %s." % dr)
        
        p_dr, t_dr = dr
        if p_dr == 25 and t_dr == 25:
            # Suggested value for this case (AN4159)
            value = 0x6a

    if value == None:
        value = __PRES[resolution]

    time.sleep (0.001)

    (status, message) = write(P, __RES_CONF, 0xFF, value)
    if not status:
        return (False, "Unable to set the pressure resolution. The error was: %s." % message)

    return (True, None)

def get_pres(P):
    ''' Get pressure resolution '''
    (status, current) = read(P, __RES_CONF, 0xFF)
    if not status:
        return (False, "Unable to set the pressure resolution. The error was: %s." % current)

    if current == 0x6A:
        # ODR = 25Hz/25Hz and RES_CONF = 0x6A
        return (True, 0.02)

    # Convert to the proper value in mbar
    for pres_res in __PRES.keys():
        if __PRES[pres_res] == current:
            return (True, (pres_res))

    # never reached
    return (False, "Unable to set the pressure resolution.")

def get_deviceid(P):
    ''' Get WHO_AM_I value '''
    (status, devid) = read(P, __WHO_AM_I, 0xff)
    if not status:
        return (False, "Unable to get the device id. The error was: %s" % devid)

    return (True, devid)

def powerdown(P):
    ''' Set power-down mode '''
    (status, message) = write(P, __CTRL_REG1, __MASK_PD, __PowerMode['power_down'])
    if not status:
        return (False, 'Unable to power down the pressure sensor. The error was %s' % message)

    return (True, None)

def poweron(P):
    ''' Set active mode '''
    (status, message) = write(P, __CTRL_REG1, __MASK_PD, __PowerMode['active'])
    if not status:
        return (False, 'Unable to power down the pressure sensor. The error was %s' % message)

    return (True, None)

def get_powermode(P):
    ''' Get power mode '''

    (status, powermode) = read_match(P, __CTRL_REG1, __MASK_PD, __PowerMode)         
    if not status:
        return (False, 'Unable to get the power mode on the pressure sensor. The error was %s' % powermode)

    return (True, powermode)

def set_dr(P, p_dr, t_dr):
    ''' Set data rate '''

    if p_dr not in __DR.keys():
        return (False, 'Unable to set the data rate %s for the pressure output. The value is not in range of data rate values for the pressure sensor.' % p_dr)

    if t_dr not in __DR[p_dr]:
        return (False, 'Unable to set the data rate %s for the temperature output with the value for the pressure output %s. The value is not in range of data rate values for the pressure sensor.' % (t_dr, p_dr))

    bits = __DR[p_dr][t_dr]
    (status, message) = write(P, __CTRL_REG1, __MASK_DR, bits)
    if not status:
        return (False, 'Unable to set the data rate on the pressure sensor. The error was %s' % message)        
    return (True, None)

def get_dr(P):
    ''' Get data rates '''
    (status, current) = read(P, __CTRL_REG1, __MASK_DR)
    if not status:
        return (False, "Unable to get the configurated data rates for the pressure sensor. The error was: %s." % current)
    
    # Convert to the proper value in Hz or 'one_shot' value
    for p_dr in __DR.keys():
        for t_dr in __DR[p_dr].keys():
            if __DR[p_dr][t_dr] == current:
                return (True, (p_dr, t_dr))

    # never reached
    return (False, "Unable to get the configurated data rates for the pressure sensor")

def set_bdu(P, value):
    ''' Set the block data update '''
    if value not in __BlockDataUpdate.keys():
        return (False, "Unable to set the block data update of the pressure sensor. The value %s is not in the allowed range" % value)

    (status, message) = write(P, __CTRL_REG1, __MASK_BDU, __BlockDataUpdate[value]) 
    if not status:
        return (False, "Unable to set the block data update of the pressure sensor. The error was %s." % message)

    return (True, None)
        
def get_bdu(P):
    ''' Get the block data update '''
    (status, bdu) = read_match(P, __CTRL_REG1, __MASK_BDU, __BlockDataUpdate)

    if not status:
        return (False, "Unable to get the block data update of the pressure sensor. The error was %s." % bdu)
    
    return (True, bdu)

def enable_delta(P, enabled):
    ''' Enable or disable delta pressure '''
    (status, message) = write(P, __CTRL_REG1, __MASK_DELTA_EN, __Enable[enabled])
    if not status:
        return (False, 'Unable to enable or disable delta pressure for pressure sensor. The error was %s' % message)
    return (True, None)
   
def isenabled_delta(P):
    ''' Check if delta pressure is enabled '''
    (status, enabled) = read_match(P, __CTRL_REG1, __MASK_DELTA_EN, __Enable)
    if not status:
        return (False, 'Unable to check if delta pressure is enabled for the pressure sensor. The error was %s' % enabled)
    return (True, enabled)

def set_bootmode(P, value):
    ''' Set the boot mode '''
    (status, message) = write(P, __CTRL_REG2, __MASK_BOOT, __BootMode[value]) 
    if not status:
        return (False, "Unable to set the boot mode of the pressure sensor. The error was %s." % message)

    return (True, None)

def get_bootmode(P):
    ''' Get the boot mode '''
    (status, bootmode) = read_match(P, __CTRL_REG2, __MASK_BOOT, __BootMode)
    if not status:
        return (False, "Unable to get the bootmode of the pressure sensor. The error was %s." % bootmode)
    
    return (True, bootmode)

def sw_reset (P):
    ''' Reset the device '''
    # Set BOOT to '1'
    (status, message) = set_bootmode (P, 'reboot_memory_content')
    if not status:
        return (False, "Unable to reset the pressure sensor. The error was %s" % message)

    # set SWRESET to '1'
    (status, message) = write(P, __CTRL_REG2, __MASK_SWRESET, __Enable[True]) 
    if not status:
        return (False, "Unable to reset the pressure sensor. The error was %s" % message)

    # set SWRESET to '0'
    (status, message) = write(P, __CTRL_REG2, __MASK_SWRESET, __Enable[False]) 
    if not status:
        return (False, "Unable to reset the pressure sensor. The error was %s" % message)


    return (True, None)    

def set_ref (P, active = True):
    ''' Set reference '''
    # set AUTO_ZERO to '1'
    (status, message) = write(P, __CTRL_REG2, __MASK_AUTO_ZERO, __Enable[active]) 
    if not status:
        return (False, "Unable to set the reference pressure. The error was %s" % message)

    if not active:
        (status, message) = write(P, __REF_P_XL, 0xFF, 0x0)
        if not status:
            return (False, "Unable to set the reference pressure. The error was: %s." % message)

        (status, message) = write(P, __REF_P_L, 0xFF, 0x0)
        if not status:
            return (False, "Unable to set the reference pressure. The error was: %s." % message)

        (status, message) = write(P, __REF_P_H, 0xFF, 0x0)
        if not status:
            return (False, "Unable to set the reference pressure. The error was: %s." % message)
        

    return (True, None)

def get_ref(P):        
    ''' Get the pressure data '''
    # Get the high, the medium and the low parts
    (status, xl) = read(P, __REF_P_XL, 0xff)
    if not status:
        return (False, "Unable to get the reference pressure on the pressure sensor. The error was: %s" % xl)
    (status, l) = read(P, __REF_P_L, 0xff)
    if not status:
        return (False, "Unable to get the reference pressure on the pressure sensor. The error was: %s" % l)

    (status, h) = read(P, __REF_P_H, 0xff)
    if not status:
        return (False, "Unable to get the reference pressure on the pressure sensor. The error was: %s" % h)

    data_u2 = (h << 16) + (l << 8) + xl
    data = bitOps.TwosComplementToCustom(data_u2, 23)

    return (True, data / 4096.0)
    
def start_conversion (P):
    ''' 
    Start new conversion using ONE_SHOT bit.
    Be careful because this function modifies the output data rate.
    '''
    (status, message) = set_dr (P, 'one_shot', 'one_shot')
    if not status:
        return (False, "Unable to start the conversion. The error was %s" % message)
    
    # Set the ONE_SHOT bit
    (status, message) = write(P, __CTRL_REG2, __MASK_ONE_SHOT, __Enable[True])
    if not status:
        return (False, "Unable to start the conversion. The error was %s" % message)

    # Get pressure and temperature values
    (status, p) = get_p (P)
    if not status:
        return (False, "Unable to start the conversion. The error was %s" % p)

    (status, t) = get_t (P)
    if not status:
        return (False, "Unable to start the conversion. The error was %s" % t)
   
    return (True, (p,t))

def isdata_overrun(P):
    ''' Check if there was data overrun for pressure and temperature values '''
    por = False
    tor = False

    (status, st) = read(P, __STATUS, 0xFF)
    if not status:
        return (False, "Unable to check if there was data overrun on the pressure sensor. The error was: %s" % st)

    por = bitOps.GetValueUnderMaskDictMatch(st, __MASK_P_OR, __Enable)
    tor = bitOps.GetValueUnderMaskDictMatch(st, __MASK_T_OR, __Enable)

    return (True, (por, tor))
    
def isdata_available(P):
    ''' Check if there is data available for pressure and temperature '''
    pda = False
    tda = False

    (status, st) = read(P, __STATUS, 0xFF)
    if not status:
        return (False, "Unable to check if there is data available on the pressure sensor. The error was: %s" % st)

    pda = bitOps.GetValueUnderMaskDictMatch(st, __MASK_P_DA, __Enable)
    tda = bitOps.GetValueUnderMaskDictMatch(st, __MASK_T_DA, __Enable)

    return (True, (pda, tda))

def to_pressure (h, l, xl):
    ''' Convert data from device to pressure '''

    data_u2 = (h << 16) + (l << 8) + xl
    data = bitOps.TwosComplementToCustom(data_u2, 23)

    return data/4096.0

def get_p(P):        
    ''' Get the pressure data '''
    # Get the high, the medium and the low parts
    (status, xl) = read(P, __PRESS_OUT_XL, 0xff)
    if not status:
        return (False, "Unable to get the pressure data on the pressure sensor. The error was: %s" % xl)
    (status, l) = read(P, __PRESS_OUT_L, 0xff)
    if not status:
        return (False, "Unable to get the pressure data on the pressure sensor. The error was: %s" % l)

    (status, h) = read(P, __PRESS_OUT_H, 0xff)
    if not status:
        return (False, "Unable to get the pressure data on the pressure sensor. The error was: %s" % h)

    data = to_pressure (h, l, xl)
    return (True, data)

def to_temperature (h, l):
    ''' Convert data from device to temperature '''    
    data_u2 = (h << 8) + l
    data = bitOps.TwosComplementToCustom(data_u2, 15)

    return 42.5 + data / 480.0

def get_t(P):        
    ''' Get the temperature data '''
    # Get the high and the low parts
    (status, l) = read(P, __TEMP_OUT_L, 0xff)
    if not status:
        return (False, "Unable to get the temperature data on the pressure sensor. The error was: %s" % l)

    (status, h) = read(P, __TEMP_OUT_H, 0xff)
    if not status:
        return (False, "Unable to get the temperature data on the pressure sensor. The error was: %s" % h)

    data = to_temperature (h,l)
    return (True, data)

def to_kelvin (temperature):
    return temperature + 273.15

def get_alt (P):
    ''' Get altitude refered to the current pressure and temperature '''
    (status, pressure) = get_p (P)
    if not status:
        return (False, "Unable to get the altitude. The error was %s" % pressure)
    (status, temperature) = get_t (P)
    if not status:
        return (False, "Unable to get the altitude. The error was %s" % temperature)

    temperature = int (temperature)

    if pressure == 0 or pressure < 0:
        return (False, "Unable to get the altitude. The pressure value is equal or less than 0")

    # Calculate altitude with the Hypsometric equation
    altitude = (numpy.log (PRESSURE_AT_SEA_LEVEL/pressure) * (to_kelvin (temperature)) * GAS_CONSTANT) / GRAVITY
    
    return (True, (float (altitude), float (temperature)))

def get_alt_ai (P):
    ''' Get altitude refered to the current pressure and temperature
    with auto-increment operations '''
    # Get pressure and temperature values
    (status, values) = read_block (P, __PRESS_OUT_XL.get_addr()+0x80, 5)
    if not status:
        return (False, "Unable to get the pressure and temperature data from altimeter device. The error was %s" % values)

    # Pressure
    xl = values[0]
    l = values[1]
    h = values[2]
    pressure = to_pressure (h,l,xl)

    # Temperature
    l = values[3]
    h = values[4]
    temperature = to_temperature (h,l)

    temperature = int (temperature)

    if pressure == 0 or pressure < 0:
        return (False, "Unable to get the altitude. The pressure value is equal or less than 0")

    # Calculate altitude with the Hypsometric equation
    altitude = (numpy.log (PRESSURE_AT_SEA_LEVEL/pressure) * (to_kelvin (temperature)) * GAS_CONSTANT) / GRAVITY
    
    return (True, (float (altitude), float (temperature), float (pressure)))

def get_conf (P):

    P['conf'] = {}

    # RES_CONF
    P['conf']['res'] = {}
    res = P['conf']['res']

    (status, value) = get_pres(P)
    if not status:
        return (False, "Unable to get the altimeter configuration. The error was %s" % value)
    res['dr'] = value

    # CTRL_REG1
    P['conf']['ctrl1'] = {}
    ctrl1 = P['conf']['ctrl1']

    (status, value) = get_powermode(P)
    if not status:
        return (False, "Unable to get the altimeter configuration. The error was %s" % value)
    ctrl1['pd'] = value

    (status, value) = get_dr(P)
    if not status:
        return (False, "Unable to get the altimeter configuration. The error was %s" % value)
    ctrl1['dr'] = value

    (status, value) = get_bdu(P)
    if not status:
        return (False, "Unable to get the altimeter configuration. The error was %s" % value)
    ctrl1['bdu'] = value

    (status, value) = isenabled_delta(P)
    if not status:
        return (False, "Unable to get the altimeter configuration. The error was %s" % value)
    ctrl1['delta_en'] = value

    # CTRL_REG2
    P['conf']['ctrl2'] = {}
    ctrl2 = P['conf']['ctrl2']

    (status, value) = get_bootmode(P)
    if not status:
        return (False, "Unable to get the altimeter configuration. The error was %s" % value)
    ctrl2['boot'] = value

    # REFERENCE
    (status, value) = get_ref(P)
    if not status:
        return (False, "Unable to get the gyro configuration. The error was %s" % value)
    P['conf']['referene'] = value

    return (True, False)

def reset (P = None):
    '''
    Reset the values of the registers
    '''
    
    delete_alt = False
    # Create the alt
    if P == None:
        delete_alt = True
        P = Init ()
        if P['error'][0]:
            return (False, "Error while creating the pressure sensor. Error message was: " + P['error'][1])

    # Reset
    (status, message) = sw_reset (P)
    if not status:
        return (False, "Error while reseting the pressure sensor. Error message was: %s" % message)

    # Finished test
    if delete_alt:
        del P
    
    return (True, None)


