# -*- coding: utf-8 -*-

# Library for the Raspberry Pi that interfaces with L3GD20 gyros on
# Polulu boards

from smbus import SMBus
from common_i2c import *
import bitOps
import numpy
import time

DEV_SLAVE_ADDR = 0x6b         # Default device slave address
DEV_BUS_ID     = 1            # Default device bus id
CALIBRATION_ITERATIONS = 20   # Default number for calibration operations

def Init (bus = None, 
          slaveAddr = DEV_SLAVE_ADDR, 
          cal_iter = CALIBRATION_ITERATIONS,
          scale = None):
    '''
    bus:       indicates the bus associated to the i2c device /dev/i2cX
    slaveAddr: indicates the address of the slave device
    cal_iter:  indicates the number of iterations to calibrate each axis
    scale:     indicates the measurement range
    '''
    G = {}
    G['bus'] = bus
    if bus == None:
        try:
            G['bus'] = SMBus(DEV_BUS_ID)
        except IOError as e:
            G['error'] = (True, "Unable to create the gyroscope. The error was: I/O error (%s): %s" %(e.errno, e.strerror))
            return G
    G['addr'] = DEV_SLAVE_ADDR
    G['cal_iter'] = cal_iter
    G['error'] = (False, None)

    # Set the gain for the out values
    if scale == None:
        (status, scale) = get_scale(G)
        if not status:
            G['error'] = (True, scale)
            return G
    elif not scale in __Scales.keys():
        G['error'] = (True, "Unable to create the gyroscope. The scale %s is not in the range." % scale)
        return G
    else:
        set_scale (G, scale)
                
    G['gain'] = __Gains [scale]    

    # Power on the device
    (status, error) = poweron (G)
    if not status:
        G['error'] = (True, "Unable to power on the gyroscope. The error was: %s" % error)
        return G

    return G
    
#############
# REGISTERS #
#############

__WHO_AM_I           = Register ('WHO_AM_I', 0x0f, mode = 'r')       # Device identification register
__CTRL_REG1          = Register ('CTRL_REG1', 0x20, mode = 'rw')       # Control register 1
__CTRL_REG2          = Register ('CTRL_REG2', 0x21, mode = 'rw')           # Control register 2
__CTRL_REG3          = Register ('CTRL_REG3', 0x22, mode = 'rw')           # Control register 3
__CTRL_REG4          = Register ('CTRL_REG4', 0x23, mode = 'rw')           # Control register 4
__CTRL_REG5          = Register ('CTRL_REG5', 0x24, mode = 'rw')           # Control register 5
__REFERENCE          = Register ('REFERENCE', 0x25, mode = 'rw')           # Reference value for interrupt generation
__OUT_TEMP           = Register ('OUT_TEMP', 0x26, mode = 'r')            # Output temperature
__STATUS_REG         = Register ('STATUS_REG', 0x27, mode = 'r')            # Status register
__OUT_X_L            = Register ('OUT_X_L', 0x28, mode = 'r')            # X-axis angular data rate LSB
__OUT_X_H            = Register ('OUT_X_H', 0x29, mode = 'r')            # X-axis angular data rate MSB
__OUT_Y_L            = Register ('OUT_Y_L', 0x2a, mode = 'r')            # Y-axis angular data rate LSB
__OUT_Y_H            = Register ('OUT_Y_H', 0x2b, mode = 'r')            # Y-axis angular data rate MSB
__OUT_Z_L            = Register ('OUT_Z_L', 0x2c, mode = 'r')            # Z-axis angular data rate LSB
__OUT_Z_H            = Register ('OUT_Z_H', 0x2d, mode = 'r')            # Z-axis angular data rate MSB
__FIFO_CTRL_REG      = Register ('FIFO_CTRL_REG', 0x2e, mode = 'rw')           # FIFO control register
__FIFO_SRC_REG       = Register ('FIFO_SRC_REG', 0x2f, mode = 'r')            # FIFO src register

# Is not posible to manage the device through interrupts (Only here for completeness)
__INT1_CFG           = Register ('INT1_CFG', 0x30, mode = 'rw')           # Interrupt 1 configuration register
__INT1_SRC           = Register ('INT1_SRC', 0x31, mode = 'r')            # Interrupt source register
__INT1_THS_XH        = Register ('INT1_THS_XH', 0x32, mode = 'rw')           # Interrupt 1 threshold level X MSB register
__INT1_THS_XL        = Register ('INT1_THS_XL', 0x33, mode = 'rw')           # Interrupt 1 threshold level X LSB register
__INT1_THS_YH        = Register ('INT1_THS_YH', 0x34, mode = 'rw')           # Interrupt 1 threshold level Y MSB register
__INT1_THS_YL        = Register ('INT1_THS_YL', 0x35, mode = 'rw')           # Interrupt 1 threshold level Y LSB register
__INT1_THS_ZH        = Register ('INT1_THS_ZH', 0x36, mode = 'rw')           # Interrupt 1 threshold level Z MSB register
__INT1_THS_ZL        = Register ('INT1_THS_ZL', 0x37, mode = 'rw')           # Interrupt 1 threshold level Z LSB register
__INT1_DURATION      = Register ('INT1_DURATION', 0x38, mode = 'rw')           # Interrupt 1 duration register


#################################
# MASKS TO MODIFY THE REGISTERS #
#################################

__MASK_Xen        = 0x01      # X enable
__MASK_Yen        = 0x02      # Y enable
__MASK_Zen        = 0x04      # Z enable
__MASK_PD         = 0x08      # Power-down
__MASK_SLEEP      = 0x0F     # Sleep
__MASK_DRBW       = 0xF0      # Data rate and Bandwidth
__MASK_BW         = 0x30      # Bandwidth
__MASK_DR         = 0xc0      # Output data rate

__MASK_HPCF       = 0x0f      # High pass filter cutoff frequency
__MASK_HPM        = 0x30      # High pass filter mode selection

__MASK_I2_EMPTY   = 0x01      # FIFO empty interrupt on DRDY/INT2
__MASK_I2_ORUN    = 0x02      # FIFO overrun interrupt on DRDY/INT2
__MASK_I2_WTM     = 0x04      # FIFO watermark interrupt on DRDY/INT2
__MASK_I2_DRDY    = 0x08      # Date-ready on DRDY/INT2
__MASK_PP_OD      = 0x10      # Push-pull / Open-drain
__MASK_H_LACTIVE  = 0x20      # Interrupt active configuration on INT1
__MASK_I1_BOOT    = 0x40      # Boot status available on INT1
__MASK_I1_Int1    = 0x80      # Interrupt enabled on INT1
__MASK_SIM        = 0x01      # SPI Serial interface selection

__MASK_FS         = 0x30      # Full scale selection
__MASK_BLE        = 0x40      # Big/little endian selection
__MASK_BDU        = 0x80      # Block data update

__MASK_OUT_SEL    = 0x03      # Out selection configuration
__MASK_INT_SEL    = 0xc0      # INT1 selection configuration
__MASK_HPEN       = 0x10      # High-pass filter enable
__MASK_FIFO_EN    = 0x40      # FIFO enable
__MASK_BOOT       = 0x80      # Reboot memory content

__MASK_ST_H       = 0xF0      # high part of the status register
__MASK_ST_L       = 0xF       # low part of the status register
__MASK_ZYXOR      = 0x80      # Z, Y, X axis overrun
__MASK_ZOR        = 0x40      # Z axis overrun
__MASK_YOR        = 0x20      # Y axis overrun
__MASK_XOR        = 0x10      # X axis overrun
__MASK_ZYXDA      = 0x08      # Z, Y, X data available
__MASK_ZDA        = 0x04      # Z data available
__MASK_YDA        = 0x02      # Y data available
__MASK_XDA        = 0x01      # X data available

__MASK_FM         = 0xe0      # FIFO mode selection
__MASK_THR        = 0x1f      # FIFO treshold - watermark level

__MASK_FSS        = 0x1f      # FIFO stored data level
__MASK_EMPTY      = 0x20      # FIFO empty bit
__MASK_OVRN       = 0x40      # Overrun status
__MASK_WTM        = 0x80      # Watermark status

# Is not posible to manage the device through interrupts (Only here for completeness)
__MASK_ANDOR       = 0x80      # And/Or configuration of interrupt events 
__MASK_LIR         = 0x40      # Latch interrupt request
__MASK_ZHIE        = 0x20      # Enable interrupt generation on Z high
__MASK_ZLIE        = 0x10      # Enable interrupt generation on Z low
__MASK_YHIE        = 0x08      # Enable interrupt generation on Y high
__MASK_YLIE        = 0x04      # Enable interrupt generation on Y low
__MASK_XHIE        = 0x02      # Enable interrupt generation on X high
__MASK_XLIE        = 0x01      # Enable interrupt generation on X low
__MASK_IA          = 0x40      # Int1 active
__MASK_ZH          = 0x20      # Int1 source Z high
__MASK_ZL          = 0x10      # Int1 source Z low
__MASK_YH          = 0x08      # Int1 source Y high
__MASK_YL          = 0x04      # Int1 source Y low
__MASK_XH          = 0x02      # Int1 source X high
__MASK_XL          = 0x01      # Int1 source X low  
__MASK_H           = 0x7f      # MSB
__MASK_L           = 0xff      # LSB
__MASK_WAIT        = 0x80      # Wait number of samples or not
__MASK_D           = 0x7f      # Duration of int1 to be recognized

##########################
# VALUE REGISTER MAPPING #
##########################

# Output data rate and bandwidth selection 
__DRBW = { 
    95:  {12.5:0x00, 25:0x01},
    190: {12.5:0x04, 25:0x05, 50:0x06, 70:0x07},
    380: {20:0x08,   25:0x09, 50:0x0a, 100:0x0b},
    760: {30:0x0c,   35:0x0d, 50:0x0e, 100:0x0f}
    }

# Power mode
__PowerMode = {'power_down': 0x0, 'sleep': 0x8, 'normal': 0x1}

# Enable/disable
__Enable = {True: 0x1, False: 0x0}

# High-pass filter mode configuration
__HPM = {
    'normal_with_reset':0x0,
    'reference_signal_for_filtering':0x1,
    'normal':0x2,
    'autoreset_on_interrupt':0x3
    }

# High-pass filter cut off frequency configuration [Hz]
__HPCF = {
    51.4  : { 760:0x00 },
    27    : { 380:0x00, 760:0x01 },
    13.5  : { 190:0x00, 380:0x01, 760:0x02 },
    7.2   : { 95:0x00, 190:0x01, 380:0x02, 760:0x03 },
    3.5   : { 95:0x01, 190:0x02, 380:0x03, 760:0x04 },
    1.8   : { 95:0x02, 190:0x03, 380:0x04, 760:0x05 },
    0.9   : { 95:0x03, 190:0x04, 380:0x05, 760:0x06 },
    0.45  : { 95:0x04, 190:0x05, 380:0x06, 760:0x07 },
    0.18  : { 95:0x05, 190:0x06, 380:0x07, 760:0x08 },
    0.09  : { 95:0x06, 190:0x07, 380:0x08, 760:0x09 },
    0.045 : { 95:0x07, 190:0x08, 380:0x09 },
    0.018 : { 95:0x08, 190:0x09 },
    0.009 : { 95:0x09 }
    }

# Block data update mode
__BlockDataUpdate = {'continous_update': 0x00, 'not_updated_until_reading': 0x01}

# Big/little endian data selection
__Endianness = {'big_endian': 0x00, 'little_endian': 0x01}

# Full scales selection
__Scales = {'250dps': 0x00, '500dps': 0x01, '2000dps': 0x02}
__Gains = {'250dps': 0.00875, '500dps': 0.0175, '2000dps': 0.07}

# Reboot memory content mode
__BootMode = {'normal': 0x00, 'reboot_memory_content': 0x01}

# Out selection configuration
__OutSel = {'LPF1': 0x0, 'HPF': 0x1, 'LPF2': 0x2}

# FIFO mode selection
__FIFOMode = {
    'bypass': 0x00,
    'FIFO': 0x01,
    'stream': 0x02,
    'stream_to_FIFO': 0x03,
    'bypass_to_stream': 0x04
    }

# Watermark status
__WTM = {'lower': 0x0, 'equal_greater': 0x1}

#############
# FUNCTIONS #
#############
    
def calibrateX(G):
    ''' Calibrate X axis '''
    buff = []
    for t in range(G['cal_iter']):
        (status, data_available) = isdata_available(G)
        while not data_available[0]:
            time.sleep(0.0001)
            (status, data_available) = isdata_available(G)

        buff.append(get_x(G)[1])
    G['meanX'] = numpy.mean(buff) 
    G['maxX'] = max(buff)
    G['minX'] = min(buff)
    
    return

def calibrateY(G):
    ''' Calibrate Y axis '''
    buff = []
    for t in range(G['cal_iter']):
        (status, data_available) = isdata_available(G)
        while not data_available[1]:
            time.sleep(0.0001)
            (status, data_available) = isdata_available(G)

        buff.append(get_y(G)[1])
    G['meanY'] = numpy.mean(buff) 
    G['maxY'] = max(buff)
    G['minY'] = min(buff)
    
    return

def calibrateZ(G):
    ''' Calibrate Y axis '''
    buff = []
    for t in range(G['cal_iter']):
        (status, data_available) = isdata_available(G)
        while not data_available[2]:
            time.sleep(0.0001)
            (status, data_available) = isdata_available(G)

        buff.append(get_z(G)[1])
    G['meanZ'] = numpy.mean(buff) 
    G['maxZ'] = max(buff)
    G['minZ'] = min(buff)

    return
    
def calibrate(G):
    ''' Calibrate the three axis '''
    calibrateX(G)
    calibrateY(G)
    calibrateZ(G)

    return

###################
# Print functions #
###################
        
def print_ctrl_reg1 (G):
    print "CTRL_REG1:"
    print "\tXen: " + str (isenabled_x(G)[1])
    print "\tYen: " + str (isenabled_y(G)[1])
    print "\tZen: " + str (isenabled_z(G)[1])
    print "\tPD: " + str (get_powermode(G)[1])
    print "\tDRBW: " + str (get_drbw(G)[1])

    return
    
def print_ctrl_reg2 (G):
    print "CTRL_REG2:"
    print "\tHPCF: " + str (get_hpcf(G)[1])
    print "\tHPM: " + str (get_hpfm(G)[1])

    return

def print_ctrl_reg4 (G):
    print "CTRL_REG4:"
    print "\tFS1-FS0: " + str (get_scale(G)[1])
    print "\tBLE: " + str (get_endianness(G)[1])
    print "\tBDU: " + str (get_bdu(G)[1])

    return

def print_ctrl_reg5 (G):
    print "CTRL_REG5:"
    print "\tOUT_SEL1-OUT_Sel0: " + str (get_outsel(G)[1])
    print "\tHPen: " + str (isenabled_hp(G)[1])
    print "\tFIFO_EN: " + str (isenabled_fifo(G)[1])
    print "\tBOOT: " + str (get_bootmode(G)[1])

    return

def print_reference (G):
    print "REFERENCE: " + str (get_reference(G)[1])

    return

def print_fifo_ctrl (G):
    print "FIFO_CTRL:"
    print "\tWTM4-WTM0: " + str (get_fifoth(G)[1])
    print "\tFM2-FM0: " + str (get_fifomode(G)[1])
    
    return
    
def print_configuration(G):
    ''' Print the configuration '''
    print "WHO_AM_I: " + hex (get_deviceid(G)[1])
    print_ctrl_reg1 (G)
    print_ctrl_reg2 (G)
    print_ctrl_reg4 (G)
    print_ctrl_reg5 (G)
    print_reference (G)
    print_fifo_ctrl (G)
    
    return
    
def get_deviceid(G):
    ''' Get WHO_AM_I value '''
    (status, devid) = read(G, __WHO_AM_I, 0xff)
    if not status:
        return (False, "Unable to get the device id. The error was: %s" % devid)

    return (True, devid)

def enable_x(G, enabled):
    ''' Enable x Axis '''
    (status, message) = write(G, __CTRL_REG1, __MASK_Xen, __Enable[enabled])
    if not status:
        return (False, 'Unable to enable or disable x axis for the gyroscope device. The error was %s' % message)
    return (True, None)
    
def isenabled_x(G):
    ''' Check if x axis is enabled '''
    (status, enabled) = read_match(G, __CTRL_REG1, __MASK_Xen, __Enable)
    if not status:
        return (False, 'Unable to check if x axis is enabled for the gyroscope device. The error was %s' % enabled)
    return (True, enabled)

def enable_y(G, enabled):
    ''' Enable y Axis '''
    (status, message) = write(G, __CTRL_REG1, __MASK_Yen, __Enable[enabled])
    if not status:
        return (False, 'Unable to enable or disable y axis for the gyroscope device. The error was %s' % message)
    return (True, None)

def isenabled_y(G):
    ''' Check if y axis is enabled '''
    (status, enabled) = read_match(G, __CTRL_REG1, __MASK_Yen, __Enable)
    if not status:
        return (False, 'Unable to check if y axis is enabled for the gyroscope device. The error was %s' % enabled)
    return (True, enabled)

def enable_z(G, enabled):
    ''' Enable z Axis '''
    (status, message) = write(G, __CTRL_REG1, __MASK_Zen, __Enable[enabled])
    if not status:
        return (False, 'Unable to enable or disable z axis for the gyroscope device. The error was %s' % message)
    return (True, None)

def isenabled_z(G):
    ''' Check if z axis is enabled '''
    (status, enabled) = read_match(G, __CTRL_REG1, __MASK_Zen, __Enable)
    if not status:
        return (False, 'Unable to check if z axis is enabled for the gyroscope device. The error was %s' % enabled)
    return (True, enabled)

def set_powerdownmode(G):
    ''' Set power-down mode '''
    (status, message) = write(G, __CTRL_REG1, __MASK_PD, __PowerMode['power_down'])
    if not status:
        return (False, 'Unable to power down the gyroscope device. The error was %s' % message)

    return (True, None)

def set_sleepmode(G):
    ''' Set sleep mode '''
    (status, message) = write(G, __CTRL_REG1, __MASK_SLEEP, __PowerMode['sleep'])
    if not status:
        return (False, 'Unable to set sleep mode for the gyroscope device. The error was %s' % message)

    return (True, None)

def poweron(G):
    ''' Set normal mode '''
    (status, message) = write(G, __CTRL_REG1, __MASK_PD, __PowerMode['normal'])
    if not status:
        return (False, 'Unable to power on the gyroscope device. The error was %s' % message)

    return (True, None)

def get_powermode(G):
    ''' Get power mode '''
    # Check sleep mode
    (status, powermode) = read_match(G, __CTRL_REG1, __MASK_SLEEP, __PowerMode)
    # Check power-down or normal mode
    if not status:
        (status, powermode) = read_match(G, __CTRL_REG1, __MASK_PD, __PowerMode)            
        
    if not status:
        return (False, 'Unable to get the power mode on the gyroscope device. The error was %s' % powermode)

    return (True, powermode)

def set_drbw(G, datarate, bandwidth):
    ''' Set data rate and bandwidth '''
    if datarate not in __DRBW.keys():
        return (False, 'Data rate %s not in range of data rate values on the gyroscope device.' % datarate)
    if bandwidth not in __DRBW[datarate].keys():
        return (False, 'Bandwidth %s cannot be assigned to data rate %s on the gyroscope device.' % (bandwidth, datarate))
    bits = __DRBW[datarate][bandwidth]

    (status, message) = write(G, __CTRL_REG1, __MASK_DRBW, bits)
    if not status:
        return (False, 'Unable to set the data rate and the bandwidth on the gyroscope device. The error was %s' % message)        

    return (True, None)

def get_drbw(G):
    ''' Get data rate and bandwidth '''
    (status, current) = read(G, __CTRL_REG1, __MASK_DRBW)
    if not status:
        return (False, "Unable to get the configurated data rate and the bandwidth for the gyroscope device. The error was: %s." % current)
    # Convert to the proper value in Hz
    for dr in __DRBW.keys():
        for bw in __DRBW[dr].keys():
            if __DRBW[dr][bw] == current:
                return (True, (dr, bw))

    # never reached
    return (False, "Unable to get the configurated data rate for the gyroscope device")

def set_hpfm(G, mode):
    ''' Set the high-pass filter mode '''
    if mode not in __HPM.keys():
        return (False, 'High pass filter mode %s is not in range of high pass frequency modes for the gyroscope device.' % mode)

    # Obtain the value and write it
    bits = __HPM[mode]
    (status, message) = write(G, __CTRL_REG2, __MASK_HPM, bits)
    if not status:
        return (False, 'Unable to set the high pass filter mode on the gyroscope device. The error was %s' % message)        
    return (True, None)

def get_hpfm(G):
    ''' Get the high-pass filter mode '''
    (status, current) = read(G, __CTRL_REG2, __MASK_HPM)
    if not status:
        return (False, "Unable to get the configurated high pass filter mode on the gyroscope device. The error was: %s." % current)

    for mode in __HPM.keys():
        if __HPM[mode] == current:
            return (True, mode)

    # never reached    
    return (False, "Unable to get the configurated high pass filter mode on the gyroscope device")

def set_hpcf(G, freq):
    ''' Set the high-pass filter cutoff frequency '''
    # Check frequency value
    if freq not in __HPCF.keys():
        return (False, "Frequency: %s is not in range of high pass frequency cut off values for the gyroscope device." % freq)
    (status, data) = get_drbw(G)
    if not status:
        return (status, "Unable to set high-pass filter cutoff frequency on the gyroscope device. The error was %s" % datarate)

    # Obtain the value and write it
    datarate = data[0]
    bits = __HPCF[freq][datarate]   
    (status, message) = write(G, __CTRL_REG2, __MASK_HPCF, bits)
    if not status:
        return (status, "Unable to set high-pass filter cutoff frequency on the gyro device. The error was %s" % message)

    return (True, None)
    
def get_hpcf(G):
    ''' Get the high-pass filter cutoff frequency '''
    (status, current) = read(G, __CTRL_REG2, __MASK_HPCF)
    if not status:
        return (False, "Unable to get the high pass filter cutoff frequency on the gyroscope device. The error was: %s" % current)
    (status, data) = get_drbw(G)
    if not status:
        return (status, "Unable to get the high pass filter cutoff frequency on the gyroscope device. The error was %s" % data)
    datarate = data[0]
    # Convert to the proper value in Hz
    for freq in __HPCF.keys():
        for dr in __HPCF[freq]:
            if dr == datarate:
                if __HPCF[freq][datarate] == current:
                    return (True, freq)

    # never reached    
    return (False, "Unable to get the high pass filter cutoff frequency on the gyroscope device.")

def set_scale(G, scale):
    ''' Set full scale '''
    if not scale in __Scales.keys():
        return (False, 'Unable to set the scale %s for the gyroscope device, the value is not in the range.' % scale)
    (status, message) = write(G, __CTRL_REG4, __MASK_FS, __Scales[scale]) 

    # Set the gain for the out values
    G['gain'] = __Gains [scale]    
    
    return (status, message)

def get_scale(G):
    ''' Get full scale '''
    (status, scale) = read_match(G, __CTRL_REG4, __MASK_FS, __Scales)
    if not status:
        return (False, "Unable to get the scale of the gyroscope device. The error was %s." % scale)

    return (True, scale)

    
def set_endianness(G, value):
    ''' Set endianness '''
    (status, message) = write(G, __CTRL_REG4, __MASK_BLE, __Endianness[value]) 
    if not status:
        return (False, "Unable to set the endianness of the gyroscope device. The error was %s.", message)

    return (True, None)
    
def get_endianness(G):
    ''' Get endianness '''
    (status, endianness) = read_match(G, __CTRL_REG4, __MASK_BLE, __Endianness)
    if not status:
        return (False, "Unable to get the endianness of the gyroscope device. The error was %s.", endianness)
    
    return (True, endianness)

def set_bdu(G, value):
    ''' Set the block data update '''
    (status, message) = write(G, __CTRL_REG4, __MASK_BDU, __BlockDataUpdate[value]) 
    if not status:
        return (False, "Unable to set the block data update of the gyroscope device. The error was %s.", message)

    return (True, None)

def get_bdu(G):
    ''' Get the block data update '''
    (status, bdu) = read_match(G, __CTRL_REG4, __MASK_BDU, __BlockDataUpdate)

    if not status:
        return (False, "Unable to get the block data update of the gyroscope device. The error was %s.", bdu)
    
    return (True, bdu)

    
def set_bootmode(G, value):
    ''' Set the boot mode '''
    (status, message) = write(G, __CTRL_REG5, __MASK_BOOT, __BootMode[value]) 
    if not status:
        return (False, "Unable to set the boot mode of the gyroscope device. The error was %s.", message)

    return (True, None)

def get_bootmode(G):
    ''' Get the boot mode '''
    (status, bootmode) = read_match(G, __CTRL_REG5, __MASK_BOOT, __BootMode)
    if not status:
        return (False, "Unable to get the bootmode of the gyroscope device. The error was %s.", bootmode)
    
    return (True, bootmode)


def enable_fifo(G, enabled):
    ''' Enable FIFO '''
    (status, message) = write(G, __CTRL_REG5, __MASK_FIFO_EN, __Enable[enabled]) 
    if not status:
        return (False, "Unable to enable or disable the FIFO on the gyroscope device. The error was %s.", message)

    return (True, None)


def isenabled_fifo(G):
    ''' Check if FIFO is enabled '''
    (status, enabled) = read_match(G, __CTRL_REG5, __MASK_FIFO_EN, __Enable)
    if not status:
        return (False, "Unable to check if the FIFO is enabled on the gyroscope device. The error was %s.", enabled)
    
    return (True, enabled)

def enable_hp(G, enabled):
    '''  Enable high-pass filter '''
    (status, message) = write(G, __CTRL_REG5, __MASK_HPEN, __Enable[enabled]) 
    if not status:
        return (False, "Unable to enable or disable the high-pass filter on the gyroscope device. The error was %s.", message)

    return (True, None)

def isenabled_hp(G):
    ''' Check if the high-pass filter is enabled '''
    (status, enabled) = read_match(G, __CTRL_REG5, __MASK_HPEN, __Enable)
    if not status:
        return (False, "Unable to check if the high-pass filter is enabled on the gyroscope device. The error was %s.", enabled)
    
    return (True, enabled)

def set_outsel(G, value):
    ''' Set the out selection configuration '''
    (status, message) = write(G, __CTRL_REG5, __MASK_OUT_SEL, __OutSel[value]) 
    if not status:
        return (False, "Unable to set the out selection configuration on the gyroscope device. The error was %s.", message)

    return (True, None)


def get_outsel(G):
    ''' Get the out selection configuration '''
    (status, out) = read_match(G, __CTRL_REG5, __MASK_OUT_SEL, __OutSel)
    if not status:
        return (False, "Unable to get the out selection configuration on the gyroscope device. The error was %s.", out)
    
    return (True, out)
    
def set_reference(G, value):
    ''' Set the reference value '''
    (status, message) = write(G, __REFERENCE, 0xff, value) 
    if not status:
        return (False, "Unable to set the reference on the gyroscope device. The error was %s.", message)

    return (True, None)

    
def get_reference(G):
    ''' Get the reference value '''
    (status, reference) = read(G, __REFERENCE, 0xff)
    if not status:
        return (False, "Unable to get the reference value on the gyroscope device. The error was: %s" % reference)
    
    return (True, reference)

def get_temp(G):
    ''' Get Temperature '''
    (status, temp) = read(G, __OUT_TEMP, 0xff)
    if not status:
        return (False, "Unable to get the temperature on the gyroscope device. The error was: %s" % temp)

    return (True, temp) 
    
def isdata_overrun(G):
    ''' Check if there was data overrun on each axis '''
    zor = False
    yor = False
    xor = False

    (status, zyx) = read(G, __STATUS_REG, 0xFF)
    if not status:
        return (False, "Unable to check if there was data overrun on the gyroscope device. The error was: %s" % zyx)

    # Check if there is overrun on at least one axis
    zyx_ov = bitOps.GetValueUnderMaskDictMatch(zyx, __MASK_ZYXOR, __Enable)
    if zyx_ov:
        zor = bitOps.GetValueUnderMaskDictMatch(zyx, __MASK_ZOR, __Enable)
        yor = bitOps.GetValueUnderMaskDictMatch(zyx, __MASK_YOR, __Enable)
        xor = bitOps.GetValueUnderMaskDictMatch(zyx, __MASK_XOR, __Enable)
    return (True, (xor, yor, zor))
    
def isdata_available(G):
    ''' Check if there is data available on each axis '''
    zda = False
    yda = False
    xda = False

    (status, zyx) = read(G, __STATUS_REG, 0xFF)
    if not status:
        return (False, "Unable to check if there is data available on the gyroscope device. The error was: %s" % zyx)

    # Check if there is overrun on at least one axis
    zyx_da = bitOps.GetValueUnderMaskDictMatch(zyx, __MASK_ZYXDA, __Enable)
    if zyx_da:
        zda = bitOps.GetValueUnderMaskDictMatch(zyx, __MASK_ZDA, __Enable)
        yda = bitOps.GetValueUnderMaskDictMatch(zyx, __MASK_YDA, __Enable)
        xda = bitOps.GetValueUnderMaskDictMatch(zyx, __MASK_XDA, __Enable)
    return (True, (xda, yda, zda))

def todegrees (G, h_u2, l):
    ''' 
    Convert the bytes to degrees 
    h_u2: high part of the information in two complement
    l: low part of the information
    '''
    # Convert high part (two complement format) into byte format
    h = bitOps.TwosComplementToByte(h_u2)
    # Check the sign of the number and obtain the degrees
    if (h < 0):
        return (h*256 - l) * G['gain']

    return (h*256 + l) * G['gain']
    
def get_x(G):        
    ''' Get the x axis angular rate data '''
    # Get the high and the low parts
    (status, l) = read(G, __OUT_X_L, 0xff)
    if not status:
        return (False, "Unable to get the x angular rate on the gyroscope device. The error was: %s" % l)
    (status, h_u2) = read(G, __OUT_X_H, 0xff)
    if not status:
        return (False, "Unable to get the x angular rate on the gyroscope device. The error was: %s" % h_u2)

    return (True, todegrees (G, h_u2, l))
    
def get_y(G):
    ''' Get the y axis angular rate data '''
    # Get the high and the low parts
    (status, l) = read(G, __OUT_Y_L, 0xff)
    if not status:
        return (False, "Unable to get the y angular rate. The error was: %s" % l)
    (status, h_u2) = read(G, __OUT_Y_H, 0xff)
    if not status:
        return (False, "Unable to get the y angular rate. The error was: %s" % h_u2)

    return (True, todegrees (G, h_u2, l))
    
def get_z(G):
    ''' Get the z axis angular rate data '''
    # Get the high and the low parts
    (status, l) = read(G, __OUT_Z_L, 0xff)
    if not status:
        return (False, "Unable to get the z angular rate. The error was: %s" % l)
    (status, h_u2) = read(G, __OUT_Z_H, 0xff)
    if not status:
        return (False, "Unable to get the z angular rate. The error was: %s" % h_u2)


    return (True, todegrees (G, h_u2, l))

def get_xyz(G):
    ''' Get the three axis angular rate data '''
    (statusx, x) = get_x(G)
    (statusy, y) = get_y(G)
    (statusz, z) = get_z(G)
    if not statusx or not statusy or not statusz:
        return (False, "Unable to get the axis angular rate data")
    return (True, (x,y,z))

def get_block_xyz (G):
    ''' Get the three axis angular rate data '''
    # Get the six values for x, y and z low and high parts
    (status, values) = read_block (G, __OUT_X_L.get_addr()+0x80, 6)
    if not status:
        return (False, "Unable to get the axis angular rate data")

    # X
    l = values[0]
    h = values[1]
    x = todegrees (G, h, l)

    # Y
    l = values[2]
    h = values[3]
    y = todegrees (G, h, l)

    # Z
    l = values[4]
    h = values[5]
    z = todegrees (G, h, l)

    return (True, (x,y,z))
    
def get_calx(G):
    ''' Get the calibrated angular rate data of the x axis '''
    (status, x) = get_x(G)
    if not status:
        return (False, "Unable to get the calibrated angular rate of the x axis. The error was: %s" % x)

    if(x >= G['minX'] and x <= G['maxX']):
        return (True, 0.0)
    else:
        return (True, float (x - G['meanX']))
            
def get_caly(G):
    ''' Get the calibrated angular rate data of the  y axis '''
    (status, y) = get_y(G)
    if not status:
        return (False, "Unable to get the calibrated angular rate of the y axis. The error was: %s" % y)

    if(y >= G['minY'] and y <= G['maxY']):
        return (True, 0.0)
    else:
        return (True, float (y - G['meanY']))
            
def get_calz(G):
    ''' Get the calibrated angular rate data of the z axis '''
    (status, z) = get_z(G)
    if not status:
        return (False, "Unable to get the calibrated angular rate of the z axis. The error was: %s" % z)

    if(z >= G['minZ'] and z <= G['maxZ']):
        return (True, 0.0)
    else:
        return (True, float (z - G['meanZ']))
    
def get_calxyz(G):
    ''' Get the calibrated angular rate data of each axis '''
    (statusx, x) = get_calx(G)
    (statusy, y) = get_caly(G)
    (statusz, z) = get_calz(G)
    if not statusx or not statusy or not statusz:
        return (False, "Unable to get the calibrated angular rate data of each axis")
    return (True, (x,y,z))
        
def set_fifoth(G, value):
    ''' Set the FIFO threshold value '''
    (status, message) = write(G, __FIFO_CTRL_REG, __MASK_THR, value) 
    if not status:
        return (False, "Unable to set the FIFO threshold value on the gyroscope device. The error was %s.", message)

    return (True, None)
    
def get_fifoth(G):
    ''' Get the FIFO threshold value '''
    (status, fifoth) = read(G, __FIFO_CTRL_REG, __MASK_THR)
    if not status:
        return (False, "Unable to get the FIFO threshold value on the gyroscope device. The error was %s.", fifoth)
    return (True, fifoth)
    
def set_fifomode(G, value):
    ''' Set FIFO mode '''
    (status, message) = write(G, __FIFO_CTRL_REG, __MASK_FM, __FIFOMode[value])         
    if not status:
        return (False, "Unable to set the FIFO mode on the gyroscope device. The error was %s.", message)

    return (True, None)

def get_fifomode(G):
    ''' Get FIFO mode '''
    (status, fifomode) = read_match(G, __FIFO_CTRL_REG, __MASK_FM, __FIFOMode)
    if not status:
        return (False, "Unable to get the fifo mode on the gyroscope device. The error was %s.", fifomode)
    
    return (True, fifomode)

def get_fifolevel(G):
    ''' Get FIFO stored data level '''
    (status, fifolevel) = read(G, __FIFO_SRC_REG, __MASK_FSS)
    if not status:
        return (False, "Unable to get the FIFO stored data level on the gyroscope device. The error was %s.", fifolevel)
    return (True, fifolevel)
    
def isfifo_empty(G):
    ''' FIFO empty '''
    (status, fifoempty) = read_match(G, __FIFO_SRC_REG, __MASK_EMPTY, __Enable)
    if not status:
        return (False, "Unable to check if FIFO is empty on the gyroscope device. The error was %s.", fifoempty)
    
    return (True, fifoempty)

    
def isfifo_full(G):
    ''' FIFO full '''
    (status, fifofull) = read_match(G, __FIFO_SRC_REG, __MASK_OVRN, __Enable)
    if not status:
        return (False, "Unable to check if FIFO is full on the gyroscope device. The error was %s.", fifofull)
    
    return (True, fifofull)
    
def get_watermark_status(G):
    ''' Get the watermark status (FIFO filling is greater or equal than watermark level) '''
    (status, wtm) = read_match(G, __FIFO_SRC_REG, __MASK_WTM, __WTM)
    if not status:
        return (False, "Unable to get the watermark status on the gyroscope device. The error was %s.", wtm)
    
    return (True, wtm)

def get_conf (G):

    G['conf'] = {}

    # CTRL_REG1
    G['conf']['ctrl1'] = {}
    ctrl1 = G['conf']['ctrl1']

    (status, value) = isenabled_x(G)
    if not status:
        return (False, "Unable to get the gyro configuration. The error was %s" % value)
    ctrl1['xen'] = value

    (status, value) = isenabled_y(G)
    if not status:
        return (False, "Unable to get the gyro configuration. The error was %s" % value)
    ctrl1['yen'] = value

    (status, value) = isenabled_z(G)
    if not status:
        return (False, "Unable to get the gyro configuration. The error was %s" % value)
    ctrl1['zen'] = value

    (status, value) = get_powermode(G)
    if not status:
        return (False, "Unable to get the gyro configuration. The error was %s" % value)
    ctrl1['pd'] = value

    (status, value) = get_drbw(G)
    if not status:
        return (False, "Unable to get the gyro configuration. The error was %s" % value)
    ctrl1['drbw'] = value

    # CTRL_REG2
    G['conf']['ctrl2'] = {}
    ctrl2 = G['conf']['ctrl2']

    (status, value) = get_hpcf(G)
    if not status:
        return (False, "Unable to get the gyro configuration. The error was %s" % value)
    ctrl2['hpcf'] = value

    (status, value) = get_hpfm(G)
    if not status:
        return (False, "Unable to get the gyro configuration. The error was %s" % value)
    ctrl2['hpfm'] = value

    # CTRL_REG4
    G['conf']['ctrl4'] = {}
    ctrl4 = G['conf']['ctrl4']

    (status, value) = get_scale(G)
    if not status:
        return (False, "Unable to get the gyro configuration. The error was %s" % value)
    ctrl4['fs'] = value

    (status, value) = get_endianness(G)
    if not status:
        return (False, "Unable to get the gyro configuration. The error was %s" % value)
    ctrl4['ble'] = value

    (status, value) = get_bdu(G)
    if not status:
        return (False, "Unable to get the gyro configuration. The error was %s" % value)
    ctrl4['bdu'] = value

    # CTRL_REG5
    G['conf']['ctrl5'] = {}
    ctrl5 = G['conf']['ctrl5']

    (status, value) = get_outsel(G)
    if not status:
        return (False, "Unable to get the gyro configuration. The error was %s" % value)
    ctrl5['outsel'] = value

    (status, value) = isenabled_hp(G)
    if not status:
        return (False, "Unable to get the gyro configuration. The error was %s" % value)
    ctrl5['hpen'] = value

    (status, value) = isenabled_fifo(G)
    if not status:
        return (False, "Unable to get the gyro configuration. The error was %s" % value)
    ctrl5['fifo_en'] = value

    (status, value) = get_bootmode(G)
    if not status:
        return (False, "Unable to get the gyro configuration. The error was %s" % value)
    ctrl5['boot'] = value

    # REFERENCE
    (status, value) = get_reference(G)
    if not status:
        return (False, "Unable to get the gyro configuration. The error was %s" % value)
    G['conf']['referene'] = value

    return (True, False)

def reset (G = None):
    '''
    Reset the values of the registers
    '''
    delete_gyro = False
    # Create the gyro
    if G == None:
        delete_gyro = True
        G = Init ()
        if G['error'][0]:
            return (False, "Error while creating the gyro. Error message was: " + G['error'][1])

    # Write the default value of the CTRL_REG1 register (normal mode set -> PD = 1)
    (status, message) = write (G, __CTRL_REG1, 0xff, 0xF)
    if not status:
        return (False, "Error while writing the default value of the CTRL_REG1 register. Error message was: " + message)

    # Write the default value of the CTRL_REG2 register
    (status, message) = write (G, __CTRL_REG2, 0xff, 0x0)
    if not status:
        return (False, "Error while writing the default value of the CTRL_REG2 register. Error message was: " + message)

    # Write the default value of the CTRL_REG3 register
    (status, message) = write (G, __CTRL_REG3, 0xff, 0x0)
    if not status:
        return (False, "Error while writing the default value of the CTRL_REG3 register. Error message was: " + message)

    # Write the default value of the CTRL_REG4 register
    (status, message) = write (G, __CTRL_REG4, 0xff, 0x0)
    if not status:
        return (False, "Error while writing the default value of the CTRL_REG4 register. Error message was: " + message)
    
    # Write the default value of the CTRL_REG5 register
    (status, message) = write (G, __CTRL_REG5, 0xff, 0x0)
    if not status:
        return (False, "Error while writing the default value of the CTRL_REG5 register. Error message was: " + message)

    # Write the default value of the REFERENCE register
    (status, message) = write (G, __REFERENCE, 0xff, 0x0)
    if not status:
        return (False, "Error while writing the default value of the REFERENCE register. Error message was: " + message)

    # Write the default value of the FIFO_CTRL_REG register
    (status, message) = write (G, __FIFO_CTRL_REG, 0xff, 0x0)
    if not status:
        return (False, "Error while writing the default value of the FIFO_CTRL_REG register. Error message was: " + message)

    # Finished test
    if delete_gyro:
        del G
    
    return (True, None)
