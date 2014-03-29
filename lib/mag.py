# -*- coding: utf-8 -*-

# Library for the Raspberry Pi that interfaces with LSM303DLHC
# magnetometer on Polulu boards

from smbus import SMBus
from common_i2c import *
import bitOps
import numpy
import time
import os
import signal
import accel

DEV_SLAVE_ADDR = 0x1e         # Default device slave address
DEV_BUS_ID     = 1            # Default device bus id
CALIBRATION_ITERATIONS = 4   # Default number for calibration operations

# Sign definition fot the accelerometer in the AltIMU-10
# Invert the X axis to make the sensing values correspond to the device body axes.   
SIGN_DEFINITION = {'x': 1.0, 'y': -1.0, 'z': -1.0}

def Init (bus = None, 
          slaveAddr = DEV_SLAVE_ADDR, 
          cal_iter = CALIBRATION_ITERATIONS,
          scale = None, dr = 30, mode = 'continuous',
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
    M = {}
    M['bus'] = bus
    if bus == None:
        try:
            M['bus'] = SMBus(DEV_BUS_ID)
        except IOError as e:
            M['error'] = (True, "Unable to create the magnetometer. The error was: I/O error (%s): %s" %(e.errno, e.strerror))
            return M

    M['addr'] = DEV_SLAVE_ADDR
    M['cal_iter'] = cal_iter
    M['sign_def'] = sign_def
    M['error'] = (False, None)

    # Set the gain for the out values
    if scale == None:
        (status, scale) = get_scale(M)
        if not status:
            M['error'] = (True, "Unable to create the magnetometer. The error was %s " %scale)
            return M
    elif not scale in __Scales.keys():
        M['error'] = (True, "Unable to create the magnetometer. The scale %s is not in the range." % scale)
        return M
    else:
        set_scale (M, scale)
                
    M['gainxy'] = __Gains_XY [scale]    
    M['gainz'] = __Gains_Z [scale]    

    # Power on the device
    (status, message) = set_dr (M, dr)
    if not status:
        M['error'] = (True, "Error while setting the data rate. Error message was: " + message)
        return M

    (status, error) = poweron (M, mode)
    if not status:
        M['error'] = (True, "Unable to power on the magnetometer. The error was: %s" % error)
        return M

    return M


#############
# REGISTERS #
#############

__CRA        = Register ('CRA', 0x0, mode = 'rw')
__CRB        = Register ('CRB', 0x1, mode = 'rw')
__MR         = Register ('MR', 0x2, mode = 'rw')
__OUT_X_H    = Register ('OUT_X_H', 0x3, mode = 'r')
__OUT_X_L    = Register ('OUT_X_L', 0x4, mode = 'r')
__OUT_Y_H    = Register ('OUT_Y_H', 0x7, mode = 'r')
__OUT_Y_L    = Register ('OUT_Y_L', 0x8, mode = 'r')
__OUT_Z_H    = Register ('OUT_Z_H', 0x5, mode = 'r')
__OUT_Z_L    = Register ('OUT_Z_L', 0x6, mode = 'r')
__SR         = Register ('OUT_Z_H', 0x9, mode = 'r')
__OUT_TEMP_H = Register ('OUT_TEMP_H', 0x31, mode = 'r')
__OUT_TEMP_L = Register ('OUT_TEMP_H', 0x32, mode = 'r')


#################################
# MASKS TO MODIFY THE REGISTERS #
#################################

# CRA
__MASK_TEMP_EN    = 0x80      # Temperature sensor enable
__MASK_DR         = 0x1C      # Data rate selection

# CRB
__MASK_GN         = 0xE0      # Gain configuration

# MR
__MASK_MD         = 0x03      # Mode selection

# SR
__MASK_LOCK       = 0x40      # Data output register lock
__MASK_DRDY       = 0x80      # Data ready

##########################
# VALUE REGISTER MAPPING #
##########################

# Output data rate selection 
__DR = { 
    0.75: 0x0, 1.5: 0x1, 3.0: 0x2, 7.5: 0x3, 
    15: 0x4, 30: 0x5, 75: 0x6, 220:0x7
    }

# Full-scale data selection (gauss)
__Scales = {1.3: 0x01, 1.9: 0x02, 2.5: 0x03, 4.0: 0x04,
            4.7: 0x05, 5.6: 0x06, 8.1: 0x07}
__Gains_XY  = {1.3: 1.0/1100, 1.9: 1.0/855, 2.5: 1.0/670, 4.0: 1.0/450,
               4.7: 1.0/400, 5.6: 1.0/330, 8.1: 1.0/230}
__Gains_Z  = {1.3: 1.0/980, 1.9: 1.0/760, 2.5: 1.0/600, 4.0: 1.0/400,
               4.7: 1.0/355, 5.6: 1.0/295, 8.1: 1.0/205}

__PowerMode = {'continuous': 0x0, 'single': 0x1, 
               'sleep_bis': 0x2, 'sleep': 0x3}

# Enable/disable
__Enable = {True: 0x1, False: 0x0}

#############
# FUNCTIONS #
#############

###################
# Print functions #
###################
        
def print_cra (M):
    print "CRA:"
    print "\tTemp_en: " + str (isenabled_temp(M)[1])
    print "\tDR: " + str (get_dr(M)[1])

    return
    
def print_crb (M):
    print "CRB:"
    print "\tGN: " + str (get_scale(M)[1])

    return

def print_mr (M):
    print "MR:"
    print "\tMD: " + str (get_powermode(M)[1])

    return
    
def print_configuration(M):
    ''' Print the configuration '''
    print_cra (M)
    print_crb (M)
    print_mr (M)
    
    return

def enable_temp(M, enabled):
    ''' Enable temperature sensor '''
    (status, message) = write(M, __CRA, __MASK_TEMP_EN, __Enable[enabled])
    if not status:
        return (False, 'Unable to enable or disable temperature sensor for the magnetometer device. The error was %s' % message)
    return (True, None)

def isenabled_temp(M):
    ''' Check if temperature sensor is enabled '''
    (status, enabled) = read_match(M, __CRA, __MASK_TEMP_EN, __Enable)
    if not status:
        return (False, 'Unable to check if temperature sensor is enabled for the magnetometer device. The error was %s' % enabled)
    return (True, enabled)

def set_dr(M, datarate):
    ''' Set data rate '''
    if datarate not in __DR.keys():
        return (False, 'Data rate %s not in range of data rate values for the magnetometer device.' % datarate)

    bits = __DR[datarate]
    (status, message) = write(M, __CRA, __MASK_DR, bits)
    if not status:
        return (False, 'Unable to set the data rate on the magnetometer device. The error was %s' % message)        
    return (True, None)

def get_dr(M):
    ''' Get data rate '''
    (status, current) = read(M, __CRA, __MASK_DR)
    if not status:
        return (False, "Unable to get the configurated data rate for the magnetometer device. The error was: %s." % current)
    
    # Convert to the proper value in Hz
    for dr in __DR.keys():
        if __DR[dr] == current:
            return (True, dr)

    # never reached
    return (False, "Unable to get the configurated data rate for the magnetometer device")

def set_scale(M, scale):
    ''' Set full scale '''
    if not scale in __Scales.keys():
        return (False, 'Unable to set the scale %s for the magnetometer device, the value is not in the range.' % scale)

    (status, message) = write(M, __CRB, __MASK_GN, __Scales[scale]) 
    if not status:
        return (False, 'Unable to set the scale %s for the magnetometer device. The error was: %s.' % message)        

    # Set the gain for the out values
    M['gainxy'] = __Gains_XY [scale]    
    M['gainz'] = __Gains_Z [scale]    

    return (True, None)

def get_scale(M):
    ''' Get full scale '''
    (status, scale) = read_match(M, __CRB, __MASK_GN, __Scales)
    if not status:
        return (False, "Unable to get the scale of the magnetometer device. The error was %s." % scale)

    return (True, scale)

def set_powermode (M, mode):
    ''' Set power mode '''
    if mode not in __PowerMode.keys():
        return (False, 'Mode %s not in range of allowed power mode values for the magnetometer device' % mode)
    
    (status, message) = write (M, __MR, __MASK_MD, __PowerMode[mode])
    if not status:
        return (False, 'Unable to set the power mode on the magnetometer device. The error was %s' % message)

    return (True, None)

def poweron(M, mode):
    ''' Power on the device '''
    return set_powermode (M, mode)

def get_powermode(M):
    ''' Get power mode '''
    # Check power-down mode
    (status, powermode) = read_match(M, __MR, __MASK_MD, __PowerMode)

    if not status:
        return (False, 'Unable to get the power mode on the magnetometer device. The error was %s' % powermode)

    return (True, powermode)


###############################
# This functions are not used #
###############################
def get_x(M):        
    ''' Get the x axis magnetic field data '''
    # Get the high and the low parts
    (status, h_l) = get_raw_x (M)
    if not status:
        return (False, h_l)

    return (True, h_l * M['gainxy'])

def get_y(M):        
    ''' Get the y axis magnetic field data '''
    # Get the high and the low parts
    (status, h_l) = get_raw_y (M)
    if not status:
        return (False, h_l)

    return (True, h_l * M['gainxy'])

def get_z(M):        
    ''' Get the z axis magnetic field data '''
    # Get the high and the low parts
    (status, h_l) = get_raw_z (M)
    if not status:
        return (False, h_l)

    return (True, h_l * M['gainz'])

def get_xyz(M):
    ''' Get the three axis magnetic field data '''
    (statusx, x) = get_x(M)
    (statusy, y) = get_y(M)
    (statusz, z) = get_z(M)
    
    if not statusx or not statusy or not statusz:
        return (False, "Unable to get the axis magnetic field data")
    return (True, (x,y,z))

###############################

def get_raw_x(M):        
    ''' Get the x axis magnetic field data '''
    # Get the high and the low parts
    (status, l) = read(M, __OUT_X_L, 0xff)
    if not status:
        return (False, "Unable to get the x axis magnetic field data on the magnetometer device. The error was: %s" % l)
    (status, h) = read(M, __OUT_X_H, 0xff)
    if not status:
        return (False, "Unable to get the x axis magnetic field data on the magnetometer device. The error was: %s" % h)

    data = bitOps.TwosComplementToCustom((h << 8) + l, 15)

    return (True, data * M['sign_def']['x'])

def get_raw_y(M):        
    ''' Get the y axis magnetic field data '''
    # Get the high and the low parts
    (status, l) = read(M, __OUT_Y_L, 0xff)
    if not status:
        return (False, "Unable to get the y axis magnetic field data on the magnetometer device. The error was: %s" % l)
    (status, h) = read(M, __OUT_Y_H, 0xff)
    if not status:
        return (False, "Unable to get the y axis magnetic field data on the magnetometer device. The error was: %s" % h)

    data = bitOps.TwosComplementToCustom((h << 8) + l, 15)

    return (True, data * M['sign_def']['y'])

def get_raw_z(M):        
    ''' Get the z axis magnetic field data '''
    # Get the high and the low parts
    (status, l) = read(M, __OUT_Z_L, 0xff)
    if not status:
        return (False, "Unable to get the z axis magnetic field data on the magnetometer device. The error was: %s" % l)

    (status, h) = read(M, __OUT_Z_H, 0xff)
    if not status:
        return (False, "Unable to get the z axis magnetic field data on the magnetometer device. The error was: %s" % h)

    data = bitOps.TwosComplementToCustom((h << 8) + l, 15)

    return (True, data * M['sign_def']['z'])

def get_raw_xyz(M):
    ''' Get the three axis magnetic field data '''
    (statusx, x) = get_raw_x(M)
    (statusy, y) = get_raw_y(M)
    (statusz, z) = get_raw_z(M)
    if not statusx or not statusy or not statusz:
        return (False, "Unable to get the axis magnetic field data")
    return (True, (x,y,z))

def get_block_xyz(M):
    ''' Get the three axis magnetic field data '''
    # Get the six values for x, y and z low and high parts
    (status, values) = read_block (M, __OUT_X_H.get_addr()+0x80, 6)
    if not status:
        return (False, "Unable to get the axis magnetic field data")

    # X
    h = values[0]
    l = values[1]
    data = bitOps.TwosComplementToCustom((h << 8) + l, 15)
    x = data * M['sign_def']['x'] * M['gainxy']

    # Y
    h = values[4]
    l = values[5]
    data = bitOps.TwosComplementToCustom((h << 8) + l, 15)
    y = data * M['sign_def']['y'] * M['gainxy']


    # Z
    h = values[2]
    l = values[3]
    data = bitOps.TwosComplementToCustom((h << 8) + l, 15)
    z = data * M['sign_def']['z'] * M['gainz']

    return (True, (x,y,z))

def calibrate (M, fix = False, save = False, three_d = False, plot = False):
    ''' Calibrate the magnetometer '''
    from matplotlib import pyplot
    import pylab
    from mpl_toolkits.mplot3d import Axes3D

    M['stop'] = False
    M['finished'] = False
    M['axis'] = 0
    M['cal_iter_aux'] = 0
    axis = 0
    files = {0: "magz", 1: "magy", 2: "magx"}
    data_x = []
    data_y = []
    data_z = []
    data_axis = [data_x, data_y, data_z] 

    def control_c_handler(signal, frame):
        
        request_op = {0: "Put the device with body axis Yb down. \n",
                      1: "Put the device with body axis Xb down. \n"}

        raw = ''
        if M['axis'] < 2:
            raw = raw_input("\t" + request_op[M['axis']] +
                            "\tWhen the calibration starts, perform a full round rotation along the axis.\n" +
                            "\tWhen a full rotation is completed then press Control-C to stop the data collection. \n" 
                            "\tAlso press Control-C to cancel the data collection. \n" +
                            "\tPress p to plot the data. \n" +
                            "\tPress any key to collect the data from this axis or c key to cancel the operation:")        
        elif three_d:
            raw = raw_input("\tPerform 3D rotations." +
                            "\tPress Control-C to cancel the data collection. \n" +
                            "\tPress t to finish the data collection. \n" +
                            "\tPress p to plot the data. \n" +
                            "\tPress any key to collect the data or c key to cancel the operation:")        

        if raw == 'p':
            fig = pylab.figure()
            ax = Axes3D(fig)
            
            ax.scatter(M['x_data'], M['y_data'], M['z_data'])
            pyplot.show()
        elif raw == 'c':
            print "Canceling the calibration of the magnetometer device."
            M['stop'] = True
        elif M['axis'] == 2 and not three_d:
            print "The data collection finished. \n Calculating the calibration parameters"
            M['stop'] = True
            M['finished'] = True
        # Case to make 3D rotations
        elif raw == 't' and three_d:
            print "The data collection finished. \n Calculating the calibration parameters"
            M['stop'] = True
            M['finished'] = True

        M['axis'] += 1
        M['cal_iter_aux'] = 0

        return None

    signal.signal(signal.SIGINT, control_c_handler)

    raw = raw_input("\tPut the device with body axis Zb down.\n" +
                    "\tWhen the calibration starts, perform a full round rotation along the axis. \n" +
                    "\tWhen a full rotation is completed then press Control-C to stop the data collection. \n" 
                    "\tAlso press Control-C to cancel the data collection. \n" +
                    "\tPress any key to start the calibration or c key to cancel the operation:")

    if raw == 'c':
        return (False, "Canceling the calibration of the magnetometer device.")

    # Data collection    
    (status, x) = get_raw_x (M)
    (status, y) = get_raw_y (M)
    (status, z) = get_raw_z (M)

    collected_data = numpy.array ([[x,y,z]])
    if plot:
        M['x_data'] = numpy.array ([x])
        M['y_data'] = numpy.array ([y])
        M['z_data'] = numpy.array ([z])

    axis_data = numpy.array ([[x,y,z]])

    print("{:7.2f} {:7.2f} {:7.2f}".format(x, y, z))
    M['cal_iter_aux'] += 1

    while (not M['stop']):
        while (M['cal_iter_aux'] < M['cal_iter'] and not M['stop']) or (not fix and not M['stop']):
            time.sleep(0.25)        
            (status, x) = get_raw_x (M)
            (status, y) = get_raw_y (M)
            (status, z) = get_raw_z (M)

            print("{:7.2f} {:7.2f} {:7.2f}".format(x, y, z))

            collected_data = numpy.concatenate ((collected_data, numpy.array ([[x,y,z]])), axis=0)
            if plot:
                M['x_data'] = numpy.concatenate ((M['x_data'], [x]))
                M['y_data'] = numpy.concatenate ((M['y_data'], [y]))
                M['z_data'] = numpy.concatenate ((M['z_data'], [z]))

            if save and axis != M['axis'] and axis < 3:
                # Save the collected data in a file
                data_axis[axis].append (axis_data)
                print "Collected data for one axis saved in the file %s.npz" % files[axis]
                axis_data = numpy.array ([[x,y,z]])
                axis += 1
            
            elif save:
                axis_data = numpy.concatenate ((axis_data, numpy.array ([[x,y,z]])), axis=0)

            M['cal_iter_aux'] += 1

    if not M['finished']:
        return (False, "The calibration was not completed")

    if save and axis != M['axis'] and axis < 3:
        # Save the collected data in a file
        data_axis[axis].append(axis_data)
        print "Collected data for one axis saved in the file %s.npz" % files[axis]

    if save:
        # Save the collected data in a file
        numpy.savez ("raw_mag_data", data=collected_data, 
                     x=data_x[0], y=data_y[0], z=data_z[0])
        print "Collected data saved in the file raw_mag_data.npz"
 
    return (True, None)

##########################################################################
# Function that uses the calibration method explained in the AN3192 file #
#                               Not used                                 #
##########################################################################
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
                if 'data' in collected_data.keys () \
                and 'x' in collected_data.keys () \
                and 'y' in collected_data.keys () \
                and 'z' in collected_data.keys ():
                    collected_data = collected_data['data']
                    magx = numpy.load (data)['x']
                    magy = numpy.load (data)['y']
                    magz = numpy.load (data)['z']
                else:
                    return (False, "Unable read data from file %s. The file does not contain the 'data' key")
            except IOError as e:
                return (False, "Unable read data from file %s. The error was: I/O error (%s): %s" %(data, e.errno, e.strerror))
            
        else:
            return (False, "Unable to get the calibration parameters matrix for the accelerometer device. The npz file %s does not exist." % data)   
    elif type (data[0]) != numpy.ndarray or \
            type (data[1]) != numpy.ndarray or \
            type (data[2]) != numpy.ndarray or \
            type (data[3]) != numpy.ndarray:
        for d in data:
            print type (d)
        return (False, "Unable to get the calibration parameters matrix for the accelerometer device. The data passed by argument is not correct")
    else:
        collected_data = data[0]
        magx = data[1]
        magy = data[2]
        magz = data[3]

    # Calculate X
    H = []
    w = []
    for data in collected_data:
        (x, y, z) = data
        H.append ([x,y,z,-(y**2),-(z**2),1])
        w.append ([x**2])

    H = numpy.array (H)
    w = numpy.array (w)

    matrix_inv = numpy.linalg.inv (numpy.dot (H.T, H))
    X = numpy.dot (numpy.dot (matrix_inv, H.T), w)

    # Calculate M_OS
    M_OSx = X[0][0]/2.0
    M_OSy = X[1][0]/(2.0*X[3][0])
    M_OSz = X[2][0]/(2.0*X[4][0])
    M_OS = [M_OSx, M_OSy, M_OSz]

    # Calculate A, B and C
    A = X[5][0] + M_OSx**2 + X[3][0]*M_OSy**2 + X[4][0]*M_OSz**2
    B = A/X[3][0]
    C = A/X[4][0]

    # Calculate M_SC
    M_SCx = numpy.sqrt (A)
    M_SCy = numpy.sqrt (B)
    M_SCz = numpy.sqrt (C)
    M_SC = [M_SCx, M_SCy, M_SCz]

    # Calculate xx, yy and zz
    Rx = []
    Ry = []
    Rz = []
    mag_data = ((Rx, magx), (Ry, magy), (Rz, magz))

    for data in mag_data:
        R = data[0]
        mag = data [1]
        Mx = mag.T[0]
        My = mag.T[1]
        Mz = mag.T[2]

        # Calculate xx, yy and zz
        xx = Mx - M_OSx
        zz = My - M_OSy
        yy = Mz - M_OSz

#        print "Equation 33: %s" % (xx**2/A + yy**2/B + zz**2/C)

        # Calculate xxx, yyy and zzz
        xxx = xx/M_SCx
        yyy = yy/M_SCy
        zzz = zz/M_SCz

        print "Equation 36: %s" % (xxx**2 + yyy**2 + zzz**2)

        # Calculate H
        H = numpy.array ([xxx, yyy, zzz])
        H = H.T

        # Calculate w
        w = numpy.sqrt (xxx**2 + yyy**2 + zzz**2)
        w = w.T

        # Calculate X matrix
        matrix_inv = numpy.linalg.inv (numpy.dot (H.T, H))
        X = numpy.dot (numpy.dot (matrix_inv, H.T), w)

        # Calculate R vector
        R.append (X/numpy.sqrt (X[0]**2 + X[1]**2 + X[2]**2))        

    Rx = Rx[0]
    Ry = Ry[0]
    Rz = Rz[0]

    # Calculate M_m
    M_m = numpy.array ([Rx, Ry, Rz])
    M_m = M_m.T

    MR = numpy.dot (M_m, numpy.identity (3) * (1.0/numpy.array ((M_SC))))
    
    if save:
        # Save the calibration parameters matrix
        cal_matrix = numpy.concatenate ((MR, numpy.array (([M_OS])).T), axis = 1)
        numpy.savez ("cal_mag", data=cal_matrix) 
        print "Calibration parameters matrix saved in the file cal_mag.npz"

    return (True, (MR, M_OS))    

def get_min_max (data_file, save = False):
    '''
    Get the minimum and maximum values of each axis in the data file
    data_file: npz file with the collected data
    save: indicates if the x_matrix has to be saved
    '''
    if os.path.isfile(data_file):
        try:
            collected_data = numpy.load (data_file)
            if 'data' in collected_data.keys ():
                collected_data = collected_data['data']
            else:
                return (False, "Unable read data from file %s. The file does not contain the 'data' key")
        except IOError as e:
            return (False, "Unable read data from file %s. The error was: I/O error (%s): %s" %(data, e.errno, e.strerror))

    minx = min (collected_data.T[0])
    maxx = max (collected_data.T[0])
    miny = min (collected_data.T[1])
    maxy = max (collected_data.T[1])
    minz = min (collected_data.T[2])
    maxz = max (collected_data.T[2])

    if save:
        # Save maximum and minimum values
        numpy.savez ("max_min", minx=minx, maxx=maxx, miny=miny, maxy=maxy, minz=minz, maxz=maxz) 
        print "Minimum and maximum values saved in max_min.npz file"

    return (True, (minx, maxx, miny, maxy, minz, maxz))    


def get_normalized_values (M, min_max, calibrate = False):
    ''' 
    Get the corresponding normalized values of each axis 
    min_max: calibration values
    calibrate: indicates if it is needed to calibrate the values
    '''
    (status, xyz) = get_raw_xyz (M)
    if not status:
        return (False, "Unable to get the normalized values. The error was: %s" % xyz)

    (x,y,z) = xyz

    if calibrate and type (min_max) != dict:
        return (False, "Unable to get the normalize values of each axis. The variable that has to contain the calibration parameters is not of type dictionary")

    # Calibrate values to provide more accurate
    if calibrate:
        minx = min_max['minx']
        maxx = min_max['maxx']
        miny = min_max['miny']
        maxy = min_max['maxy']
        minz = min_max['minz']
        maxz = min_max['maxz']

        x = ((x-minx) / (maxx - minx)) * 2 -1
        y = ((y-miny) / (maxy - miny)) * 2 -1
        z = ((z-minz) / (maxz - minz)) * 2 -1

    # Normalize the magnetometer values
    norm = numpy.sqrt (x**2 + y**2 + z**2)
    x_n = x/norm
    y_n = y/norm
    z_n = z/norm

    return (True, (x_n, y_n, z_n))

def degrees_to_rad (value):
    ''' Convert the value in radians to degrees '''
    temp = numpy.pi/180
    return value*temp

def rad_to_degrees (value):
    ''' Convert the value in radians to degrees '''
    temp = 180/numpy.pi
    return value*temp

def tilt_heading (pitch_roll, mag_values = None, M = None, calibrate = True, cal_mag = None):

    if mag_values == None:
        (status, xyz) = get_normalized_values (M, cal_mag, calibrate)
        if not status:
            return (False, "Unable to get the heading angle. The error was: %s" % xyz)

        (x,y,z) = xyz
    else:
        (x,y,z) = mag_values

    p, r = pitch_roll

    p = degrees_to_rad (p)
    r = degrees_to_rad (r)

    Mx2 = x * numpy.cos(p) + z * numpy.sin(p)
    My2 = x * numpy.sin(r) * numpy.sin(p) + y * numpy.cos(r) - z * numpy.sin(r) * numpy.cos (p)
    Mz2 = -x * numpy.cos(r) * numpy.sin(p) + y * numpy.sin(r) + z * numpy.cos(r) * numpy.cos (p)

    if Mx2 > 0 and My2 >= 0:
        heading = rad_to_degrees (numpy.arctan (My2/Mx2))
    elif Mx2 < 0:
        heading = 180.0 + rad_to_degrees (numpy.arctan (My2/Mx2))
    elif Mx2 > 0 and My2 <= 0:
        heading = 360.0 + rad_to_degrees (numpy.arctan (My2/Mx2))
    elif Mx2 == 0 and My2 < 0:
        heading = 270.0
    else:
        heading = 90.0

    M_abs = numpy.sqrt (Mx2**2 + My2**2 + Mz2**2)

    return (True, (float (heading), float (M_abs)))

        

def get_heading (M, A, cal_mag, cal_acc, calibrate = False, view_notilt = False):
    ''' Get the heading angle '''

    data = numpy.load (cal_mag)
    min_max = {}
    min_max['minx'] = data['minx']
    min_max['maxx'] = data['maxx']
    min_max['miny'] = data['miny']
    min_max['maxy'] = data['maxy']
    min_max['minz'] = data['minz']
    min_max['maxz'] = data['maxz']

    (status, xyz) = get_normalized_values (M, min_max, calibrate)
    if not status:
        return (False, "Unable to get the heading angle. The error was: %s" % xyz)

    (x,y,z) = xyz

    if view_notilt:

        if x > 0 and y >= 0:
            heading = rad_to_degrees (numpy.arctan (y/x))
        elif x < 0:
            heading = 180.0 + rad_to_degrees (numpy.arctan (y/x))
        elif x > 0 and y <= 0:
            heading = 360.0 + rad_to_degrees (numpy.arctan (y/x))
        elif x == 0 and y < 0:
            heading = 270.0
        else:
            heading = 90.0

        M_abs = numpy.sqrt (x**2 + y**2 + z**2)
        
        print("\theading_notilt = {:7.2f}".format(heading))
            
        print "\t|M_notilt| = %6.3F" % M_abs

    (status, p_r) = accel.get_pitch_roll (A, cal_acc, calibrate)
    if not status:
        return (False, "Unable to get the pitch and roll angles. The error was: %s" % p_r)

    (p,r, a_abs) = p_r

    (status, values) = tilt_heading ((p,r), mag_values = (x,y,z))
    if not status:
        return (False, "Unable to get heading. The error was %s" % values)

    heading, M_abs = values

    return (True, (float (heading), float (M_abs)))

def isdata_ready(M):
    ''' Check if there is data available on each axis '''
    (status, drdy) = read_match(M, __SR, __MASK_DRDY, __Enable)
    if not status:
        return (False, "Unable to check if there is data ready on the magnetometer device. The error was: %s" % drdy)

    return (True, drdy)

def get_temp(M):
    ''' Get Temperature '''
    (status, temp_h) = read(M, __OUT_TEMP_H, 0xff)
    if not status:
        return (False, "Unable to get the temperature on the magnetometer device. The error was: %s" % temp_h)

    (status, temp_l) = read(M, __OUT_TEMP_L, 0xff)
    if not status:
        return (False, "Unable to get the temperature on the magnetometer device. The error was: %s" % temp_l)

    data = bitOps.TwosComplementToCustom(((temp_h << 8) + temp_l) >> 4, 11)    

    return (True, float (data/8.0)) 

def get_conf (M):

    M['conf'] = {}

    # CRA
    M['conf']['cra'] = {}
    cra = M['conf']['cra']

    (status, value) = get_dr(M)
    if not status:
        return (False, "Unable to get the magnetometer configuration. The error was %s" % value)
    cra['dr'] = value

    (status, value) = isenabled_temp(M)
    if not status:
        return (False, "Unable to get the magnetometer configuration. The error was %s" % value)
    cra['temp_en'] = value

    # CRB
    M['conf']['crb'] = {}
    crb = M['conf']['crb']

    (status, value) = get_scale(M)
    if not status:
        return (False, "Unable to get the magnetometer configuration. The error was %s" % value)
    crb['gn'] = value

    # MR
    M['conf']['mr'] = {}
    mr = M['conf']['mr']

    (status, value) = get_powermode(M)
    if not status:
        return (False, "Unable to get the magnetometer configuration. The error was %s" % value)
    mr['md'] = value

    return (True, False)


def reset (M = None):
    '''
    Reset the values of the registers
    '''
    
    delete_mag = False
    # Create the magnetometer
    if M == None:
        delete_mag = True
        M = Init ()
        if M['error'][0]:
            return (False, "Error while creating the magnetometer. Error message was: " + M['error'][1])

    # Write the default value of the CRA register
    (status, message) = write (M, __CRA, 0xff, 0x10)
    if not status:
        return (False, "Error while writing the default value of the CRA register of the magnetometer device. Error message was: " + message)

    # Write the default value of the CRB register
    (status, message) = write (M, __CRB, 0xff, 0x20)
    if not status:
        return (False, "Error while writing the default value of the CRB register of the magnetometer device. Error message was: " + message)

    # Write the default value of the MR register
    (status, message) = write (M, __MR, 0xff, 0x0)
    if not status:
        return (False, "Error while writing the default value of the MR register of the magnetometer device. Error message was: " + message)

    # Finished test
    if delete_mag:
        del M
    
    return (True, None)


