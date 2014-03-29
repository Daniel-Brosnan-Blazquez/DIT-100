# -*- coding: utf-8 -*-

# Collector for the IMU device
import sys
sys.path.append ("../lib")
import gyro
import accel
import mag
import alt
import common
import os
import numpy
import time

def correct_angle (angle, quadrants):
    ''' Function to modify angle to follow the accelerometer convention '''
    # Angle from accelerometer is
    #                   angle=0 
    #                      | 
    #                  3   |  0
    #                      |       
    #      angle=-90 ------+------ angle=90
    #                      |        
    #                  2   |  1    
    #                      |
    #                   angle=0

    if quadrants['quadrant'] == 0 and angle > 90:
        angle = 180 - angle
        quadrants['quadrant'] = 1
        quadrants['sign'] = -1
    elif quadrants['quadrant'] == 1 and angle < 0:
        quadrants['quadrant'] = 2
        quadrants['sign'] = -1
    elif quadrants['quadrant'] == 2 and angle < -90:
        angle = -180 - angle
        quadrants['quadrant'] = 3
        quadrants['sign'] = 1
    elif quadrants['quadrant'] == 3 and angle > 0:
        quadrants['quadrant'] = 0
        quadrants['sign'] = 1
    elif quadrants['quadrant'] == 0 and angle < 0:
        quadrants['quadrant'] = 3
    elif quadrants['quadrant'] == 3 and angle < -90:
        angle = -180 - angle
        quadrants['quadrant'] = 2
        quadrants['sign'] = -1
    elif quadrants['quadrant'] == 2 and angle > 0:
        quadrants['quadrant'] = 1
    elif quadrants['quadrant'] == 1 and angle > 90:
        angle = 180 - angle
        quadrants['quadrant'] = 0
        quadrants['sign'] = 1
    # end if

    return angle

def apply_scale (value, scales):
    ''' Function to calibrate gyro data '''

    min_scale = scales['pos_min'][0]
    min_vel = scales['pos_min'][1]
    max_scale = scales['pos_max'][0]
    max_vel = scales['pos_max'][1]
    if value < 0:
        min_scale = scales['neg_min'][0]
        min_vel = scales['neg_min'][1]
        max_scale = scales['neg_max'][0]
        max_vel = scales['neg_max'][1]
    # end if

    if min_scale == None or max_scale == None:
        return value
    # end if

    scale = min_scale + (value - min_vel)*((max_scale - min_scale)/(max_vel - min_vel))

    if scale < 1:
        scale = 1
    # end if

    return (value * scale, scale)

def get_gyro_info (params):
    ''' Function to get info from gyro '''

    bf = time.time()

    # Get the imu context
    imu = params['imu']

    # Get the gyro context
    G = imu['gyro']

    # # Get information of the fifo. Commented to reduce time
    # (status, empty) = gyro.isfifo_empty(G)
    # if not status:
    #     return (False, "Unable to check if fifo on gyro device is empty. The error was %s" % empty)
    # # end if

    diff_i2c = 0
    bf_i2c = time.time ()
    
    (status,level) = gyro.get_fifolevel(G)
    if not status:
        return (False, "Unable to get the fifo level on gyro device. The error was %s" % level)
    # end if

    # Store the level value
    params['gyro_nvalues'].append (level)

    af_i2c = time.time ()
    diff_i2c += (af_i2c - bf_i2c)

    # Check fifo overrun
    if level == 31:
        print "Fifo overrun on gyro device"
    # end if

    G['x'] = 0.0
    G['y'] = 0.0
    G['z'] = 0.0

    if level == 0:
        # Nothing to do

        # Register time
        af = time.time ()
        diff = af - bf
        if not params['con_time'][0].has_key('get_gyro_info'):
            params['con_time'][0]['get_gyro_info'] = [diff]
        else:
            params['con_time'][0]['get_gyro_info'].append (diff)
        # end if

        # Register time to perform i2c operations
        if not params['con_time'][0].has_key('i2c_gyro'):
            params['con_time'][0]['i2c_gyro'] = [diff_i2c]
        else:
            params['con_time'][0]['i2c_gyro'].append (diff_i2c)
        # end if

        return (True, None)

    params['new_gyro_data'] = True
    while level > 0:

        bf_i2c = time.time ()

        (status, xyz) = gyro.get_block_xyz(G)
        if not status:
            return (False, "Unable to get the values on gyro device. The error was %s" % xyz)
        # end if

        af_i2c = time.time ()
        diff_i2c += (af_i2c - bf_i2c)

        i = 0
        for axis in ['x', 'y', 'z']:

            value = xyz[i]

            garbage_data = False
            # Detect garbage data
            if value <= imu['gyro']['max_gd'][axis] and value >= imu['gyro']['min_gd'][axis]:
                value = 0
                garbage_data = True
            elif value <= imu['gyro']['max_cd'][axis] and value >= imu['gyro']['min_cd'][axis]:
                value = 0
                garbage_data = True
            # end if

            scale = 0
            if not garbage_data:
                apply_sc = True
                # Apply offset depending on the sign of the value
                if value < 0:
                    value = value - imu['gyro']['neg_bias'][axis]
                    if value > 0:
                        apply_sc = False
                        value = -value
                    # end if
                else:
                    value = value - imu['gyro']['pos_bias'][axis]
                # end if

                # Commented. The scales are not working properly
                # if apply_sc:
                #     (value, scale) = apply_scale (value, imu['gyro']['scales'][axis])
                # # end if
            # end if 

            # Sign conversion to aling axis with accelerometer (to
            # follow the convention of axis that is explained in the
            # AN3192 file)
            if axis == 'y':
                value = -value
            # end if                
            
            G[axis] += value*imu['gyro']['dr']
                    
            i += 1
        # end for

        level -= 1
    # end while

    # It is warranted that at least there is one value available
    #  (r->x, p->y) Set data for the actuators (y axis has the sign
    #  inverted to follow the convention)
    params['velr_gyro'] = xyz[0]
    params['velp_gyro'] = -xyz[1]

    # Register time
    af = time.time ()
    diff = af - bf
    if not params['con_time'][0].has_key('get_gyro_info'):
        params['con_time'][0]['get_gyro_info'] = [diff]
    else:
        params['con_time'][0]['get_gyro_info'].append (diff)
    # end if

    # Register time to perform i2c operations
    if not params['con_time'][0].has_key('i2c_gyro'):
        params['con_time'][0]['i2c_gyro'] = [diff_i2c]
    else:
        params['con_time'][0]['i2c_gyro'].append (diff_i2c)
    # end if

    return (True, None)

def get_acc_info (params):
    ''' Function to get info from the accelerometer '''

    bf = time.time()

    # Get the imu context
    imu = params['imu']

    # Get the accelerometer context
    A = imu['accel']

    # # Get fifo status data from accelerometer. Commented to reduce time
    # (status, aempty) = accel.isfifo_empty(A)
    # if not status:
    #     return (False, "Unable to check if fifo on the accelerometer device is empty. The error was %s" % aempty)
    # # end if
        
    diff_i2c = 0
    bf_i2c = time.time ()

    (status,alevel) = accel.get_fifolevel(A)
    if not status:
        return (False, "Unable to get the fifo level on the accelerometer device. The error was %s" % alevel)
    # end if

    # Store the level value
    params['acc_nvalues'].append (alevel)

    af_i2c = time.time ()
    diff_i2c += (af_i2c - bf_i2c)

    if alevel == 31:
        print "Fifo overrun on the accelerometer device"
    # end if

    if alevel == 0:
        # Nothing to do

        # Register time
        af = time.time ()
        diff = af - bf
        if not params['con_time'][0].has_key('get_acc_info'):
            params['con_time'][0]['get_acc_info'] = [diff]
        else:
            params['con_time'][0]['get_acc_info'].append (diff)
        # end if

        # Register time to perform i2c operations
        if not params['con_time'][0].has_key('i2c_acc'):
            params['con_time'][0]['i2c_acc'] = [diff_i2c]
        else:
            params['con_time'][0]['i2c_acc'].append (diff_i2c)
        # end if

        return (True, None)
    # end if

    values = {}
    while alevel > 0:

        bf_i2c = time.time ()

        (status, dxyz) = accel.get_block_xyz(A)
        if not status:
            return (False, "Unable to get the accelerometer values. The error was %s" % dxyz)
        # end if

        af_i2c = time.time ()
        diff_i2c += (af_i2c - bf_i2c)

        # Store data for debug porposes
        params['acc_data'].append (dxyz)

        values['x'],values['y'],values['z'] = dxyz

        for axis in ['x','y', 'z']:

            # Take out the first item
            imu['accel']['mean_data'][axis].pop (0)

            # Insert the obtained new value        
            imu['accel']['mean_data'][axis].append (values[axis])
        # end for

        alevel -= 1
    # end while

    values['x'] = numpy.mean (imu['accel']['mean_data']['x'])
    values['y'] = numpy.mean (imu['accel']['mean_data']['y'])
    values['z'] = numpy.mean (imu['accel']['mean_data']['z'])

    x_a = values['x']
    y_a = values['y']
    z_a = values['z']

    norm = numpy.sqrt (x_a**2 + y_a**2 + z_a**2)
    if norm == 0:
        print "Unable to get the normalized values. The norm is equal to 0"
    # end if

    x_a = x_a/norm
    y_a = y_a/norm
    z_a = z_a/norm

    (status, vals) = accel.get_pitch_roll (A, normalize = False, values = (x_a,y_a,z_a,norm))
    if not status:
        return (False, "Unable to get pitch and roll values from accelerometer. The error was %s" % vals)
    # end if

    p,r,n = vals

    imu['accel']['p'] = p
    imu['accel']['r'] = r

    # Register time
    af = time.time ()
    diff = af - bf
    if not params['con_time'][0].has_key('get_acc_info'):
        params['con_time'][0]['get_acc_info'] = [diff]
    else:
        params['con_time'][0]['get_acc_info'].append (diff)
    # end if

    # Register time to perform i2c operations
    if not params['con_time'][0].has_key('i2c_acc'):
        params['con_time'][0]['i2c_acc'] = [diff_i2c]
    else:
        params['con_time'][0]['i2c_acc'].append (diff_i2c)
    # end if

    return (True, None)

def get_mag_info (params):
    ''' Function to get info from the magnetometer '''    

    bf = time.time()

    # Get the imu context
    imu = params['imu']

    # Get the gyro context
    M = imu['mag']

    bf_i2c = time.time ()

    # Get raw data from the magnetometer
    (status, values) = mag.get_block_xyz (M)
    if not status:
        return (False, "Unable to get the heading. The error was %s" % values)
    # end if

    af_i2c = time.time ()
    diff_i2c = (af_i2c - bf_i2c)

    x,y,z = values

    (status, values) = mag.tilt_heading ((imu['p'],imu['r']), mag_values = values, M = M, cal_mag=M, calibrate=True)
    if not status:
        return (False, "Unable to get the heading. The error was %s" % values)
    # end if

    M['h'],M['M_abs'] = values

    # Register time
    af = time.time ()
    diff = af - bf
    if not params['con_time'][0].has_key('get_mag_info'):
        params['con_time'][0]['get_mag_info'] = [diff]
    else:
        params['con_time'][0]['get_mag_info'].append (diff)
    # end if

    # Register time to perform i2c operations
    if not params['con_time'][0].has_key('i2c_mag'):
        params['con_time'][0]['i2c_mag'] = [diff_i2c]
    else:
        params['con_time'][0]['i2c_mag'].append (diff_i2c)
    # end if

    return (True, None)

def get_alt_info (params):
    ''' Function to get info from the altimeter '''    

    bf = time.time()

    # Get the imu context
    imu = params['imu']

    # Get the altimeter context
    P = imu['alt']

    bf_i2c = time.time ()

    # Calculate altitude
    (status, values) = alt.get_alt_ai (P)
    if not status:
        return (False, "Unable to get the information from IMU device. The error was %s" % values)

    af_i2c = time.time ()
    diff_i2c = (af_i2c - bf_i2c)

    P['a'],P['t'],P['p'] = values

    # Store data for debug purposes
    params['alt_data'].append (P['a'])

    # Register time
    af = time.time ()
    diff = af - bf
    if not params['con_time'][0].has_key('get_alt_info'):
        params['con_time'][0]['get_alt_info'] = [diff]
    else:
        params['con_time'][0]['get_alt_info'].append (diff)
    # end if

    # Register time to perform i2c operations
    if not params['con_time'][0].has_key('i2c_alt'):
        params['con_time'][0]['i2c_alt'] = [diff_i2c]
    else:
        params['con_time'][0]['i2c_alt'].append (diff_i2c)
    # end if

    return (True, None)


def get_info (params):
    '''
    Get information from IMU device
    params: dictionary with the context needed
    '''

    bf = time.time()

    # # Check if params is of type dictionary (commented to reduce time)
    # if type (params) != dict:
    #     return (False, "Unable to get the information from IMU device. The params parameter is not of type dict, is of type %s" % type (params))
    # # end if

    # # Check if imu key is inside the params (commented to reduce time)
    # keys = ['imu']
    # if not common.check_keys (keys, params):
    #     return (False, "Unable to get the information from IMU device. The params parameter does not contain the necesary key value 'imu'")
    # # end if

    # # Check if gyro, accel, mag and alt keys are inside the params (commented to reduce time)
    # keys = ['gyro', 'accel', 'mag', 'alt']
    # if not common.check_keys (keys, imu):
    #     return (False, "Unable to get the information from IMU device. The params parameter does not contain the necesary key values 'gyro', 'accel', 'mag' and 'alt'")
    # # end if
    
    # Get the imu context
    imu = params['imu']
    G = imu['gyro']
    A = imu['accel']
    M = imu['mag']
    P = imu['alt']

    # Variable to indicate if there is new data available from gyro
    params['new_gyro_data'] = False

    # Get info from the gyro device
    (status, message) = get_gyro_info (params)
    if not status:
        return (False, "Unable to get info from the gyro device. The error was %s" % message)
    # end if

    # Set pitch, roll and heading with the data from the gyro
    imu['r'] += G['x']*imu['r_quadrants']['sign']
    imu['r_tocheck'] += G['x']*imu['r_quadrants_tocheck']['sign']
    imu['p'] += G['y']*imu['p_quadrants']['sign']
    imu['p_tocheck'] += G['y']*imu['p_quadrants_tocheck']['sign']
    imu['h'] += G['z']

    # Correct angles following the convention for angles from the
    # accelerometer
    imu['p'] = correct_angle (imu['p'], imu['p_quadrants'])
    imu['p_tocheck'] = correct_angle (imu['p_tocheck'], imu['p_quadrants_tocheck'])

    imu['r'] = correct_angle (imu['r'], imu['r_quadrants'])
    imu['r_tocheck'] = correct_angle (imu['r_tocheck'], imu['r_quadrants_tocheck'])


    # Get info from the acelerometer device
    (status, message) = get_acc_info (params)
    if not status:
        return (False, "Unable to get info from the acclerometer device. The error was %s" % message)
    # end if

    # Calibrate pitch and roll from gyro data with the data from the accelerometer (COMPLEMENTARY FILTER)
    imu['p'] = imu['p']*params['comp'] + (1-params['comp'])*imu['accel']['p']
    imu['r'] = imu['r']*params['comp'] + (1-params['comp'])*imu['accel']['r']

    # Get info from the magnetometer device
    (status, message) = get_mag_info (params)
    if not status:
        return (False, "Unable to get info from the magnetometer device. The error was %s" % message)
    # end if

    # Get info grom the altimeter device
    (status, message) = get_alt_info (params)
    if not status:
        return (False, "Unable to get info from the altimeter device. The error was %s" % message)
    # end if

    # Get temperatures
    # (status, temp) = gyro.get_temp (G)
    # if not status:
    #     return (False, "Unable to get the information from IMU device. The error was %s" % temp)
    # end if

    # temp = 0
    # imu['tg'] = temp
    # G['t'] = temp

    # (status, temp) = mag.get_temp (M)
    # if not status:
    #     return (False, "Unable to get the information from IMU device. The error was %s" % temp)
    # end if

    # temp = 0
    # imu['tam'] = temp
    # A['t'] = temp
    # M['t'] = temp

    # Register time
    af = time.time ()
    diff = af - bf
    if not params['con_time'][0].has_key('get_info_imu'):
        params['con_time'][0]['get_info_imu'] = [diff]
    else:
        params['con_time'][0]['get_info_imu'].append (diff)
    # end if

    return (True, None)

def get_file (__file):
    if os.path.exists ("/etc/mini-brain/calibration-files/%s" % __file):
        return "/etc/mini-brain/calibration-files/%s" % __file
    if os.path.exists ("../%s" % __file):
        return "../collectors/%s" % __file
    return __file

def get_conf (params):
    '''
    Get configuration of each device on the IMU
    params: dictionary with the context needed
    '''

    if type (params) != dict:
        return (False, "Unable to get the information from IMU device. The params parameter is not of type dict, is of type %s" % type (params))

    keys = ['imu']
    if not common.check_keys (keys, params):
        return (False, "Unable to get the information from IMU device. The params parameter does not contain the necesary key value 'imu'")

    imu = params['imu']

    keys = ['gyro', 'accel', 'mag', 'alt']
    if not common.check_keys (keys, imu):
        return (False, "Unable to get the information from IMU device. The params parameter does not contain the necesary key values 'gyro', 'accel', 'mag' and 'alt'")
    
    G = imu['gyro']
    A = imu['accel']
    M = imu['mag']
    P = imu['alt']

    # Get configuration
    for dev in [(G,gyro), (A,accel), (M,mag), (P,alt)]:
        ctx = dev[0]
        mod = dev[1]
        (status, message) = mod.get_conf (ctx)
        if not status:
            return (False, "Unable to get the information from IMU device. The error was %s" % message)
        
    return (True, None)


def init_gyro (imu):
    ''' Function to initialize the gyro device '''
    
    # Reset the registers of the device
    gyro.reset ()

    # Create the gyro context
    G = gyro.Init (scale = '500dps')
    if G['error'][0]:
        return (False, "Unable to initialize the IMU device. The gyro context could not be created. The error was %s" % G['error'][1])
    # end if
    
    # Enable fifo with stream mode
    (status, message) = gyro.enable_fifo (G, True)
    if not status:
        return (False, "Unable to set fifo on gyro device. The error was %s" % message)
    # end if

    (status, enable) = gyro.isenabled_fifo (G)
    if not status:
        return (False, "Unable to set fifo on gyro device. The error was %s" % enable)
    # end if
    
    if not enable:
        return (False, "Unable to set fifo on gyro device. The funtion to enable fifo succeeded but the fifo was not set")
    # end if

    (status, message) = gyro.set_fifomode (G, 'stream')
    if not status:
        return (False, "Unable to set fifo mode to stream on gyro device. The error was %s" % message)
    # end if

    (status, mode) = gyro.get_fifomode (G)
    if not status:
        return (False, "Unable to set fifo mode to stream on gyro device. The error was %s" % mode)
    # end if

    if mode != 'stream':
        return (False, "Unable to set fifo mode to stream on gyro device. The funtion to set the fifo mode succeeded but the mode was not set")
    # end if

    imu['gyro'] = G

    # Get calibration data
    cal_file = get_file ("cal_gyro.npz")
    cal_data = numpy.load (cal_file)
    imu['gyro']['pos_bias'] = {}
    imu['gyro']['neg_bias'] = {}
    imu['gyro']['min_gd'] = {}
    imu['gyro']['max_gd'] = {}
    [imu['gyro']['pos_bias']['x'], imu['gyro']['pos_bias']['y'], imu['gyro']['pos_bias']['z']] = cal_data['pos_bias']
    [imu['gyro']['neg_bias']['x'], imu['gyro']['neg_bias']['y'], imu['gyro']['neg_bias']['z']] = cal_data['neg_bias']
    [imu['gyro']['min_gd']['x'], imu['gyro']['max_gd']['x']] = cal_data['garbage_data'][0]
    [imu['gyro']['min_gd']['y'], imu['gyro']['max_gd']['y']] = cal_data['garbage_data'][1]
    [imu['gyro']['min_gd']['z'], imu['gyro']['max_gd']['z']] = cal_data['garbage_data'][2]
    imu['gyro']['min_cd'] = {}
    imu['gyro']['max_cd'] = {}
    [imu['gyro']['min_cd']['x'], imu['gyro']['max_cd']['x']] = cal_data['correct_data'][0]
    [imu['gyro']['min_cd']['y'], imu['gyro']['max_cd']['y']] = cal_data['correct_data'][1]
    [imu['gyro']['min_cd']['z'], imu['gyro']['max_cd']['z']] = cal_data['correct_data'][2]
    imu['gyro']['scales'] = {}
    i = 0
    for axis in ['x', 'y', 'z']:
        imu['gyro']['scales'][axis] = {}
        imu['gyro']['scales'][axis]['pos_min'] = cal_data['group_scales'][i]['pos_min_scale']
        imu['gyro']['scales'][axis]['pos_max'] = cal_data['group_scales'][i]['pos_max_scale']
        imu['gyro']['scales'][axis]['neg_min'] = cal_data['group_scales'][i]['neg_min_scale']
        imu['gyro']['scales'][axis]['neg_max'] = cal_data['group_scales'][i]['neg_max_scale']
        
        i += 1
    # end for

    # Get the data rate
    (status,drbw) = gyro.get_drbw (G)
    if not status:
        return (False, "Unable to get the data rate from gyro device. The error was %s" % drbw)
    # end if

    freq, bw = drbw
    imu['gyro']['dr'] = 1.0/freq

    return (True, None)

def init_acc (imu):
    ''' Function to initialize the accelerometer device '''

    # Reset the registers of the device
    accel.reset ()

    # Create the accelerometer context
    A = accel.Init (dr = 100, scale = '4G')
    if A['error'][0]:
        return (False, "Unable to initialize the IMU device. The accelerometer context could not be created. The error was %s" % A['error'][1])
    # end if

    # Get the data rate value which indicates the time between data updates
    (status,freq) = accel.get_dr (A)
    if not status:
        return (False, "Unable to initialize the IMU device. The accelerometer context could not be created. The error was %s" % freq)
    # end if

    dr = 1.0/freq

    # Enable fifo with stream mode
    (status, message) = accel.enable_fifo (A, True)
    if not status:
        return (False, "Unable to initialize the IMU device. The accelerometer context could not be created. The error was %s" % message)
    # end if

    (status, enable) = accel.isenabled_fifo (A)
    if not status:
        return (False, "Unable to initialize the IMU device. The accelerometer context could not be created. The error was %s" % enable)
    # end if

    if not enable:
        return (False, "Unable to set fifo on accelerometer device. The funtion to enable fifo succeeded but the fifo was not set")
    # end if

    (status, message) = accel.set_fifomode (A, 'stream')
    if not status:
        return (False, "Unable to set fifo mode to stream on the accelerometer device. The error was %s" % message)
    # end if

    (status, mode) = accel.get_fifomode (A)
    if not status:
        return (False, "Unable to set fifo on accelerometer device. The error was %s" % mode)
    # end if

    if mode != 'stream':
        return (False, "Unable to set fifo on accelerometer device. The funtion to set the fifo mode succeeded but the mode was not set")
    # end if

    imu['accel'] = A

    # Storages for calibrate the accelerometer data
    imu['accel']['mean_data'] = {}
    imu['accel']['mdata_length'] = 20

    return (True, None)

def init_mag (imu):
    ''' Function to initialize the magnetometer device '''
    M = mag.Init ()
    if M['error'][0]:
        return (False, "Unable to initialize the IMU device. The magnetometer context could not be created. The error was %s" % M['error'][1])
    # end if
    
    (status, message) = mag.enable_temp (M, True)
    if not status:
        return (False, "Unable to initialize the IMU device. The error was %s" % message)
    # end if

    imu['mag'] = M

    cal_file = get_file ("max_min.npz")

    data = numpy.load (cal_file)
    imu['mag']['minx'] = data['minx']
    imu['mag']['maxx'] = data['maxx']
    imu['mag']['miny'] = data['miny']
    imu['mag']['maxy'] = data['maxy']
    imu['mag']['minz'] = data['minz']
    imu['mag']['maxz'] = data['maxz']

    return (True, None)

def init_alt (imu):
    ''' Funtion to initialize the altimeter device '''
    P = alt.Init ()
    if P['error'][0]:
        return (False, "Unable to initialize the IMU device. The altimeter context could not be created. The error was %s" % P['error'][1])
    # end if
    
    imu['alt'] = P

    return (True, None)

def initialize (params):
    
    if type (params) != dict:
        return (False, "Unable to initialize the IMU device. The params parameter is not of type dict, is of type %s" % type (params))
    # end if

    # Create the imu context
    params['imu'] = {}
    imu = params['imu']
    
    # Initialize gyro, accel, mag and alt devices
    (status, message) = init_gyro (imu)
    if not status:
        return (False, "Unable to initialize the gyro device. The error was %s" % message)
    # end if
    (status, message) = init_acc (imu)
    if not status:
        return (False, "Unable to initialize the accelerometer device. The error was %s" % message)
    # end if
    (status, message) = init_mag (imu)
    if not status:
        return (False, "Unable to initialize the magnetometer device. The error was %s" % message)
    # end if
    (status, message) = init_alt (imu)
    if not status:
        return (False, "Unable to initialize the altimeter device. The error was %s" % message)
    # end if


    # Initialize pitch, roll and heading values
    ## PITCH AND ROLL
    A = imu['accel']
    (status, dxyz) = accel.get_xyz(A)
    if not status:
        return (False, "Unable to get the accelerometer data. The error was %s" % dxyz)
    # end if
    
    x,y,z = dxyz

    norm = numpy.sqrt (x**2 + y**2 + z**2)
    if norm == 0:
        x = 0
        y = 0
        z = 0
        print "Unable to get the normalized values. The norm is equal to 0"
    else:
        x = x/norm
        y = y/norm
        z = z/norm
    # end if
        
    # Initialize mean data vectors for the accelerometer with the
    # first value
    imu['accel']['mean_data']['x'] = [x]*imu['accel']['mdata_length']
    imu['accel']['mean_data']['y'] = [y]*imu['accel']['mdata_length']
    imu['accel']['mean_data']['z'] = [z]*imu['accel']['mdata_length']
    
    (status, vals) = accel.get_pitch_roll (A, normalize = False, values = (x,y,z,norm))
    if not status:
        return (False, "Unable to get the pitch and roll values. The error was %s" % vals)
    # end if

    p,r,n = vals

    ## HEADING
    # Calculate heading
    M = imu['mag']
    (status, values) = mag.get_heading (M, A, get_file ("max_min.npz"), get_file ("cal_matrix.npz"), calibrate = True)
    if not status:
        return (False, "Unable to get heading. The error was %s" % values)
    # end if

    h,M_abs = values

    # Final pitch, heading and roll 
    imu['p'] = p
    imu['accel']['p'] = p
    imu['p_tocheck'] = p
    imu['r'] = r
    imu['accel']['r'] = r
    imu['r_tocheck'] = r
    imu['h'] = h

    # Get position of the drone
    z_up = True
    if z < -0.8:
        z_up = False
        
    # Set quadrant for the pitch angle
    if p >= 0 and z_up == True:
        imu['p_quadrants'] = {'quadrant': 0, 'sign': 1}
        imu['p_quadrants_tocheck'] = {'quadrant': 0, 'sign': 1}
    elif p >= 0 and z_up == False:
        imu['p_quadrants'] = {'quadrant': 1, 'sign': -1}
        imu['p_quadrants_tocheck'] = {'quadrant': 1, 'sign': -1}
    elif p < 0 and z_up == False:
        imu['p_quadrants'] = {'quadrant': 2, 'sign': -1}
        imu['p_quadrants_tocheck'] = {'quadrant': 2, 'sign': -1}
    else:
        imu['p_quadrants'] = {'quadrant': 3, 'sign': 1}
        imu['p_quadrants_tocheck'] = {'quadrant': 3, 'sign': 1}
    # end if

    # Set quadrant for the roll angle
    if r >= 0 and z_up == True:
        imu['r_quadrants'] = {'quadrant': 0, 'sign': 1}
        imu['r_quadrants_tocheck'] = {'quadrant': 0, 'sign': 1}
    elif r >= 0 and z_up == False:
        imu['r_quadrants'] = {'quadrant': 1, 'sign': -1}
        imu['r_quadrants_tocheck'] = {'quadrant': 1, 'sign': -1}
    elif r < 0 and z_up == False:
        imu['r_quadrants'] = {'quadrant': 2, 'sign': -1}
        imu['r_quadrants_tocheck'] = {'quadrant': 2, 'sign': -1}
    else:
        imu['r_quadrants'] = {'quadrant': 3, 'sign': 1}
        imu['r_quadrants_tocheck'] = {'quadrant': 3, 'sign': 1}
    # end if
        
    # Complementary filter
    imu['comp'] = params['comp']
    
    return (True, None)
