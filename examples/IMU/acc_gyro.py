
# Program to print the degrees of the movement of the device on each
# reading

import time
import sys
import signal
import os
import commands
from matplotlib import pyplot
import numpy
sys.path.append ("../../lib")
import gyro
import accel

FIGWIDTH = 20
FIGHEIGTH = 10

def apply_scale (value, scales):

    min_scale = scales['pos_min'][0]
    min_vel = scales['pos_min'][1]
    max_scale = scales['pos_max'][0]
    max_vel = scales['pos_max'][1]
    if value < 0:
        min_scale = scales['neg_min'][0]
        min_vel = scales['neg_min'][1]
        max_scale = scales['neg_max'][0]
        max_vel = scales['neg_max'][1]

    if min_scale == None or max_scale == None:
        return value

    scale = min_scale + (value - min_vel)*((max_scale - min_scale)/(max_vel - min_vel))

    if scale < 1:
        scale = 1

    return (value * scale, scale)


deg = [{},{}]
degacum = [{},{}]
diffvel = [{},{}]
vel = [{},{}]
bf_value = [{},{}]

# 0 calibrated data
# 1 raw data
for i in xrange (2):
    deg[i]['x'] = []
    deg[i]['y'] = []
    deg[i]['z'] = []
    
    degacum[i]['x'] = []
    degacum[i]['y'] = []
    degacum[i]['z'] = []
    
    diffvel[i]['x'] = []
    diffvel[i]['y'] = []
    diffvel[i]['z'] = []
    
    vel[i]['x'] = []
    vel[i]['y'] = []
    vel[i]['z'] = []

    bf_value[i]['x'] = 0
    bf_value[i]['y'] = 0
    bf_value[i]['z'] = 0

scales_values = {}
scales_values['x'] = []
scales_values['y'] = []
scales_values['z'] = []

accel_values = {}
accel_values['x'] = []
accel_values['y'] = []
accel_values['z'] = []

degacum_withoutupdates = {}
degacum_withoutupdates['x'] = []
degacum_withoutupdates['y'] = []
degacum_withoutupdates['z'] = []

accel_norms = []

ctrl = {}
ctrl['STOP'] = False
ctrl['NEW_ITER'] = False
def control_c_handler(signal, frame):

    raw = raw_input ("Enter c to cancel the program or p to plot the data:")
    if raw == 'c':
        ctrl['STOP'] = True
        return

    if raw == 'p':
        j = 1
        for axis in ['x','y','z']:
            fig = pyplot.figure (j, figsize = (FIGWIDTH,FIGHEIGTH))
            j += 1
            fig.suptitle ("Data of axis %s" % axis)
            s1 = fig.add_subplot (611)
            p1, = s1.plot(deg[1][axis])
            p2, = s1.plot(deg[0][axis])
            s1.legend ([p1,p2], ["Raw data", "Calibrated data"])
            s1.set_title ("degrees on each instant")
            
            s2 = fig.add_subplot (612)
            p1, = s2.plot(degacum_withoutupdates[axis])
            p2, = s2.plot(degacum[0][axis])
            s2.legend ([p1,p2], ["data without updates", "updated data"])
            s2.set_title ("Degrees acumulated")
            
            s3 = fig.add_subplot (613)
            p1, = s3.plot(vel[1][axis])
            p2, = s3.plot(vel[0][axis])
            s3.legend ([p1,p2], ["Raw data", "Calibrated data"])
            s3.set_title ("Velocity on each instant")
            
            s4 = fig.add_subplot (614)
            s4.plot(scales_values[axis])
            s4.set_title ("Scales")

            s5 = fig.add_subplot (615)
            s5.plot(accel_values[axis])
            s5.set_title ("Accelerometer values")

            s6 = fig.add_subplot (616)
            s6.plot(accel_norms)
            s6.set_title ("Norm of the accelerometer values")
            
        pyplot.show ()
            
        pyplot.close (1)
        pyplot.close (2)
        pyplot.close (3)

    ctrl['NEW_ITER'] = True

    return


signal.signal(signal.SIGINT, control_c_handler)

# Initializing the gyro
gyro.reset ()
G = gyro.Init (scale = '500dps')
if G['error'][0]:
    print 'Failed while initializing gyro'
    sys.exit(0)

(status, message) = gyro.enable_fifo (G, True)
if not status:
    print message
    sys.exit (0)

(status, enable) = gyro.isenabled_fifo (G)
if not status:
    print enable
    sys.exit (0)
    
if not enable:
    print "Error while enabling FIFO"
    sys.exit (0)

(status, message) = gyro.set_fifomode (G, 'stream')
if not status:
    print message
    sys.exit (0)

(status, mode) = gyro.get_fifomode (G)
if not status:
    print message
    sys.exit (0)

if mode != 'stream':
    print "Error while setting the stream mode"
    sys.exit (0)

# Initializing
accel.reset ()
A = accel.Init (dr = 100, scale = '4G')
if A['error'][0]:
    print 'Failed while initializing the accelerometer'
    sys.exit(0)

# Get the data rate value which indicates the time between data updates
(status,(freq)) = accel.get_dr (A)
dr = 1.0/freq

(status, message) = accel.enable_fifo (A, True)
if not status:
    print message
    sys.exit (0)
        
(status, enable) = accel.isenabled_fifo (A)
if not status:
    print enable
    sys.exit (0)
    
if not enable:
    print "Error while enabling FIFO"
    sys.exit (0)

(status, message) = accel.set_fifomode (A, 'stream')
if not status:
    print message
    sys.exit (0)

(status, mode) = accel.get_fifomode (A)
if not status:
    print message
    sys.exit (0)

if mode != 'stream':
    print "Error while setting the stream mode"
    sys.exit (0)


# Get calibration data
cal_file = "/etc/mini-brain/calibration-files/cal_gyro.npz"
cal_data = numpy.load (cal_file)
pos_bias = {}
neg_bias = {}
min_gd = {}
max_gd = {}
[pos_bias['x'], pos_bias['y'], pos_bias['z']] = cal_data['pos_bias']
[neg_bias['x'], neg_bias['y'], neg_bias['z']] = cal_data['neg_bias']
[min_gd['x'], max_gd['x']] = cal_data['garbage_data'][0]
[min_gd['y'], max_gd['y']] = cal_data['garbage_data'][1]
[min_gd['z'], max_gd['z']] = cal_data['garbage_data'][2]
min_cd = {}
max_cd = {}
[min_cd['x'], max_cd['x']] = cal_data['correct_data'][0]
[min_cd['y'], max_cd['y']] = cal_data['correct_data'][1]
[min_cd['z'], max_cd['z']] = cal_data['correct_data'][2]
scales = {}
i = 0
for axis in ['x', 'y', 'z']:
    scales[axis] = {}
    scales[axis]['pos_min'] = cal_data['group_scales'][i]['pos_min_scale']
    scales[axis]['pos_max'] = cal_data['group_scales'][i]['pos_max_scale']
    scales[axis]['neg_min'] = cal_data['group_scales'][i]['neg_min_scale']
    scales[axis]['neg_max'] = cal_data['group_scales'][i]['neg_max_scale']
    
    i += 1

dr = 1.0/95.0

mean_data = {}
values = {}
length = 20

iterations_between_prints = 5
iterations = 0
bf_time = time.time ()

initialized = False

while not ctrl['STOP']:        


    if ctrl['NEW_ITER']:
        ctrl['NEW_ITER'] = False
        deg = [{},{}]
        degacum = [{},{}]
        diffvel = [{},{}]
        vel = [{},{}]
        bf_value = [{},{}]
        
        for i in xrange (2):
            deg[i]['x'] = []
            deg[i]['y'] = []
            deg[i]['z'] = []
            
            degacum[i]['x'] = []
            degacum[i]['y'] = []
            degacum[i]['z'] = []
            
            diffvel[i]['x'] = []
            diffvel[i]['y'] = []
            diffvel[i]['z'] = []
            
            vel[i]['x'] = []
            vel[i]['y'] = []
            vel[i]['z'] = []
            
            bf_value[i]['x'] = 0
            bf_value[i]['y'] = 0
            bf_value[i]['z'] = 0

    (status, empty) = gyro.isfifo_empty(G)
    if not status:
        print empty
        sys.exit (0)

    (status, full) = gyro.isfifo_full(G)
    if not status:
        print full
        sys.exit (0)
        
    if full:
        print "\n Gyro FIFO OVERRUN\n"

    (status,level) = gyro.get_fifolevel(G)
    if not status:
        print level
        sys.exit (0)

    (status, aempty) = accel.isfifo_empty(A)
    if not status:
        print aempty
        sys.exit (0)

    (status, afull) = accel.isfifo_full(A)
    if not status:
        print afull
        sys.exit (0)
        
    if afull:
        print "\n Accel FIFO OVERRUN\n"

    (status,alevel) = accel.get_fifolevel(A)
    if not status:
        print alevel
        sys.exit (0)
        
    while not empty and level > 0:
        (status, xyz) = gyro.get_xyz(G)
    
        if not status:
            print xyz
            sys.exit (0)

        i = 0
        for axis in ['x', 'y', 'z']:
            value = xyz[i]

            diff = 0.0
            garbage_data = False
            # Detect garbage data
            if value <= max_gd[axis] and value >= min_gd[axis]:
                value = 0
                garbage_data = True
            elif value <= max_cd[axis] and value >= min_cd[axis]:
                value = 0
                garbage_data = True

            scale = 0
            if not garbage_data:
                apply_sc = True
                # Apply displacement depending on the sign of the value
                if value < 0:
                    value = value - neg_bias[axis]
                    if value > 0:
                        apply_sc = False
                        value = -value
                else:
                    value = value -pos_bias[axis]

                if apply_sc:
                    (value, scale) = apply_scale (value, scales[axis])

            scales_values[axis].append (scale)

            # Calibrated data
            j = 0
            sum_d = 0
            if len (degacum[j][axis]) > 0:
                sum_d = degacum[j][axis][len (degacum[j][axis])-1]
            sum_d += value*dr;
            vel[j][axis].append (value)
            diffvel[j][axis].append (value-bf_value[j][axis])
            bf_value[j][axis] = value
            degacum[j][axis].append (sum_d)
            sum_d = 0
            if len (degacum_withoutupdates[axis]) > 0:
                sum_d = degacum_withoutupdates[axis][len (degacum_withoutupdates[axis])-1]
            sum_d += value*dr;

            degacum_withoutupdates[axis].append (sum_d)
            deg[j][axis].append (value*dr)

            # Raw data
            value = xyz[i]
            j = 1
            sum_d = 0
            if len (degacum[j][axis]) > 0:
                sum_d = degacum[j][axis][len (degacum[j][axis])-1]
            sum_d += value*dr;
            vel[j][axis].append (value)
            diffvel[j][axis].append (value-bf_value[j][axis])
            bf_value[j][axis] = value
            degacum[j][axis].append (sum_d)
            deg[j][axis].append (value*dr)

            i += 1

            level -= 1

    while not aempty and alevel > 0:
        (status, dxyz) = accel.get_xyz(A)
        if not status:
            print dxyz
            sys.exit (0)

        values['x'],values['y'],values['z'] = dxyz

        for axis in ['x','y', 'z']:
            if axis not in mean_data.keys():
                mean_data[axis] = [values[axis]]*length

            # Take out the first item
            mean_data[axis].pop (0)

            # Insert the obtained new value        
            mean_data[axis].append (values[axis])
            values[axis] = numpy.mean (mean_data[axis])
            accel_values[axis].append (values[axis])
    
        alevel -= 1


    if not initialized:
        x = values['x']
        y = values['y']
        z = values['z']

        norm = numpy.sqrt (x**2 + y**2 + z**2)
        if norm == 0:
            print "Unable to get the normalized values. The norm is equal to 0"
        x = x/norm
        y = y/norm
        z = z/norm

        (status, vals) = accel.get_pitch_roll (A, normalize = False, values = (x,y,z,norm))
        if not status:
            print vals

        p,r,n = vals

        for i in xrange (2):
            degacum[i]['x'].append (r)
            degacum_withoutupdates['x'].append (degacum_withoutupdates['x'][len (degacum_withoutupdates['x'])-1])
            degacum[i]['y'].append (p)
            degacum_withoutupdates['y'].append (degacum_withoutupdates['y'][len (degacum_withoutupdates['y'])-1])

        initialized = True
        

    iterations += 1

    af_time = time.time ()
    diff = af_time - bf_time
    bf_time = af_time

    if iterations == iterations_between_prints:
        print "elapsed time = %s" % (diff)
        x = degacum[0]['x'][len (degacum[0]['x'])-1]
        y = degacum[0]['y'][len (degacum[0]['y'])-1]
        z = degacum[0]['z'][len (degacum[0]['z'])-1]
        
        print("gyro -> x = {:7.2f} y = {:7.2f} z = {:7.2f}".format(x,y,z))

        x_a = values['x']
        y_a = values['y']
        z_a = values['z']

        print("accel -> x = {:7.2f} y = {:7.2f} z = {:7.2f}".format(x_a,y_a,z_a))

        norm = numpy.sqrt (x_a**2 + y_a**2 + z_a**2)
        if norm == 0:
            print "Unable to get the normalized values. The norm is equal to 0"

        accel_norms.append (norm)

        x_a = x_a/norm
        y_a = y_a/norm
        z_a = z_a/norm

        (status, vals) = accel.get_pitch_roll (A, normalize = False, values = (x_a,y_a,z_a,norm))
        if not status:
            print vals

        p,r,n = vals

        p = -p

        # Update the gyro data if the difference between gyro data and
        # accel data is bigger than 10 degress
        if abs (p-y) > 10:
            degacum[0]['y'].append (p)
            degacum_withoutupdates['y'].append (degacum_withoutupdates['y'][len (degacum_withoutupdates['y'])-1])

        if abs (r-x) > 10:
            degacum[0]['x'].append (r)            
            degacum_withoutupdates['x'].append (degacum_withoutupdates['x'][len (degacum_withoutupdates['x'])-1])


        print("accel -> p = {:7.2f} r = {:7.2f}".format(p,r))

        iterations = 0


    # x = degacum[0]['x'][len (degacum[0]['x'])-1]
    # y = degacum[0]['y'][len (degacum[0]['y'])-1]
    # z = degacum[0]['z'][len (degacum[0]['z'])-1]
    
    # print("gyro -> x = {:7.2f} y = {:7.2f} z = {:7.2f}".format(x,y,z))
    # print("accel -> x = {:7.2f} y = {:7.2f} z = {:7.2f}".format(values['x'],values['y'],values['z']))
    # iterations = 0
