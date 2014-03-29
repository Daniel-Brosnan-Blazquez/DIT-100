
# Program to print the degrees of the movement of the device on each
# reading

import time
import sys
import signal
import os
import commands
from matplotlib import pyplot
import numpy
sys.path.append ("../../../lib")
import gyro

PLOT = False
datax_plot = []
datay_plot = []
dataz_plot = []

def control_c_handler(signal, frame):
    if PLOT:
        pyplot.figure (1)
        pyplot.plot(datax_plot)
        pyplot.ylabel ('degrees')
        pyplot.xlabel ('readings')
        
        pyplot.figure (2)
        pyplot.plot(datay_plot)
        pyplot.ylabel ('degrees')
        pyplot.xlabel ('readings')
        
        pyplot.figure (3)
        pyplot.plot(dataz_plot)
        pyplot.ylabel ('degrees')
        pyplot.xlabel ('readings')
        
        pyplot.show()

    print
    print 'Exit'
    sys.exit(0)

signal.signal(signal.SIGINT, control_c_handler)

# Initializing
gyro.reset ()
G = gyro.Init ()
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
#(status, message) = gyro.set_fifomode (G, 'FIFO')
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

#if mode != 'FIFO':
#    print "Error while setting the FIFO mode"
#    sys.exit (0)
    
print "Calibrating the angles"
gyro.calibrate (G)
print "End of the calibration of the angles"

print("minx: {:7.2f}, maxx: {:7.2f}, meanx {:7.2f}".format(G['minX'], G['maxX'], G['meanX']))
print("miny: {:7.2f}, maxy: {:7.2f}, meany {:7.2f}".format(G['minY'], G['maxY'], G['meanY']))
print("minz: {:7.2f}, maxz: {:7.2f}, meanz {:7.2f}".format(G['minZ'], G['maxZ'], G['meanZ']))

print "gain: " + str (G['gain'])

# Print configuration
gyro.print_configuration (G)

raw_input("Enter something to continue: ")

# Print angles
dt = 0.12
x = 0
y = 0
z = 0
bf = time.time ()
while 1==1:        
    time.sleep(dt)

    os.system ("clear")

    (status, empty) = gyro.isfifo_empty(G)
    if not status:
        print empty
        sys.exit (0)

    (status, full) = gyro.isfifo_full(G)
    if not status:
        print full
        sys.exit (0)
        
    if full:
        print "\nFIFO OVERRUN\n"

#    print commands.getoutput ('i2cdump -y 1 0x6b')
    (status,level) = gyro.get_fifolevel(G)
    if not status:
        print level
        sys.exit (0)

    values = []
    while not empty and level > 0:
        (status, dxyz) = gyro.get_xyz(G)
        if not status:
            print dxyz
            sys.exit (0)

        values.append (dxyz)

        (status, empty) = gyro.isfifo_empty(G)
        if not status:
            print empty
            sys.exit (0)
            
        (status, full) = gyro.isfifo_full(G)
        if not status:
            print full
            sys.exit (0)
                
        if full:
            print "\nFIFO OVERRUN\n"

        level -= 1

    af = time.time ()
    diff = af - bf
#    print "diff time = %s" % diff
    bf = af

    j = 0.0
    k = 0.0
    l = 0.0
    if len(values) > 0:
        interval = diff/len (values)        

#     for value in values:
#         x_,y_,z_ = value
# #        print "x = %s, y = %s, z = %s" % value
#         j = x_*interval
#         k = y_*interval
#         l = z_*interval
#         if abs (j) > 0.1:
#             datax_plot.append (j)
#         if abs (k) > 0.1:
#             datay_plot.append (k)
#         if abs (l) > 0.1:
#             dataz_plot.append (l)

#        print "j = %s, k = %s, l = %s" % (j,k,l)

#    print "Number of values in FIFO = %s" % len (values)
    if len (values) > 0:
        interval = diff/len (values)
        while len (values) > 0:
            xyz = values.pop ()
            if abs (xyz[0]*interval) > 0.1:
                datax_plot.append (xyz[0]*interval)
                x += xyz[0]*interval;
            if abs (xyz[1]*interval) > 0.1:
                datay_plot.append (xyz[1]*interval)
                y += xyz[1]*interval;
            if abs (xyz[2]*interval) > 0.1:
                dataz_plot.append (xyz[2]*interval)
                z += xyz[2]*interval;

        # gyro.set_fifomode (G, 'bypass')
        # gyro.set_fifomode (G, 'FIFO')


    print("{:7.2f} {:7.2f} {:7.2f}".format(x, y, z))


