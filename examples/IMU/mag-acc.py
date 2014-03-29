
# Program to print x, y and z axis magnetic field values of the
# magnetometer device

import sys
sys.path.append ("../../lib")
import mag
import accel
import time
import numpy
import os

mag.reset ()
accel.reset ()

M = mag.Init ()
mag.enable_temp (M, True)
A = accel.Init ()

if M['error'][0]:
    print "Error creating the magnetometer. The error was: %s" % (M['error'][1])
    sys.exit (0)

if A['error'][0]:
    print "Error creating the accelerometer. The error was: %s" % (A['error'][1])
    sys.exit (0)

while(1):
    time.sleep(0.25)
    os.system ("clear")
    print "\n\n\n"

    (status, values) = accel.get_pitch_roll (A, "acc/cal_matrix.npz", calibrate = True)
    if not status:
        print values
        sys.exit(0)

    p,r,a_abs = values

    print("\tp = {:7.2f} r = {:7.2f}".format(p,r))
    print("\t|A| = {:7.2f}".format(a_abs))

    (status, values) = mag.get_heading (M, A, "mag/max_min.npz", "acc/cal_matrix.npz", calibrate = True, view_notilt = True)
    if not status:
        print values
        sys.exit(0)

    heading,M_abs = values

    print("\theading = {:7.2f}".format(heading))

    print "\t|M| = %6.3F" % M_abs

    (status, temp) = mag.get_temp(M)
    if not status:
        print temp
    print "\tTemperature = %s" % temp

