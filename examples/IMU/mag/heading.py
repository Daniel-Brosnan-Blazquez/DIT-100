
# Program to obtain the heading value of the magnetometer device

import sys
sys.path.append ("../../../lib")
import accel
import mag
import time
import numpy
import os

accel.reset ()
A = accel.Init ()
mag.reset ()
M = mag.Init ()

while(1):
    time.sleep(0.3)

    os.system ("clear")

    print "\n\n\n"

    (status, values) = mag.get_heading (M, A, "max_min.npz", "../acc/cal_matrix.npz", calibrate = True, view_notilt = True)
    if not status:
        print values
        sys.exit(0)

    heading,M_abs = values

    print("\theading = {:7.2f}".format(heading))

    print "\t|M| = %6.3F" % M_abs
