
# Program to print raw data of the accelerometer device

import sys
sys.path.append ("../../../lib")
import accel
import time
import numpy
import os

A = accel.Init ()

while(1):
    time.sleep(0.25)
    os.system ("clear")
    print "\n\n\n\n"
    (status, x) = accel.get_x (A)
    (status, y) = accel.get_y (A)
    (status, z) = accel.get_z (A)
    print("\t{:7.2f} {:7.2f} {:7.2f}".format(x, y, z))

    print "\t|A| = %6.3F" % numpy.sqrt (x*x + y*y + z*z)
