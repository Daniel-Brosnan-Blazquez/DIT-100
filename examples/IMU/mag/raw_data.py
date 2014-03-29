
# Program to print the raw data from the magnetometer device

import sys
sys.path.append ("../../../lib")
import mag
import time
import numpy
import os

mag.reset ()
M = mag.Init ()
if M['error'][0]:
    print "Error creating the magnetometer. The error was: %s" % (M['error'][1])
    sys.exit (0)

print M['gainxy']
print M['gainz']


while(1):
    time.sleep(0.25)
    os.system ("clear")
    print "\n\n\n"
    (status, x) = mag.get_raw_x (M)
    (status, y) = mag.get_raw_y (M)
    (status, z) = mag.get_raw_z (M)
    print("\t{:7.10f} {:7.10f} {:7.10f}".format(x, y, z))

    (status, values) = mag.get_normalized_values (M, "max_min.npz", calibrate = True)
    if not status:
        print values
        sys.exit (0)

    (x,y,z) = values

    print("\t{:7.10f} {:7.10f} {:7.10f}".format(x, y, z))

    print "\t|M| = %6.3F" % numpy.sqrt (x*x + y*y + z*z)
