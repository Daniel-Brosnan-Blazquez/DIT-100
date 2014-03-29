
# Program to calibrate the magnetometer device getting the maximum and minimum values

import sys
sys.path.append ("../../../lib")
import mag
import numpy
#import accel


(status, values) = mag.get_min_max ("raw_mag_data.npz", save = True)
if not status:
    print values

print "minx = %s, maxx = %s" % (values[0],values[1])
print "miny = %s, maxy = %s" % (values[2],values[3])
print "minz = %s, maxz = %s" % (values[4],values[5])


