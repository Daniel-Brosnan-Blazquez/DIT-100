
# Program to calibrate the magnetometer device

import sys
sys.path.append ("../../../lib")
import mag
import numpy
#import accel

mag.reset ()
#accel.reset ()
M = mag.Init (cal_iter = 1)

mag.print_configuration (M)

# xyz = numpy.load ("raw_mag_data.npz")['data']
# x = numpy.load ("raw_mag_data.npz")['x']
# y = numpy.load ("raw_mag_data.npz")['y']
# z = numpy.load ("raw_mag_data.npz")['z']

# data = (xyz,x,y,z)

# data = "raw_mag_data.npz"

# (status, matrix) = mag.get_params (data, save = True)
# if not status:
#     print matrix

(status, matrix) = mag.calibrate (M, fix = False, save = True, three_d = True, plot = True)
if not status:
    print matrix


