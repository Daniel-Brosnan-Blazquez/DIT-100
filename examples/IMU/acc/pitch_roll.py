
# Program to print the pitch and roll values

import sys
sys.path.append ("../../../lib")
import accel
import time
import numpy
import os

accel.reset ()
A = accel.Init ()

while(1):
    time.sleep(0.25)
    os.system ("clear")
    print "\n\n\n"

    (status, values) = accel.get_normalized_values (A, "cal_matrix.npz", calibrate = False)
    if not status:
        print values
        sys.exit(0)

    x,y,z,norm = values
    (status, values) = accel.get_pitch_roll (A, "cal_matrix.npz", calibrate = False)
    if not status:
        print values
        sys.exit(0)

    p,r, m_abs = values

    print("\tx = {:7.2f} y = {:7.2f} z = {:7.2f}".format(x,y,z))

    print("\tp = {:7.2f} r = {:7.2f}".format(p,r))

    print "\t|A| = %6.3F" % numpy.sqrt (x*x + y*y + z*z)

    # (status, values) = accel.get_pitch_roll (A, "cal_matrix.npz", calibrate = True)
    # if not status:
    #     print values
    #     sys.exit(0)

    # p_c,r_c = values

    # print("\tp_c = {:7.2f} r_c = {:7.2f}".format(p_c,r_c))

