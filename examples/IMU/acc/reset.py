
# Program to reset the accelerometer device

import sys
sys.path.append ("../../../lib")
import accel
import time
import numpy

(status, message) = accel.reset ()
if not status:
    print message
