
# Program to reset the magnetometer device

import sys
sys.path.append ("../../../lib")
import mag
import time
import numpy

(status, message) = mag.reset ()
if not status:
    print message
