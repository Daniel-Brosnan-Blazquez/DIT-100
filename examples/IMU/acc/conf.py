
# Program to print the configuration of the accelerometer device

import sys
sys.path.append ("../../../lib")
import accel

A = accel.Init ()
accel.print_configuration (A)
