
# Program to print the configuration of the gyro device

import sys
sys.path.append ("../../../lib")
import gyro

l3g = gyro.Init ()
gyro.print_configuration (l3g)
