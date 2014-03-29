
# Program to print the configuration of the magnetometer device

import sys
sys.path.append ("../../../lib")
import mag

M = mag.Init ()
mag.print_configuration (M)
