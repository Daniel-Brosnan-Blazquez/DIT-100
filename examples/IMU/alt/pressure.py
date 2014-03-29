
# Program to print the pitch and roll values

import sys
sys.path.append ("../../../lib")
import alt
import time
import numpy
import os

alt.reset ()
P = alt.Init ()

while(1):
    time.sleep(0.25)
    os.system ("clear")
    print "\n\n\n"

    (status, values) = alt.get_alt (P)
    if not status:
        print values
        sys.exit(0)

    p,t = values

    print("\tpressure = {:7.2f}, temperature = {:7.2f}".format(p,t))

