
# Program to print the degrees of the movement of the device on each
# reading

import time
import sys
import signal

sys.path.append ("../../../lib")
import gyro

def control_c_handler(signal, frame):
    print
    print 'Exit'
    sys.exit(0)

signal.signal(signal.SIGINT, control_c_handler)

# Initializing
G = gyro.Init (scale = '250dps')

if G['error'][0]:
    print 'Failed while initializing gyro'
    sys.exit(0)
    
print "Calibrating the angles"
gyro.calibrate (G)
print "End of the calibration of the angles"

print("minx: {:7.2f}, maxx: {:7.2f}, meanx {:7.2f}".format(G['minX'], G['maxX'], G['meanX']))
print("miny: {:7.2f}, maxy: {:7.2f}, meany {:7.2f}".format(G['minY'], G['maxY'], G['meanY']))
print("minz: {:7.2f}, maxz: {:7.2f}, meanz {:7.2f}".format(G['minZ'], G['maxZ'], G['meanZ']))

print "gain: " + str (G['gain'])

# Print configuration
gyro.print_configuration (G)

raw_input("Enter something to continue: ")

# Print angles
dt = 0.02
x = 0
y = 0
z = 0
last_x = 0
last_y = 0
last_z = 0
while 1==1:        
    time.sleep(dt)
    (status, (xda, yda, zda)) = gyro.isdata_available(G)

    (status, dxyz) = gyro.get_calxyz(G)

    if xda:
        x += dxyz[0]*dt;
    if yda:
        y += dxyz[1]*dt;
    if zda:
        z += dxyz[2]*dt;

    print("{:7.2f} {:7.2f} {:7.2f}".format(x, y, z))
