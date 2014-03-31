from matplotlib import pyplot
import numpy
from operator import add
from operator import sub

# File from get data
file = "time_code_reduction6.npz"

# Get data from numpy file
data = numpy.load (file)['con_time'][0]

display = []

# Time get gyro info, i2c operations and differences between both
tag1 = 'get_gyro_info'
gyro = data[tag1]
tag2 = 'i2c_gyro'
i2c_gyro = data[tag2]
tag3 = 'Computation time'
total_i2c = map (sub, gyro, i2c_gyro)
display.append (("Gyroscope collector data",[gyro, i2c_gyro, total_i2c], ["Time to get info from gyro", "Time of i2c operations", tag3], 3))

# Time get acc info, i2c operations and differences between both
tag1 = 'get_acc_info'
acc = data[tag1]
tag2 = 'i2c_acc'
i2c_acc = data[tag2]
tag3 = 'Computation time'
total_i2c = map (sub, acc, i2c_acc)
display.append (("Accelerometer collector data",[acc, i2c_acc, total_i2c], ["Time to get info from the accelerometer", "Time of i2c operations", tag3], 3))

# Time level3
tag1 = 'act_level3'
level3 = data[tag1]
tag2 = 'i2c_act_level3'
i2c_act_level3 = data[tag2]
tag3 = 'Computation time'
total_i2c = map (sub, level3, i2c_act_level3)
display.append (("Actuator data", [level3, i2c_act_level3, total_i2c], ["Time to perform the control of the power of the motors", "Time of i2c operations", tag3], 3))

# Global time
tag1 = 'mini-brain'
mini_brain = data[tag1]
tag2 = 'i2c'
i2c = map (add, i2c_gyro, i2c_acc)
i2c = map (add, i2c, i2c_act_level3)
tag3 = 'Computation time'
comp_time = map (sub, mini_brain, i2c)
display.append (("Mini brain data",[mini_brain, i2c, comp_time], ["Time of each main iteration", "Time of i2c operations on each iteration", tag3], 3))

# Time get info imu
tag1 = 'get_info_imu'
imu = data[tag1]
tag2 = 'i2c'
i2c = map (add, i2c_gyro, i2c_acc)
tag3 = 'Computation time'
comp_time = map (sub, imu, i2c)
display.append (("IMU collector data",[imu, i2c, comp_time], ["Time to get info from imu (gyro+acc)", "Time of i2c operations", tag3], 3))

i = 1
for info in display:

    title = info[0]

    datas = info[1]
    tags = info[2]
    ndata = info[3]

    fig = pyplot.figure (i, figsize = (20,10))
    fig.suptitle (title)

    # Set title for the window
    fig.canvas.set_window_title (title)
    plots = []

    s = fig.add_subplot (111)
    s.grid (True)
    for j in xrange (ndata):
        p, = s.plot(datas[j])
        plots.append (p)

    s.legend (plots, tags, prop={'size':8},fancybox=True, loc='upper left')

    i += 1
# end for

pyplot.show ()            
for i in xrange (len (display)):
    pyplot.close (i+1)
