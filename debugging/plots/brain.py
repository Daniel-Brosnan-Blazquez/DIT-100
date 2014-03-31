from matplotlib import pyplot
import numpy

# File from get data
file = "check_level3.npz"

# Get data from numpy file
data = numpy.load (file)

# Pitches from gyro corrected by values from accelerometer
p_corr = data['p_corr']

# Pitches from gyro not corrected
p_notcorr = data['p_notcorr']

# Pitches from accelerometer
p_acc = data['p_acc']

# Angular acceleration values every iteration
accs_iter = data['accsp_iter']

# Tangential components of gravity every iteration
gts_iter = data['gts_iter']

# Buffer to store the velocities due to the action of the motors
vels_Q = []
prev_vel_Q = 0

# Number of values from acc and gyro
acc_nvalues = data['acc_nvalues']
gyro_nvalues = data['gyro_nvalues']

# Q values
Qs = data['Qs']
Qrs = data['Qrs']
Qr_reduces = data['Qr_reduces']
Qr_velcorrs = data['Qr_velcorrs']
Qgs = data['Qgs']

# Power values for each motor
p1s = data['p1_values']
p2s = data['p2_values']

# Velocity values
vels = data['velsp_gyro_iter']

# Time of each iteration
time = data['time']

# Buffer to store the angular acceleration values due to the action of the motors
maccs = []

# Buffer to store the acceleration due to the motors based on the interpolation
maccs_inter = []

# Buffer to store the differences between accelerations
accs_diff = []
prev_acc  = 0

rad_to_degrees = 180/numpy.pi
radius = 0.3

i = 0
while i < len (accs_iter):
    
    # Get angular acceleration
    acc = accs_iter[i]
    diff = acc - prev_acc
    prev_acc = acc
    accs_diff.append (abs (diff))
    
    # Get tangential component of the gravity
    gt = gts_iter[i]

    # Set angular acceleration due to the motors
    macc = acc - gt

    # Store value
    maccs.append (macc)

    # Get acceleration from interpolation
    Q = Qs[i]
    sign = -abs (Q)/Q
    macc_inter = (Q-5)*(1.5/5) + 1.5
    macc_inter_gt = macc_inter
    macc_inter = abs (macc_inter) - abs (gt)
    maccs_inter.append (abs (macc_inter)*sign)

    # Calculate the velocity due to the action of the motors
    acc_Q = (abs (macc_inter)*sign*rad_to_degrees)/0.3
    vel_Q = prev_vel_Q + acc_Q*time[i]
    prev_vel_Q = vel_Q
    vels_Q.append (vel_Q)

    i += 1
# end while

fig = pyplot.figure (1, figsize = (20,10))
fig.suptitle (file)


s = fig.add_subplot (511)
p1, = s.plot(vels)
#p2, = s.plot(vels_Q)
s.grid (True)
#s.legend ([p1, p2], ["Velocity", "Velocity due to the motors"], prop={'size':8},fancybox=True, loc='upper left')
s.legend ([p1], ["Velocity"], prop={'size':8},fancybox=True, loc='upper left')

s = fig.add_subplot (512)
p1, = s.plot(p_corr)
#p2, = s.plot(p_notcorr)
#p3, = s.plot(p_acc)
# p4, = s.plot(predicted_p)
s.grid (True)
s.legend ([p1], ["angle"], prop={'size':8},fancybox=True, loc='upper left')
#s.legend ([p1, p2, p3], ["Pitches from gyro corrected by values from accelerometer", "Pitches from gyro not corrected", "pitches from accelerometer"], prop={'size':8},fancybox=True, loc='upper left')

# s = fig.add_subplot (513)
# p1, = s.plot(accs_iter)
# p2, = s.plot(gts_iter)
# s.grid (True)
# s.legend ([p1, p2], ["angular acceleration values", "tangential components of gravity"], prop={'size':8},fancybox=True, loc='upper left')

s = fig.add_subplot (513)
p1, = s.plot(accs_iter)
p2, = s.plot (maccs_inter)
p3, = s.plot (gts_iter)
s.grid (True)
s.legend ([p1, p2, p3], ["Angular acceleration due to the motors", "Angular acceleration due to the motors inter", "Angular acceleration due to the gravity"], prop={'size':8},fancybox=True, loc='upper left')

s = fig.add_subplot (515)
p1, = s.plot(Qrs)
p2, = s.plot(Qgs)
p3, = s.plot(Qs)
s.grid (True)
s.legend ([p1, p2, p3], ["Qrs","Qgs", "Qs"], prop={'size':8},fancybox=True, loc='upper left')

# s = fig.add_subplot (515)
# p1, = s.plot(acc_nvalues)
# p2, = s.plot(gyro_nvalues)
# s.grid (True)
# s.legend ([p1,p2], ["Number of from acc","Number of from gyro"], prop={'size':8},fancybox=True, loc='upper left')

# s = fig.add_subplot (514)
# p1, = s.plot(p1s)
# p2, = s.plot(p2s)
# s.grid (True)
# s.legend ([p1,p2], ["p1","p2"], prop={'size':8},fancybox=True, loc='upper left')

s = fig.add_subplot (514)
p1, = s.plot(Qr_velcorrs)
p2, = s.plot(Qr_reduces)
s.grid (True)
s.legend ([p1, p2], ["velcorr", "reduces"], prop={'size':8},fancybox=True, loc='upper left')


pyplot.show ()            
pyplot.close (1)
