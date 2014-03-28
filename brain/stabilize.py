# -*- coding: utf-8 -*-

# Mini brain for the drone stabilization and communication

import sys
import os
import re
import importlib
import time
sys.path.append ("../lib")
import common
import threading
import numpy
import signal
import pwm

DT = 0.02
COMPLEMENTARY = 0.98
FREQUENCY = 400
MIN_CHANNEL = 12
MAX_CHANNEL = 15
# VREF = 5                   # Default value for Vref (when the Rpi is supplied through the BEC)
VREF = 4.82                   # Used value when Rpi is supplied through t
INIT_POWER = 0.0
DUTY_MAX = 800.0
DUTY_MIN = 400.0

FIGWIDTH = 20
FIGHEIGTH = 10

def get_acc (Q, P):
    ''' Function to obtain the Q value from acceleration '''
    return Q*P
#    return (Q-5)*(1.5/5) + 1.5

class th(threading.Thread):  
    ''' Thread to manage the motor's power '''
    def __init__(self, params):  
        threading.Thread.__init__(self) 
        self.params = params
        
    def run(self):  
        
        raw = ''
        # Get the power increment to all the motors
        while raw != 'c':
            raw = raw_input("The motor's power value in percentage is %s. Enter the increment power to apply or c to terminate or n to apply increment or r to reset the values:" % self.params['current_power'])
            
            if raw == 'c':
                self.params['stop'] = True
                continue
            
            if raw == 'n':
                self.params['next'] = True
                continue

            if raw == 's':
                self.params['start'] = True
                continue

            if raw == 'r':
                self.params['reset'] = True
                continue
            
            try:
                value = int (raw)
                if abs (value) > 100 or abs (value) < 0:
                    print "The entered value was incorrect. It must be on the range (0,100)"
                    continue
                else:
                    self.params['power'] = int (value)
                    self.params['inc_pw'] = True
                    self.params['next'] = True
            except:
                print "The entered value was incorrect. It must be of type int"
                continue

def main():
    
    # Search collectors
    collectors_dir = "../collectors"
    if not os.path.isdir (collectors_dir):
        print "The path to the collectors is incorrect. It has to be an existing directory"
        return
    # end if

    parent_dir = os.path.dirname (collectors_dir)
    sys.path.append (parent_dir)
    (head, dir_modules) = os.path.split (collectors_dir)

    entries = os.listdir (collectors_dir)
    collectors = []

    for entry in entries:
        # Avoid problematic names
        if "#" in entry or entry == "__init__.py":
            continue
        # end if

        # Check if it is a .py file
        if entry[len(entry)-3:len(entry)] == ".py":
            entry = entry[0:len(entry)-3]
            module = "%s.%s" % (dir_modules,entry)
            collector = importlib.import_module(module)
            # Check if the collector has to be skipped
            if hasattr (collector, 'SKIP') and collector.SKIP:
                continue
            collectors.append (collector)
        # end if
    # end for

    # Search actuators
    actuators_dir = "../actuators"
    if not os.path.isdir (actuators_dir):
        print "The path to the actuators is incorrect. It has to be an existing directory"
        return
    # end if

    parent_dir = os.path.dirname (actuators_dir)
    sys.path.append (parent_dir)
    (head, dir_modules) = os.path.split (actuators_dir)

    entries = os.listdir (actuators_dir)
    sys.path.append (actuators_dir)

    actuators = []

    for entry in entries:
        # Avoid problematic names
        if "#" in entry or entry == "__init__.py":
            continue
        # end if

        # Check if it is a .py file
        if entry[len(entry)-3:len(entry)] == ".py":
            entry = entry[0:len(entry)-3]
            module = "%s.%s" % (dir_modules,entry)
            actuator = importlib.import_module(module)
            # Check if the collector has to be skipped
            if hasattr (actuator, 'SKIP') and actuator.SKIP:
                continue
            actuators.append (actuator)
        # end if
    # end for

    ctx = {}

    # Period of the main brain
    ctx['dt'] = DT
    # Value for the complementary filter
    ctx['comp'] = COMPLEMENTARY
    # Frequency for the PWM signal
    ctx['freq'] = FREQUENCY
    # Channels of the PWM device
    ctx['maxch'] = MAX_CHANNEL
    ctx['minch'] = MIN_CHANNEL
    # Reference voltage for the ADC device
    ctx['vref'] = VREF
    # Initial power for the motors
    ctx['init_power'] = INIT_POWER
    # Current power of the motors
    ctx['current_power'] = INIT_POWER
    ctx['next'] = False
    # Flag to increment current total power of the motors
    ctx['inc_pw'] = False
    ctx['reset'] = True
    # Range for the duty cycle of the PWM signal
    ctx['duty_max'] = DUTY_MAX
    ctx['duty_min'] = DUTY_MIN
    # Flag to stop the program
    ctx['stop'] = False
    # Flag to start the control of the motors
    ctx['start'] = False
    ctx['dcs'] = {'m12': None,'m13': None,'m14': None,'m15':None}
    ctx['check_b'] = False
    # Flag to indicate if the motors stopped
    ctx['motors_stopped'] = False
    
    # Debug information
    # Constant value to convert bettween acceleration and Q values
    ctx['P'] = 3/10.0
    # Buffers to store the power values of each motor
    for motor in xrange (ctx['minch'], ctx['maxch']+1):
        ctx['powers_p%s' % motor] = []

    # Buffer to store the global time taken in the actuator
    ctx['time'] = []
    # Buffers to store the angular velocity values on each iteration
    # for pitch and roll angles
    ctx['velsp_gyro_iter'] = []
    ctx['velsr_gyro_iter'] = []
    # Value of time to calculate the elapsed global time on each
    # iteration
    ctx['bf_time_imu'] = time.time ()
    # previous velocity value for pitch and roll angles
    ctx['prev_velp'] = 0
    ctx['prev_velr'] = 0
    # Buffer to store the angular acceleration values on each iteration
    ctx['accsp_iter'] = []
    ctx['accsr_iter'] = []
    # Buffer to store the tangential component of the gravity on each
    # iteration for pitch and roll angles
    ctx['p_gts_iter'] = []
    ctx['r_gts_iter'] = []
    # Buffers to store the values of the roll and pitch angles
    # corrected by the accelerometer, not corrected and the values
    # from the accelerometer
    ctx['p_corr'] = []
    ctx['p_notcorr'] = []
    ctx['p_acc'] = []
    ctx['p_180'] = []
    ctx['r_corr'] = []
    ctx['r_notcorr'] = []
    ctx['r_acc'] = []
    ctx['r_180'] = []
    # Scales for gravity
    ctx['sp'] = 0.804
    ctx['sn'] = 0.856
    # Constant to convert to radians from degrees
    ctx['degrees_to_rad'] = numpy.pi/180
    ctx['rad_to_degrees'] = 180/numpy.pi
    # Constant for the radius of the drone to represent the pendulum
    ctx['radius'] = 0.3
    # Buffer to store the Q values
    ctx['p_Qs'] = []
    ctx['p_Qts'] = []
    ctx['p_Qgs'] = []
    ctx['r_Qs'] = []
    ctx['r_Qts'] = []
    ctx['r_Qgs'] = []
    # Q limit
    ctx['Q_limit'] = 25
    # Acceleration limit in m/s^2
    ctx['acc_limit'] = get_acc (ctx['Q_limit'], ctx['P'])
    # Acceleration limit in degrees/s^2
    ctx['acc_limit_d'] = (ctx['acc_limit']*ctx['rad_to_degrees'])/ctx['radius']
    # Jerk limit in m/s^2
    ctx['jerk_limit'] = 150.0
    # Jerk limit in degrees/s^2
    ctx['jerk_limit_d'] = (ctx['jerk_limit']*ctx['rad_to_degrees'])/ctx['radius']
    # Velocity limit in degrees/s
    ctx['vel_limit'] = 150.0
    # Buffer to store consumed time
    ctx['con_time'] = [{}]
    # Buffers to store the number of values got from the gyroscope and from the accelerometer
    ctx['acc_nvalues'] = []
    ctx['gyro_nvalues'] = []
    # Variable to indicate if there is new data available from gyro
    ctx['new_gyro_data'] = False
    # Variables to manage trajectories
    ctx['p_T'] = 0
    ctx['p_Ta_end'] = 0
    ctx['p_Td_begin'] = 0
    ctx['p_trajectory'] = False
    ctx['p_T_sign'] = 0
    ctx['p_T_quadrant'] = None
    ctx['r_T'] = 0
    ctx['r_Ta_end'] = 0
    ctx['r_Td_begin'] = 0
    ctx['r_trajectory'] = False
    ctx['r_T_sign'] = 0
    ctx['r_T_quadrant'] = None
    # Buffers to store time values of the trajectory
    ctx['p_Ts'] = []
    ctx['p_Tas'] = []
    ctx['p_Tds'] = []
    ctx['r_Ts'] = []
    ctx['r_Tas'] = []
    ctx['r_Tds'] = []
    # Buffer for the values from the accelerometer
    ctx['acc_data'] = []
    # Buffer for the values from the altimeter
    ctx['alt_data'] = []

    # Initialize the collectors
    for collector in collectors:
        if hasattr (collector, 'initialize'):
            (status, message) = collector.initialize (ctx)
            if not status:
                print message
            # end if
        # end if
    # end for

    # Remove collectors that have not got get_info function
    index = 0
    while index < len (collectors):
        if not hasattr (collectors[index], 'get_info'):
            collectors.pop (index)
        else:
            index += 1

    interface = th (ctx)
    interface.start ()

    bf = time.time ()
    # Brain loop

    iterations_between_prints = 1
    iterations = 0

    # Variables for the automated control of time with alarms
    stop_time = 10
    init_power_time = 10
    init_power_value = 50
    inc_power_time = 1
    inc_power_value = 5

    # Function to stop the main-brain with an alarm signal
    def stop(p, frame):
        print "Stop the program"
        for motor in xrange (ctx['minch'], ctx['maxch']+1):
            ctx['m%s' % motor] = ctx['current_power']
        # end for

        ctx['stop'] = True
        return

    def inc_power(p, frame):
        print "Incrementing power to = %s" % (ctx['current_power'] + inc_power_value)

        ctx['power'] = inc_power_value
        ctx['inc_pw'] = True

        signal.signal(signal.SIGALRM, stop)
        signal.alarm(stop_time)

        return

    # Function to introduce the first value of power for all motors
    def init_power(p, frame):
        
        print "Setting init power to = %s" % init_power_value
        ctx['power'] = init_power_value
        ctx['inc_pw'] = True
        ctx['start'] = True

        signal.signal(signal.SIGALRM, inc_power)
        signal.alarm(inc_power_time)

        return

    signal.signal(signal.SIGALRM, init_power)
    signal.alarm(init_power_time)
  
    next = time.time ()

    while not ctx['motors_stopped']:

        time_to_sleep = next - time.time ()
        if time_to_sleep > 0:
            time.sleep (time_to_sleep)

        # Collect information
        for collector in collectors:
            if hasattr (collector, 'SKIP') and collector.SKIP:
                continue
            # end if

            if hasattr (collector, 'get_info'):
                (status, message) = collector.get_info (ctx)
                if not status:
                    print message
                    sys.exit(0)
                # end if
                    
            # end if
        # end for

        # Act depending on the collected information
        for actuator in actuators:
            if hasattr (actuator, 'SKIP') and actuator.SKIP:
                continue
            # end if

            if hasattr (actuator, 'act'):
                (status, message) = actuator.act (ctx)
                if not status:
                    print message
                    sys.exit(0)
                # end if

            # end if

        # end for

        # Register time
        af = time.time ()
        diff = af - bf
        bf = af
        if not ctx['con_time'][0].has_key('mini-brain'):
            ctx['con_time'][0]['mini-brain'] = [diff]
        else:
            ctx['con_time'][0]['mini-brain'].append (diff)
        # end if

        # Next time for the loop
        next += ctx['dt']

    # end while

    # Ensure that all the motors are stopped
    (status, message) = pwm.set_pwm_ai (ctx['pwm'], ctx['minch'], [(400,0),(400,0),(400,0),(400,0)])
    if not status:
        return (False, "Unable to stop the motors. The error was %s" % message)

    file_data = "data"
    print "data saved in the file %s" % file_data
    numpy.savez (file_data, p12_values = ctx['powers_p12'], p13_values = ctx['powers_p13'], p14_values = ctx['powers_p14'], p15_values = ctx['powers_p15'], p_corr=ctx['p_corr'], p_notcorr = ctx['p_notcorr'], p_180=ctx['p_180'], p_acc = ctx['p_acc'], r_corr=ctx['r_corr'], r_notcorr = ctx['r_notcorr'], r_180=ctx['r_180'], r_acc = ctx['r_acc'], accsr_iter = ctx['accsr_iter'], accsp_iter = ctx['accsp_iter'], p_gts_iter = ctx['p_gts_iter'], r_gts_iter = ctx['r_gts_iter'], p_Qs = ctx['p_Qs'], p_Qts = ctx['p_Qts'], p_Qgs = ctx['p_Qgs'], r_Qs = ctx['r_Qs'], r_Qts = ctx['r_Qts'], r_Qgs = ctx['r_Qgs'], velsp_gyro_iter=ctx['velsp_gyro_iter'], velsr_gyro_iter=ctx['velsr_gyro_iter'], con_time=ctx['con_time'], acc_nvalues= ctx['acc_nvalues'], gyro_nvalues= ctx['gyro_nvalues'], p_Ts=ctx['p_Ts'], p_Tas=ctx['p_Tas'], p_Tds=ctx['p_Tds'], r_Ts=ctx['r_Ts'], r_Tas=ctx['r_Tas'], r_Tds=ctx['r_Tds'], acc_data=ctx['acc_data'], alt_data=ctx['alt_data'])

    return

if __name__ == "__main__":
    main()
