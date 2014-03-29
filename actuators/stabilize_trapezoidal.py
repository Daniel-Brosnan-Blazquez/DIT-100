import sys
sys.path.append ("../lib")
import lib.pwm as pwm
import common
import time
import numpy

# First motor makes the angle negative when goes up
motors_angle = {'p':[13,15], 'r':[14,12]}

SKIP = False

def get_gt (angle, quadrant, params, sign):
    ''' Get tangential component from gravity in m/s^2 '''
    # Calculate beta angle for the pendulum model
    #                angle=0, b=90 
    #                      | 
    #                  3   |  0
    #                      |       
    # angle=-90, b=0 ------+------ angle=90, b=0
    #                      |        
    #                  2   |  1    
    #                      |
    #               angle=0, b=270
    if quadrant == 0:
        beta = numpy.pi/2.0 - angle   
    elif quadrant == 1:
        beta = -(numpy.pi/2.0 - angle)
    elif quadrant == 2:
        beta = -numpy.pi/2.0 + angle
    else:
        beta = numpy.pi/2.0 - angle   
    # end if
        
    gt = 9.81 * numpy.cos (beta)
    if gt > 0:
        gt = gt*params['sp']
    else:
        gt = gt*params['sn']
    # end if

    # Set the right sign for the tangential component of gravity
    gt = gt * sign
    
    return gt

def get_sign (quadrant):
    ''' Get sign for Q value for each angle
                      angle=0
                     -   |  +
             angle=-90 --+-- angle=90
                     -   |  +    
                      angle=0
    '''
    sign = 1
    if quadrant == 2 or quadrant == 3:
        sign = -1

    return sign

def get_Qg (sign, gt, params):
    ''' Get qg value (qg = -gt) acceleration to counter the gravity '''
    # qg in m/s^2
    qg = abs (gt)
    # Qg = 5 + (qg - 1.5) * (10 - 5)/(3 - 1.5)
    # Convert to PWM 
    Qg = get_Q (qg, params['P'])
    Qg = abs (Qg)*sign

    return Qg

def set_trajectory (angle, vel, sign, quadrant, params, angle_name):

    T = 0
    Ta = 0
    Td = 0

    # Modify angle to follow this convention
    #                   angle=0
    #                  3   |  0
    #          angle=-90 --+-- angle=90
    #                  2   |  1    
    #                   angle=180
    if quadrant == 1:
        angle = 180 - angle
    elif quadrant == 2:
        angle = -180 -angle

    # Period of the main brain
    dt = params['dt']

    # Maximum acceleration and velocity values in degrees/s^2 and
    # degrees/s respectively
    amax = params['acc_limit_d']*sign*(-1)
    vmax = params['vel_limit']*sign*(-1)
    v0 = vel
    h = angle
    # Check if the trajectory is feasible
    if abs (amax*h) >= v0**2/2.0:
        # The trajectory is feasible
        # Check if the maximum value of velocity can be reached
        if abs (h*amax) > vmax**2 - v0**2/2.0:
            # The maximum value of velocity can be reached
            Ta = (vmax - v0)/amax
            Td = vmax/amax
            term1 = abs (h/vmax)
            term2 = (vmax/(2*amax)) * (1 - (v0/vmax))**2
            term3 = (vmax/(2*amax))
            T = term1 + term2 + term3
        else:
            # The maximum value of velocity can't be reached
            vlim = ((abs (h * amax) + v0**2/2.0)**(1/2.0))*sign*(-1)
            Ta = abs ((vlim - v0)/amax)
            Td = abs (vlim/amax)
            T = Ta + Td
        # end if

        # The time has to be positive
        Ta = abs (Ta)
        Td = abs (Td)
        T = abs (T)
                
        params['%s_trajectory' % angle_name] = True
            
        t0 = time.time ()
        params['%s_T' % angle_name] = t0 + T
        params['%s_Ta_end' % angle_name] = t0 + Ta
        params['%s_Td_begin' % angle_name] = t0 + (T-Td)
        params['%s_T_sign' % angle_name] = sign
        params['%s_T_quadrant' % angle_name] = quadrant
                
    # end if

    return (T, Ta, Td)

def determine_endT (angle, quadrant, current_time, params):
    ''' Check if the trajectory plan has finished '''

    t_label = '%s_trajectory' % angle
    q_label = '%s_T_quadrant' % angle
    if params[t_label] and current_time > (params['%s_T' % angle]):
        params[t_label] = False
    elif params[t_label] and params[q_label] == 0 and quadrant == 3:
        params[t_label] = False
    elif params[t_label] and params[q_label] == 3 and quadrant == 0:
        params[t_label] = False
    elif params[t_label] and params[q_label] == 1 and quadrant == 2:
        params[t_label] = False
    elif params[t_label] and params[q_label] == 2 and quadrant == 1:
        params[t_label] = False
    # end if

    return

def get_Qt (angle, current_time, params):
    ''' Get the Q component due to the trajectory plan '''

    # Get sign of the acceleration
    sign_acc = 0
    if params['%s_trajectory' % angle]:
        if current_time <= params['%s_Ta_end' % angle]:
            if (params['%s_Ta_end' % angle] - current_time) < params['dt']:
                sign_acc = 0
            else:
                sign_acc = 1
            # end if
        elif current_time >= params['%s_Td_begin' % angle] \
                and current_time < params['%s_T' % angle]:
            if (params['%s_T' % angle] - current_time) < params['dt']:
                sign_acc = 0
            else:
                sign_acc = -1
            # end if
        # end if            

    # end if

    Qt = params['Q_limit']*params['%s_T_sign' % angle]*sign_acc

    return Qt

def set_power (motors, Q, params):
    ''' Set power for each pair of motors '''

    # Get data to calculate power for each motor
    m1 = motors[0]
    m2 = motors[1]

    # Get total power
    total = params['current_power']*2

    # Set power for each motor
    p2 = (total-Q)/2.0
    p1 = Q + p2

    # Discretize the power values for each motor
    sign = 1
    if p2 != 0:
        sign = abs (p2)/p2

    p2 = abs (p2)
    p2 = p2 - (p2 % 0.25)
    p2 = p2*sign

    sign = 1
    if p1 != 0:
        sign = abs (p1)/p1

    p1 = abs (p1)
    p1 = p1 - (p1 % 0.25)
    p1 = p1*sign

    params['m%s' % m1] = inc_pw (0, p1)
    params['m%s' % m2] = inc_pw (0, p2)
 
    # Store power values for each motor
    params['powers_p%s' % m1].append (params['m%s' % m1])
    params['powers_p%s' % m2].append (params['m%s' % m2])

    return

def get_dc (power,d_max,d_min):
    return int (d_max + (power-100.0) * ((d_min-d_max)/(0.0-100.0)))

def get_Q (acc, P):
    ''' Function to obtain the acceleration from the Q value '''
    return acc/P
#    return 5 + (acc - 1.5) * (10 - 5)/(3 - 1.5)

def get_acc (Q, P):
    ''' Function to obtain the Q value from acceleration '''
    return Q*P
#    return (Q-5)*(1.5/5) + 1.5

def inc_pw (power, inc):
    if (power + inc) > 100:
        return 100

    # default minimal
    if (power + inc) < 12:
        return 12

    return power + inc

def apply_pw (params):

    values = []
    for motor in xrange (params['minch'], params['maxch']+1):
        dc = get_dc (params['m%s' % motor],params['duty_max'],params['duty_min'])
        values.append ((dc, 0))
    # end for

    bf_i2c = time.time ()
    (status, message) = pwm.set_pwm_ai (params['pwm'], params['minch'], values)
    if not status:
        return (False, "Unable to set the pwm signal. The error was %s" % message)
    # end if

    af_i2c = time.time ()
    params['diff_i2c'] += (af_i2c - bf_i2c)

    params['dcs']['m%s' % motor] = dc

    return (True, None)

def act (params):

    bf = time.time ()
    params['diff_i2c'] = 0
    
    # Get difference time bettween interations
    af_time = time.time ()
    diff = af_time - params['bf_time_imu']
    params['bf_time_imu'] = af_time

    if params['stop']:
        # stop all engines
        engines_stopped = 0
        for motor in xrange (params['minch'], params['maxch']+1):
            params['m%s' % motor] -=  0.25
            if params['m%s' % motor] <= 0:
                engines_stopped += 1
                params['m%s' % motor] = 0
        # end for

        apply_pw (params)
        if engines_stopped == 4:
            params['motors_stopped'] = True

        # Register time
        af = time.time ()
        diff = af - bf
        if not params['con_time'][0].has_key('act'):
            params['con_time'][0]['act'] = [diff]
        else:
            params['con_time'][0]['act'].append (diff)
        # end if

        # Register time due to i2c operations
        if not params['con_time'][0].has_key('i2c_act'):
            params['con_time'][0]['i2c_act'] = [params['diff_i2c']]
        else:
            params['con_time'][0]['i2c_act'].append (params['diff_i2c'])
        # end if

        return (True, None)
    # end if
    
    # Check battery
    if params['check_b']:
        for i in xrange (3):
            if params['bm']['cell%s' % i][1] < 20:
                for motor in xrange (params['minch'], params['maxch']+1):
                    params['m%s' % motor] = 0
                # end for
                    
                apply_pw (params)

                # Register time
                af = time.time ()
                diff = af - bf
                if not params['con_time'][0].has_key('act'):
                    params['con_time'][0]['act'] = [diff]
                else:
                    params['con_time'][0]['act'].append (diff)
                # end if

                # Register time due to i2c operations
                if not params['con_time'][0].has_key('i2c_act'):
                    params['con_time'][0]['i2c_act'] = [params['diff_i2c']]
                else:
                    params['con_time'][0]['i2c_act'].append (params['diff_i2c'])
                # end if

                return (True, None)
            # end if
        # end for
    # end if

    if params ['reset']:
        print "reset"
        for motor in xrange (params['minch'], params['maxch']+1):
            params['m%s' % motor] = params['init_power']
        # end for

        params ['reset'] = False

        apply_pw (params)

        # Register time
        af = time.time ()
        diff = af - bf
        if not params['con_time'][0].has_key('act'):
            params['con_time'][0]['act'] = [diff]
        else:
            params['con_time'][0]['act'].append (diff)
        # end if

        # Register time due to i2c operations
        if not params['con_time'][0].has_key('i2c_act'):
            params['con_time'][0]['i2c_act'] = [params['diff_i2c']]
        else:
            params['con_time'][0]['i2c_act'].append (params['diff_i2c'])
        # end if

        return (True, None)
    # end if

    # Check if has to be incremented the power of all motors
    if params['inc_pw']:
        params['current_power'] = inc_pw (params['current_power'], params['power'])
        for motor in xrange (params['minch'], params['maxch']+1):
            params['m%s' % motor] = params['current_power']
        # end for

        params['power'] = 0
        params['inc_pw'] = False
        apply_pw (params)

        # Register time
        af = time.time ()
        diff = af - bf
        if not params['con_time'][0].has_key('act'):
            params['con_time'][0]['act'] = [diff]
        else:
            params['con_time'][0]['act'].append (diff)
        # end if

        # Register time due to i2c operations
        if not params['con_time'][0].has_key('i2c_act'):
            params['con_time'][0]['i2c_act'] = [params['diff_i2c']]
        else:
            params['con_time'][0]['i2c_act'].append (params['diff_i2c'])
        # end if

        return (True, None)
    # end if

    if not params ['start']:
        print "No collecting data"

        # Register time
        af = time.time ()
        diff = af - bf
        if not params['con_time'][0].has_key('act'):
            params['con_time'][0]['act'] = [diff]
        else:
            params['con_time'][0]['act'].append (diff)
        # end if

        # Register time due to i2c operations
        if not params['con_time'][0].has_key('i2c_act'):
            params['con_time'][0]['i2c_act'] = [params['diff_i2c']]
        else:
            params['con_time'][0]['i2c_act'].append (params['diff_i2c'])
        # end if

        return (True, None)

    # If there is no new data from gyro 
    if not params['new_gyro_data']:
        # Register time
        af = time.time ()
        diff = af - bf
        if not params['con_time'][0].has_key('act'):
            params['con_time'][0]['act'] = [diff]
        else:
            params['con_time'][0]['act'].append (diff)
        # end if

        # Register time due to i2c operations
        if not params['con_time'][0].has_key('i2c_act'):
            params['con_time'][0]['i2c_act'] = [params['diff_i2c']]
        else:
            params['con_time'][0]['i2c_act'].append (params['diff_i2c'])
        # end if

        return (True, None)

    # Get imu context
    imu = params['imu']

    # Data rate for gyro device
    dr = imu['gyro']['dr']

    # Radius of the drone for the pendulum model
    radius = params['radius']
    
    # Constant to convert from degrees to radians
    degrees_to_rad = params['degrees_to_rad']

    # Get pitch and roll values
    p = imu['p']
    r = imu['r']

    # Get quadrants for each angle
    p_quadrant = imu['p_quadrants']['quadrant']
    p_sign = imu['p_quadrants']['sign']
    r_quadrant = imu['r_quadrants']['quadrant']
    r_sign = imu['r_quadrants']['sign']

    for angle in ['p','r']:
        # Store the values associated to each angle, corrected and not
        # corrected by the accelerometer and the one obtained from the
        # accelerometer
        params['%s_corr' % angle].append (imu['%s' %angle])
        params['%s_notcorr' % angle].append (imu['%s_tocheck' % angle])
        params['%s_acc' % angle].append (imu['accel']['%s' % angle])

        # Store angular velocity in degrees/s of each angle
        vel = params['vel%s_gyro' % angle]
        params['vels%s_gyro_iter' % angle].append (vel)

        # Store the angular acceleration in m/s^2 of each angle
        # Degrees/s^2
        acc = (vel - params['prev_vel%s' % angle])/dr
        params['prev_vel%s' % angle] = vel
        # m/s^2
        acc = acc*degrees_to_rad*radius
        params['accs%s_iter' % angle].append (acc)

    # Get angles in radians
    p_rad = p*degrees_to_rad
    r_rad = r*degrees_to_rad

    # Get tangential component from gravity in m/s^2 for each angle
    p_gt = get_gt (p_rad, p_quadrant, params, p_sign)
    r_gt = get_gt (r_rad, r_quadrant, params, r_sign)

    params['p_gts_iter'].append (p_gt)
    params['r_gts_iter'].append (r_gt)

    # Get sign for Q value for each angle
    p_sign = get_sign (p_quadrant)
    r_sign = get_sign (r_quadrant)

    # Get qg value (qg = -gt) acceleration to counter the gravity
    p_Qg = get_Qg (p_sign, p_gt, params)
    r_Qg = get_Qg (r_sign, r_gt, params)

    # Plan the trajectory for the pitch angle if it is not planned
    T = 0
    Ta = 0
    Td = 0
    if not params['p_trajectory']:
        (T, Ta, Td) = set_trajectory (p, vel, p_sign, p_quadrant, params, 'p')

    params['p_Ts'].append (T)
    params['p_Tas'].append (Ta)
    params['p_Tds'].append (Td)

    # Plan the trajectory for the roll angle if it is not planned
    T = 0
    Ta = 0
    Td = 0
    if not params['r_trajectory']:
        (T, Ta, Td) = set_trajectory (r, vel, r_sign, r_quadrant, params, 'r')

    params['r_Ts'].append (T)
    params['r_Tas'].append (Ta)
    params['r_Tds'].append (Td)

    
    current_time = time.time ()

    # Determine if the trajectory plan has finished
    determine_endT ('p', p_quadrant, current_time, params)
    determine_endT ('r', r_quadrant, current_time, params)
            
    # Calculate Q component due to the trajectory plan
    p_Qt = get_Qt ('p', current_time, params)
    p_Q = p_Qg + p_Qt

    r_Qt = get_Qt ('r', current_time, params)
    r_Q = r_Qg + r_Qt

    # Store values of Q
    params['p_Qs'].append (p_Q)
    params['p_Qgs'].append (p_Qg)
    params['p_Qts'].append (p_Qt)
    params['r_Qs'].append (r_Q)
    params['r_Qgs'].append (r_Qg)
    params['r_Qts'].append (r_Qt)

    # Calculate power for each motor
    set_power (motors_angle['p'], p_Q, params)
    set_power (motors_angle['r'], r_Q, params)

    (status, message) = apply_pw(params)

    # Register time
    af = time.time ()
    diff = af - bf
    if not params['con_time'][0].has_key('act'):
        params['con_time'][0]['act'] = [diff]
    else:
        params['con_time'][0]['act'].append (diff)
    # end if

    # Register time due to i2c operations
    if not params['con_time'][0].has_key('i2c_act'):
        params['con_time'][0]['i2c_act'] = [params['diff_i2c']]
    else:
        params['con_time'][0]['i2c_act'].append (params['diff_i2c'])
    # end if

    return (status, message)
