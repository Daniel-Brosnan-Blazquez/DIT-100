import sys
sys.path.append ("../lib")
import lib.pwm as pwm
import common
import time
import numpy

# First motor makes the angle negative when goes up
angle_to_level = 'p'
#angle_to_level = 'r'
motors_angle = {'p':[13,15], 'r':[14,12]}
motors = motors_angle[angle_to_level]

SKIP = False

def angle_to_key (angle):
    abs_angle = abs (angle)
    if abs_angle >= 35:
        return 35
    return abs_angle - (abs_angle % 5) + 5

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
    if (power + inc) < 15:
        return 15

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
        for motor in motors:
            params['m%s' % motor] = 0
        # end for

        apply_pw (params)
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
                for motor in motors:
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
        for motor in motors:
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

    # Get angle value
    angle = imu['%s' % angle_to_level]

    # Get quadrant
    quadrant = imu['%s_quadrants' % angle_to_level]['quadrant']

    # Store the angle values, corrected and not corrected by the
    # accelerometer and the one obtained from the accelerometer
    params['%s_corr' % angle_to_level].append (angle)
    params['%s_notcorr' % angle_to_level].append (imu['%s_tocheck' % angle_to_level])
    params['%s_acc' % angle_to_level].append (imu['accel']['%s' % angle_to_level])

    # Store angular velocity in degrees/s
    vel = params['vel%s_gyro' % angle_to_level]
    params['vels%s_gyro_iter' % angle_to_level].append (vel)

    # Store the angular acceleration in m/s^2
    # Degrees/s^2
    acc = (vel - params['prev_vel%s' % angle_to_level])/dr
    params['prev_vel%s' % angle_to_level] = vel
    # m/s^2
    acc = acc*degrees_to_rad*radius
    params['accs%s_iter' % angle_to_level].append (acc)

    # Get angle in radians
    angle_rad = angle*params['degrees_to_rad']

    # Get tangential gravity from gravity in m/s^2 and the angle
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
        beta = numpy.pi/2.0 - angle_rad   
    elif quadrant == 1:
        beta = -(numpy.pi/2.0 - angle_rad)
    elif quadrant == 2:
        beta = -numpy.pi/2.0 + angle_rad
    else:
        beta = numpy.pi/2.0 - angle_rad   
    # end if
        
    gt = 9.81 * numpy.cos (beta)
    if gt > 0:
        gt = gt*params['sp']
    else:
        gt = gt*params['sn']
    # end if
        
    # Set the right sign for the tangential component of gravity
    gt = gt * imu['%s_quadrants' % angle_to_level]['sign']
    params['gts_iter'].append (gt)

    # Get sign for Q value
    #                   angle=0
    #                  -   |  +
    #          angle=-90 --+-- angle=90
    #                  -   |  +    
    #                   angle=0
    sign = 1
    if quadrant == 2 or quadrant == 3:
        sign = -1

    # Get qg value (qg = -gt) acceleration to counter the gravity
    # qg in m/s^2
    qg = abs (gt)
    # Qg = 5 + (qg - 1.5) * (10 - 5)/(3 - 1.5)
    # Convert to PWM 
    Qg = get_Q (qg, params['P'])
    Qg = abs (Qg)*sign

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

    params['%s_180' % angle_to_level].append (angle)

    # Plan the trajectory if it is not planned
    T = 0
    Ta = 0
    Td = 0
    dt = params['dt']
    if not params['trajectory']:
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
                
            params['trajectory'] = True
            
            t0 = time.time ()
            params['T'] = t0 + T
            params['Ta_end'] = t0 + Ta
            params['Td_begin'] = t0 + (T-Td)
            params['T_sign'] = sign
            params['T_quadrant'] = quadrant

            # if Ta > dt and Td > dt:
            #     params['trajectory'] = True
                
            #     t0 = time.time ()
            #     params['T'] = t0 + T
            #     params['Ta_end'] = t0 + Ta
            #     params['Td_begin'] = t0 + (T-Td)
            #     params['T_sign'] = sign
            #     params['T_quadrant'] = quadrant
            # else:
            #     Ta = 0
            #     Td = 0
            #     T = 0
               
            # end if
                
    # end if

    params['Ts'].append (T)
    params['Tas'].append (Ta)
    params['Tds'].append (Td)
    
    current_time = time.time ()
    # Check if the trajectory plan has finished
    if params['trajectory'] and current_time > (params['T']):
        params['trajectory'] = False
    elif params['trajectory'] and params['T_quadrant'] == 0 and quadrant == 3:
        params['trajectory'] = False
    elif params['trajectory'] and params['T_quadrant'] == 3 and quadrant == 0:
        params['trajectory'] = False
    elif params['trajectory'] and params['T_quadrant'] == 1 and quadrant == 2:
        params['trajectory'] = False
    elif params['trajectory'] and params['T_quadrant'] == 2 and quadrant == 1:
        params['trajectory'] = False
    # end if

            
    # If there is a trajectory plan, calculate Q component
    sign_acc = 0
    if params['trajectory']:
        if current_time <= params['Ta_end']:
            if (params['Ta_end'] - current_time) < dt:
                sign_acc = 0
            else:
                sign_acc = 1
            # end if
        elif current_time >= params['Td_begin'] \
                and current_time < params['T']:
            if (params['T'] - current_time) < dt:
                sign_acc = 0
            else:
                sign_acc = -1
            # end if
        # end if            

    # end if

#    print "sign = %s, sign2 = %s" % (sign_acc, sign*sign_acc)

    # Set Q value (Q = p1-p2)
    Qp = params['Q_limit']*params['T_sign']*sign_acc
    Q = Qg + Qp
    Q = Q + params['offset']

    params['Qs'].append (Q)
    params['Qgs'].append (Qg)
    params['Qps'].append (Qp)

    # Get data to calculate power for each motor
    m1 = motors[0]
    m2 = motors[1]
    p1 = params['m%s' % m1]
    p2 = params['m%s' % m2]

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
    params['powers_p1'].append (params['m%s' % m1])
    params['powers_p2'].append (params['m%s' % m2])

    params['time'].append (diff)

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
