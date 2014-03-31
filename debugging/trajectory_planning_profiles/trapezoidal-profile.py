import numpy
import time
from matplotlib import pyplot

def main (params):

    angle = params['p0']
    vel = params['v0']
    sign = params['sign']

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
        vlim = vmax
        # Check if the trajectory is feasible
        print "abs (amax*h) >= v0**2/2.0 = %s" % (abs (amax*h) >= v0**2/2.0)
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

            print "Ta = %s, Td = %s" % (Ta, Td)

            params['trajectory'] = True
            
            params['T'] = T
            params['Ta'] = Ta
            params['Td'] = Td
            params['T_sign'] = sign*(-1)
            params['vv'] = vlim

            # if Ta > dt and Td > dt:
            #     params['trajectory'] = True
                
            #     params['T'] = T
            #     params['Ta'] = Ta
            #     params['Td'] = Td
            #     params['T_sign'] = sign*(-1)
            #     params['vv'] = vlim
            # else:
            #     Ta = 0
            #     Td = 0
            #     T = 0
               
            # end if
                
    # end if

    return

def plot (params):
    
    t = 0
    interval = params['dt']

    # Sign
    sign = params['T_sign']
    
    # Maximum values
    amax = params['acc_limit_d']*sign
    vmax = params['vel_limit']*sign

    # Buffers to store the motion
    positions = []
    vels = []
    accs = []
    # Initial values of the motion
    v0 = params['v0']
    p0 = params['p0']
    vv = params['vv']

    T = params['T']
    Ta = params['Ta']
    Td = params['Td']
    
    # Acceleration phase
    while t < Ta:
        # Position
        pos = p0 + v0*t + ((vv - v0)/(2*Ta))*t**2
        positions.append (pos)
        # Velocity
        vel = v0 + ((vv - v0)/(Ta))*t
        vels.append (vel)
        # Acceleration
        acc = (vv - v0)/Ta
        accs.append (acc)
        
        t += interval
    # end while

    # Constant velocity phase
    while t < (T - Td):
        # Position
        pos = p0 + v0*(Ta/2.0) + vv*(t-(Ta/2.0))
        positions.append (pos)
        # Velocity
        vel = vv
        vels.append (vel)
        # Acceleration
        acc = 0
        accs.append (acc)
        
        t += interval
    # end while

    # Deceleration phase
    while t < T:
        # Position
        pos = 0 - (vv/(2*Td))*(T-t)**2
        positions.append (pos)
        # Velocity
        vel = (vv/Td)*(T-t)
        vels.append (vel)
        # Acceleration
        acc = -(vv/Td)
        accs.append (acc)
        
        t += interval
    # end while

    fig = pyplot.figure (1, figsize = (20,10))

    s = fig.add_subplot (311)
    p, = s.plot(positions)
    s.grid (True)
    s.set_title ("position")

    s = fig.add_subplot (312)
    p, = s.plot(vels)
    s.grid (True)
    s.set_title ("velocity")

    s = fig.add_subplot (313)
    p, = s.plot(accs)
    s.grid (True)
    s.set_title ("acceleration")

    pyplot.show ()
    pyplot.close (1)

    return

if __name__ == "__main__":          
    params = {}
    # Period
    params['dt'] = 0.015
    # Flag to indicate if it is necessary to compute the trajectory
    # (not needed here)
    params['trajectory'] = False
    # Velocity, acceleration and jerk limits in degrees/s^2
    params['vel_limit'] = 150.0
    rad_to_degrees = 180.0/numpy.pi
    radius = 0.3
    # m/s^2
    params['acc_limit'] = 7.5
    # degrees/s^2
    params['acc_limit_d'] = (params['acc_limit']*rad_to_degrees)/radius

    # # p0 = 0. Checked, trajectory unfeasible
    # # p0 
    # params['p0'] = 0.0
    # # v0
    # params['v0'] = 100.0

    # p0 > 50 v0 = 0. Checked, trajectory feasible
    # p0 
    params['p0'] = 80.0
    # v0
    params['v0'] = 0.0

    # # p0 > 50 v0 < limit. Checked, trajectory feasible
    # # p0 
    # params['p0'] = 80.0
    # # v0
    # params['v0'] = 50.0

    # # p0 > 50 v0 = limit. Checked, trajectory feasible
    # # p0 
    # params['p0'] = 80.0
    # # v0
    # params['v0'] = 100.0

    # # p0 > 50 v0 > limit. Checked, trajectory feasible
    # # p0 
    # params['p0'] = 80.0
    # # v0
    # params['v0'] = -150.0

    # # p0 < 50 p0 > 0 v0 = 0. Checked, trajectory feasible
    # # p0 
    # params['p0'] = 20.0
    # # v0
    # params['v0'] = 0.0

    # # p0 < 50 p0 > 0 v0 < limit. REVIEW IT!!!!!!!!!
    # # p0 
    # params['p0'] = 20.0
    # # v0
    # params['v0'] = 50.0

    # # p0 < 50 p0 > 0 v0 = limit. Checked, trajectory feasible
    # # p0 
    # params['p0'] = 20.0
    # # v0
    # params['v0'] = 100.0

    # # p0 < 50 p0 > 0 v0 > limit. Checked, trajectory feasible
    # # p0 
    # params['p0'] = 20.0
    # # v0
    # params['v0'] = 150.0

    # # p0 < -50 v0 = 0. Checked, trajectory feasible
    # # p0 
    # params['p0'] = -80.0
    # # v0
    # params['v0'] = 0.0

    # # p0 < -50 v0 < limit. Checked, trajectory feasible
    # # p0 
    # params['p0'] = -80.0
    # # v0
    # params['v0'] = 50.0

    # # p0 < -50 v0 = limit. Checked, trajectory feasible
    # # p0 
    # params['p0'] = -80.0
    # # v0
    # params['v0'] = 100.0

    # # p0 < -50 v0 > limit. Checked, trajectory feasible
    # # p0 
    # params['p0'] = -80.0
    # # v0
    # params['v0'] = 150.0

    # # p0 > -50 p0 < 0 v0 = 0. Checked, trajectory feasible
    # # p0 
    # params['p0'] = -20.0
    # # v0
    # params['v0'] = 0.0

    # # p0 > -50 p0 < 0 v0 < limit. Checked, trajectory feasible
    # # p0 
    # params['p0'] = -20.0
    # # v0
    # params['v0'] = -50.0

    # # p0 > -50 p0 < 0 v0 = limit. Checked, trajectory feasible
    # # p0 
    # params['p0'] = -20.0
    # # v0
    # params['v0'] = 100.0

    # # p0 > -50 p0 < 0 v0 > limit. Checked, trajectory feasible
    # # p0 
    # params['p0'] = -20.0
    # # v0
    # params['v0'] = 150.0

    # # p0 > -50 p0 < 0 v0 > limit. Checked, trajectory feasible
    # # p0 
    # params['p0'] = -20.0
    # # v0
    # params['v0'] = 200.0

    # sign
    params['sign'] = 1
#    params['sign'] = -1


#     # p0 
#    params['p0'] = 11.0962258945
# #    params['p0'] = 22.0
#     # v0
#    params['v0'] = 71.19
# #    params['v0'] = 0.0

    main(params)

    print "Trajectory performed: %s" % params['trajectory']
    if params['trajectory']:
        T = params['T']
        Ta = params['Ta']
        Td = params['Td']
        print "T = %s, Ta = %s, Td = %s" %(T, Ta, Td)

        plot (params)
    
