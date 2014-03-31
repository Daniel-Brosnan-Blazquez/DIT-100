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
    Tj1 = 0                         
    Tj2 = 0                         
    dt = params['dt']               
    if not params['trajectory']:    
        # Maximum acceleration and velocity values in degrees/s^2 and     
        # degrees/s respectively    
        jmax = params['jerk_limit_d']*sign*(-1)
        amax = params['acc_limit_d']*sign*(-1)
        vmax = params['vel_limit']*sign*(-1)

        v0 = vel                    
        h = angle                   
        # Calculate feasibility of the trajectory (3.17)                  
        Tj = min ((abs (vel/jmax))**(1/2.0), abs (amax/jmax))             
        feasible = False            
        first_case = Tj < (abs (amax/jmax))                               
        if first_case:              
            if abs (h) > Tj*abs (vel):
                # The trajectory is feasible (3.18-1)                     
                feasible = True     
            # end if                
        else:                       
            second_term = 1/2.0*abs (vel)*(Tj+(abs (vel/amax)))           
            if abs (h) > second_term:                                     
                # The trajectory is feasible (3.18-2)                     
                feasible = True     
            # end if                
        # end if                    

        # Check if the trajectory is feasible                             
        if feasible:                
            # Determine if the maximum acceleration is reached on         
            # acceleration and deceleration phases
            amax_reached = True     
            amin_reached = True     
            # (3.19)                
            if abs ((vmax - vel)*jmax) < amax**2:                         
                amax_reached = False
            # (3.20)                
            elif abs (vmax*jmax) < amax**2:                               
                amin_reached = False
            # end if                

            # Calculate Tj1 and Ta  
            if not amax_reached:    
                # (3.21)            
                Tj1 = abs ((vmax-vel)/jmax)**(1/2.0)                      
                Ta = 2*Tj1          
            else:                   
                # (3.22)            
                Tj1 = amax/jmax     
                Ta = Tj1 + abs ((vmax - vel)/amax)                        
            # end if                

            # Calculate Tj2 and Td  
            if not amin_reached:    
                # (3.23)            
                Tj2 = (vmax/jmax)**(1/2.0)                                
                Td = 2*Tj2          
            else:                   
                # (3.24)            
                Tj2 = amax/jmax     
                Td = Tj2 + vmax/amax
            # end if                

            # Calculate Tv (3.25) 
            term1 = abs (h/vmax)
            term2 = Ta/2.0 * (1+(vel/vmax))
            term3 = Td/2.0
            Tv = term1 - term2 - term3

            find_params = False
            # Check if vlim < vmax  
            if Tv < 0:
                # The maximum velocity is not reached in the trajectory   
                # The constant velocity segment is not present            
                Tv = 0              
                                    
                gamma = 1.0
                find_params = True
                initial_amax = amax     
                while gamma > 0 and find_params:
                    # Determine amax for recursive computation            
                    amax = gamma*initial_amax

                    # Calculate Tj1 and Tj2 (3.26a)
                    Tj = amax/jmax  
                    Tj1 = Tj        
                    Tj2 = Tj        

                    # Calculate A (3.27)                                  
                    term1 = (amax**4)/(jmax**2)                           
                    term2 = 2*vel**2
                    term3 = abs (amax) * (4*(-h)-2*(amax/jmax)*vel)             
                    A = term1 + term2 + term3                             
                    A = abs (A)

                    # Calculate Ta (3.26b)                                
                    Ta = ((amax**2/abs (jmax)) - 2*vel + A**(1/2.0))/(2*abs (amax))   
                                    
                    # Calculate Td (3.26c)                                
                    Td = ((amax**2/abs (jmax)) + A**(1/2.0))/(2*abs (amax))
                    
                    # Check negative value of Td                          
                    if Td < 0:    
                        Td = 0      
                        Tj2 = 0     
                        # Calculate Ta (3.29a)                            
                        Ta = (2*(-h))/(vel)                               
                                    
                        # Calculate Tj2 (3.29b)                           
                        term1 = jmax*(-h)                                 
                        term2 = (jmax*(jmax*(h**2)-(vel**2)*(-vel)))**(1/2.0)                                   
                        term3 = jmax*vel                                  
                        Tj2 = (term1 - term2)/term3                       
                                    
                        # Parameters set                                  
                        find_params = False                               

                        sign_v0 = 1
                        if v0 < 0:
                            sign_v0 = -1
                        # end if
                            
                        sign_vmax = 1
                        if vmax < 0:
                            sign_vmax = -1
                        # end if

                        params['only_Ta_sign'] = 1
                        if sign_v0 != sign_vmax:
                            params['only_Ta_sign'] = -1

                    # Check negative value of Ta                          
                    elif Ta < 0:      
                        Ta = 0      
                        Tj1 = 0     
                        # Calculate Td (3.28a)

                        Td = (2*(-h))/(vel)                               
                                    
                        # Calculate Tj2 (3.28b)                           
                        term1 = jmax*(-h)                                 
                        term2 = (jmax*(jmax*(h**2)+(vel**2)*(-vel)))**(1/2.0)                                   
                        term3 = jmax*vel                                  
                        Tj2 = (term1 - term2)/term3                       
                                    
                        # Parameters set
                        find_params = False
                        
                        sign_v0 = 1
                        if v0 < 0:
                            sign_v0 = -1
                        # end if
                            
                        sign_vmax = 1
                        if vmax < 0:
                            sign_vmax = -1
                        # end if

                        params['only_Td_sign'] = 1
                        if sign_v0 != sign_vmax:
                            params['only_Td_sign'] = -1
                        
                    elif Ta > 2*Tj and Td > 2*Tj:                         
                        # Parameters set                                  
                        find_params = False                               
                    # end if        
                                    
                    # Continue searching the parameters                   
                    gamma -= 0.25   
                # end while         
                                    
            # end if

            if not find_params:
                params['trajectory'] = True
            # end if

            if params['trajectory']:
                # The time has to be positive                                 
                Ta = abs (Ta)           
                Td = abs (Td)           
                Tv = abs (Tv)           
                Tj1 = abs (Tj1)         
                Tj2 = abs (Tj2)         
                T = Ta + Td + Tv    
                
                params['trajectory'] = True
                
                t0 = 0
                params['T'] = t0 + T
                params['Ta'] = Ta
                params['Td'] = Td
                params['Tv'] = Tv
                params['Tj1'] = Tj1
                params['Tj2'] = Tj2
                params['T_sign'] = sign                                   
                
                # Check if the initial velocity is higher than the limit velocity
                params['Ta_sign'] = 1
                sign_v0 = 1
                if v0 < 0:
                    sign_v0 = -1
                # end if

                sign_vmax = 1
                if vmax < 0:
                    sign_vmax = -1
                # end if

                if sign_v0 == sign_vmax and not 'only_Ta_sign' in params.keys():
                    if abs (v0) > abs (vmax):
                        params['Ta_sign'] = -1
                    # end if

                # end if                
                                    
        # end if                    
                                    
    # end if

    return

def plot (params):
    
    t = 0
    interval = 0.015

    # Sign
    sign = params['T_sign']*(-1)
    
    # Sign for the acceleration phase. Inverted in case that the
    # initial velocity is higher than the limit
    Ta_sign = params['Ta_sign']

    # Maximum values
    jmax = params['jerk_limit_d']*sign
    amax = params['acc_limit_d']*sign
    vmax = params['vel_limit']*sign

    # Buffers to store the motion
    positions = []
    vels = []
    accs = []
    jerks = []
    # Initial values of the motion
    v0 = params['v0']
    p0 = params['p0']

    T = params['T']
    Ta = params['Ta']
    Td = params['Td']
    Tv = params['Tv']
    Tj1 = params['Tj1']
    Tj2 = params['Tj2']
    
    # Acceleration phase
    initial_jmax = jmax
    jmax = jmax*Ta_sign

    if 'only_Ta_sign' in params.keys ():
        jmax = jmax*params['only_Ta_sign']

    # a
    while t < Tj1:
        # Position
        pos = p0 + v0*t + jmax*(t**3/6.0)
        positions.append (pos)
        # Velocity
        vel = v0 + jmax*(t**2/2.0)
        vels.append (vel)
        # Acceleration
        acc = jmax*t
        accs.append (acc)
        # Jerk
        jerks.append (jmax)
        
        t += interval
    # end while

    # b
    alim = jmax*Tj1
    while t < Ta - Tj1:
        # Position
        pos = p0 + v0*t + (alim/6.0)*(3*t**2 - 3*Tj1*t + Tj1**2)
        positions.append (pos)
        # Velocity
        vel = v0 + alim*(t - (Tj1/2.0))
        vels.append (vel)
        # Acceleration
        acc = alim
        accs.append (acc)
        # Jerk
        jerks.append (0)
        
        t += interval
    # end while

    vlim = v0 + (Ta-Tj1)*alim 
    # c
    # Correct sign of jmax
    jmax = jmax*(-1)
    while t < Ta:
        # Position
        pos = p0 + (vlim + v0)*(Ta/2.0) - vlim *(Ta - t) - jmax *((Ta-t)**3/6.0)
        positions.append (pos)
        # Velocity
        vel = vlim + jmax*((Ta-t)**2/2.0)
        vels.append (vel)
        # Acceleration
        acc = -jmax*(Ta - t)
        accs.append (acc)
        # Jerk
        jerks.append (jmax)
        
        t += interval
    # end while
        
    # Restore jmax value
    jmax = initial_jmax*(-1)

    # Constant velocity phase
    while t < Ta+Tv:
        # Position
        pos = p0 + (vlim + v0)*(Ta/2.0) + vlim*(t-Ta)
        positions.append (pos)
        # Velocity
        vel = vlim
        vels.append (vel)
        # Acceleration
        acc = 0
        accs.append (acc)
        # Jerk
        jerks.append (0)
        
        t += interval
    # end while

    # Deceleration phase
    # a
    if 'only_Td_sign' in params.keys ():
        jmax = jmax*params['only_Td_sign']
    while t < T - Td + Tj2:
        # Position
        pos = 0 - vlim * (Td/2.0) + vlim * (t-T+Td) + jmax * ((t-T+Td)**3/6.0)
        positions.append (pos)
        # Velocity
        vel = vlim + jmax * ((t-T+Td)**2/2.0)
        vels.append (vel)
        # Acceleration
        acc = jmax*(t-T+Td)
        accs.append (acc)
        # Jerk
        jerks.append (jmax)
        
        t += interval
    # end while

    # b
    alim = jmax*Tj2
    while t < T - Tj2:
        # Position
        pos = 0 - vlim*(Td/2.0) + vlim*(t-T+Td) + alim/6.0*(3*(t-T+Td)**2 - 3*Tj2*(t-T+Td) + Tj2**2)
        positions.append (pos)
        # Velocity
        vel = vlim + alim*(t-T+Td-Tj2/2.0)
        vels.append (vel)
        # Acceleration
        acc = alim
        accs.append (acc)
        # Jerk
        jerks.append (0)
        
        t += interval
    # end while

    # c
    # Correct sign of jmax
    jmax = jmax*(-1)
    while t < T:
        # Position
        pos = 0 - jmax*((T-t)**3/6.0)
        positions.append (pos)
        # Velocity
        vel = jmax*((T-t)**2/2.0)
        vels.append (vel)
        # Acceleration
        acc = -jmax*(T-t)
        accs.append (acc)
        # Jerk
        jerks.append (jmax)
        
        t += interval
    # end while

    fig = pyplot.figure (1, figsize = (20,10))

    s = fig.add_subplot (411)
    p, = s.plot(positions)
    s.grid (True)
    s.set_title ("position")

    s = fig.add_subplot (412)
    p, = s.plot(vels)
    s.grid (True)
    s.set_title ("velocity")

    s = fig.add_subplot (413)
    p, = s.plot(accs)
    s.grid (True)
    s.set_title ("acceleration")

    s = fig.add_subplot (414)
    p, = s.plot(jerks)
    s.grid (True)
    s.set_title ("jerk")

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
    # m/s^3
    params['jerk_limit'] = 150.0
    # degrees/s^3
    params['jerk_limit_d'] = (params['jerk_limit']*rad_to_degrees)/radius

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
#     params['p0'] = 11.0962258945
# #    params['p0'] = 22.0
#     # v0
#     params['v0'] = 71.19
# #    params['v0'] = 0.0

    main(params)

    print "Trajectory performed: %s" % params['trajectory']
    if params['trajectory']:
        T = params['T']
        Ta = params['Ta']
        Td = params['Td']
        Tv = params['Tv']
        Tj1 = params['Tj1']
        Tj2 = params['Tj2']
        print "T = %s, Ta = %s, Td = %s, Tv = %s, Tj1 = %s, Tj2 = %s" %(T, Ta, Td, Tv, Tj1, Tj2)

        plot (params)
    
