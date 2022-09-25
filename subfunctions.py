import numpy as np
from math import erf 
from define_rover import *

rover, planet = define_rover_1()

#works with test file
def tau_dcmotor(omega,motor):  
    tau=0                      
    if np.isscalar(omega):  #test if omega is scalar
        tau = motor['torque_stall'] - ((motor['torque_stall']       
            - motor['torque_noload'])/motor['speed_noload'])*omega 
        if omega > motor['speed_noload']:   #test if the value of omega is greater than the no load speed
            tau = 0                         #of the motor. if so, set tau = 0
        elif omega < 0:                      #test if value of omega is less than 0. if so, set tau = motor stall torque
            tau = motor['torque_stall']
    elif isinstance(omega,np.ndarray):  #test if omega is numpy array
        tau = motor['torque_stall'] - ((motor['torque_stall']      
            - motor['torque_noload'])/motor['speed_noload'])*omega 
        for i in range(len(omega)):     #run a loop iterating over all values in omega vector
            if omega[i] > motor['speed_noload']:    #test if the current value of omega is greater than the no load
                tau[i] = 0                          #speed. if so, set tau = 0
            elif omega[i] < 0:                          #test if current value of omega is less than 0. if so, set 
                tau[i] = motor['torque_stall']          
    else: raise Exception('The first argument is neither a scalar nor a vector.')   #raise exception if type of omega is not a scalar or vector
    if not isinstance(motor, dict): #test if type of motor is dict
        raise Exception('The second argument is not a dict.')   #raise exception if type of motor is not dict
    return tau  #return the calculated value of tau

def get_gear_ratio(speed_reducer):  #define function get_gear_ratio that receives dict speed_reducer and returns
    Ng=0                            #the gear ratio of the speed reducer
    if not isinstance(speed_reducer,dict):  #test if the the type of speed_reducer is not dict. if not, raise exception
        raise Exception('The argument is not a dict.')
    if speed_reducer['type'].casefold() != 'reverted':  #test if the value of 'type' in speed_reducer is not 'reverted'
        raise Exception('The type of the given speed reducer is not reverted.')     #if not, raise exception
    Ng = (speed_reducer['diam_gear']/speed_reducer['diam_pinion'])**2   #calculate the gear ratio of speed reducer
    return Ng   #return value of gear ratio

def get_mass(rover):        #define function get_mass that receives dict rover and returns mass of rover
    if not isinstance(rover,dict):  #test if the typer of rover is not dict. if not, raise exception
        raise Exception("The argument is not a dict.")
    #calculate the mass of all 6 wheel assemblies from data in rover dict   
    wheel_assembly = 6*(rover['wheel_assembly']['wheel']['mass']+rover['wheel_assembly']['speed_reducer']['mass']+rover['wheel_assembly']['motor']['mass'])
    mass = rover['power_subsys']['mass'] + rover['science_payload']['mass'] + rover['chassis']['mass'] + wheel_assembly
    return mass     #return value of rover mass

#this works with the test file
def F_drive(omega,rover):       #define function F_drice that receives array or scalar omega and dict rover
    Fd=0
    if np.isscalar(omega) or isinstance(omega,np.ndarray): #test if omega is scalar or array
        tau_in = tau_dcmotor(omega,rover['wheel_assembly']['motor'])
        tau_out = tau_in*get_gear_ratio(rover['wheel_assembly']['speed_reducer'])
        Fd = 6*tau_out/rover['wheel_assembly']['wheel']['radius']       #calculate drive force
    else: raise Exception('The first argument is neither a scalar nor a vector.')   #raise exception if not array or scalar
    if not isinstance(rover['wheel_assembly']['motor'], dict):
        raise Exception('The second argument is not a dict.')
    return Fd   #return value of drive force

#this works with the test file
def F_gravity(terrain_angle,rover,planet):
    Fgt=0
    if np.isscalar(terrain_angle): 
        if terrain_angle < -75 or terrain_angle > 75:
            raise Exception('One of the values for the terrain angle is not between -75 and 75 degrees.')
        Fgt = -1*get_mass(rover)*planet['g']*np.sin(terrain_angle*np.pi/180)
    elif isinstance(terrain_angle,np.ndarray):
        if (terrain_angle < -75).any() or (terrain_angle > 75).any():
            raise Exception('One of the values for the terrain angle is not between -75 and 75 degrees.')
        Fgt = -1*get_mass(rover)*planet['g']*np.sin(terrain_angle*np.pi/180)
    else: raise Exception('The first argument is neither a scalar nor a vector.')
    if not isinstance(rover, dict):
        raise Exception('The second argument is not a dict.')
    if not isinstance(planet, dict):
        raise Exception('The third argument is not a dict.')
    return Fgt


#assuming this works since f_net works
def F_rolling(omega,terrain_angle,rover,planet,Crr):
    Frr=0
    Fn=0
    if np.isscalar(omega) or isinstance(omega,np.ndarray):
        if np.isscalar(terrain_angle): 
            if terrain_angle < -75 or terrain_angle > 75:
                raise Exception('One of the values for the terrain angle is not between -75 and 75 degrees.')
            Fn = get_mass(rover)*planet['g']*np.cos(terrain_angle*np.pi/180)
        elif isinstance(terrain_angle,np.ndarray):
            if (terrain_angle < -75).any() or (terrain_angle > 75).any():
                raise Exception('One of the values for the terrain angle is not between -75 and 75 degrees.')
            Fn = get_mass(rover)*planet['g']*np.cos(terrain_angle*np.pi/180)
        else: raise Exception('The second argument is neither a scalar nor a vector.') 
    else: raise Exception('The first argument is neither a scalar nor a vector.')
    if not isinstance(rover,dict):
        raise Exception('The third argument is not a dict.')
    if not isinstance(planet,dict):
        raise Exception('The fourth argument is not a dict.')
    if not np.isscalar(Crr) or isinstance(Crr, str):
        raise Exception('The fifth argument is not a scalar.')
    elif Crr <= 0:
        raise Exception('The fifth argument is not postive.')
    Frrs = -1*Crr*Fn
    omega_out = omega/get_gear_ratio(rover['wheel_assembly']['speed_reducer'])
    v_rover = omega_out*rover['wheel_assembly']['wheel']['radius']    
    if isinstance(omega,np.ndarray):
        Frr=[0]
        for i in range(len(omega)):
            Frr = erf(40*v_rover[i])*Frrs
    else: Frr = erf(40*v_rover)*Frrs  
    return Frr


#this works with the test file
def F_net(omega, terrain_angle, rover, planet, Crr):
    F=0
    if not isinstance(rover,dict):
        raise Exception('The third argument is not a dict.')
    if not isinstance(planet,dict):
        raise Exception('The fourth argument is not a dict.')
    if not np.isscalar(Crr) or isinstance(Crr, str):
        raise Exception('The fifth argument is not a scalar.')
    elif Crr <= 0:
        raise Exception('The fifth argument is not postive.') 
    if np.isscalar(omega) or isinstance(omega, np.ndarray):
        if np.isscalar(terrain_angle):
            if terrain_angle < -75 or terrain_angle > 75:
                raise Exception('One of the values for the terrain angle is not between -75 and 75 degrees.')
            F = F_drive(omega,rover) + F_gravity(terrain_angle, rover, planet) + F_rolling(omega, terrain_angle, rover, planet, Crr)
        elif isinstance(terrain_angle,np.ndarray):
            if (terrain_angle < -75).any() or (terrain_angle > 75).any():
                raise Exception('One of the values for the terrain angle is not between -75 and 75 degrees.')
        else: raise Exception('The second argument is neither a scalar nor a vector.')
    else: raise Exception('The first argument is neither a scalar not a vector.')
    F = F_drive(omega,rover) + F_gravity(terrain_angle,rover,planet) + F_rolling(omega,terrain_angle,rover,planet,Crr)
    return F 
