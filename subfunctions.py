import numpy as np
from math import erf 
from define_rover import *

#call in define_rover_1 from define_rover file to get dictionaries
rover, planet = define_rover_1()


def tau_dcmotor(omega,motor):       #define function tau_dcmotor that receives scalar or array omega and motor dict
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

def F_drive(omega,rover):       #define function F_drice that receives array or scalar omega and dict rover
    Fd=0
    if not isinstance(rover, dict):
        raise Exception('The second argument is not a dict.')
    if np.isscalar(omega) or isinstance(omega,np.ndarray): #test if omega is scalar or array
        tau_in = tau_dcmotor(omega,rover['wheel_assembly']['motor'])
        tau_out = tau_in*get_gear_ratio(rover['wheel_assembly']['speed_reducer'])
        Fd = 6*tau_out/rover['wheel_assembly']['wheel']['radius']       #calculate drive force
    else: raise Exception('The first argument is neither a scalar nor a vector.')   #raise exception if not array or scalar
    return Fd   #return value of drive force

def F_gravity(terrain_angle,rover,planet):      #define function F_gravity that receives scalar or array terrain_angle and rover and planet dicts
    Fgt=0
    if not isinstance(rover, dict) or not isinstance(planet, dict):     #check if rover and planet are not dict types
            raise Exception('Either the second or third argument is not a dict.')
    if np.isscalar(terrain_angle):  #test if terrain_angle is a scalar
        if terrain_angle < -75 or terrain_angle > 75:       #check if angle value is not between positive and negative 75 degrees
            raise Exception('One of the values for the terrain angle is not between -75 and 75 degrees.')
        Fgt = -1*get_mass(rover)*planet['g']*np.sin(terrain_angle*np.pi/180)    #calculate the value gravity force
    elif isinstance(terrain_angle,np.ndarray):  #test if terrain_angle is an array
        if (terrain_angle < -75).any() or (terrain_angle > 75).any():   #check if any value is not between positive and negative 75 degrees
            raise Exception('One of the values for the terrain angle is not between -75 and 75 degrees.')
        Fgt = -1*get_mass(rover)*planet['g']*np.sin(terrain_angle*np.pi/180) #calculate array of gravity force values
    else: raise Exception('The first argument is neither a scalar nor a vector.')
    return Fgt

def F_rolling(omega,terrain_angle,rover,planet,Crr):    #define function F_rolling that receives scalar or array omega and terrain_angle, rover and planet dict, and scalar Crr
    Frr=0
    Fn=0
    if not isinstance(rover,dict) or not isinstance(planet,dict):        #check if rover or planet are not dicts
        raise Exception('Either the third or fourth argument is not a dict.')
    if np.isscalar(omega) or isinstance(omega,np.ndarray):      #check if omega is a scalar or array
        if np.isscalar(terrain_angle): #check if terrain_angle is a scalar
            if terrain_angle < -75 or terrain_angle > 75:   #test if terrain angle is not between positive and negative 75
                raise Exception('One of the values for the terrain angle is not between -75 and 75 degrees.')
            Fn = get_mass(rover)*planet['g']*np.cos(terrain_angle*np.pi/180)        #calculate value of normal force
        elif isinstance(terrain_angle,np.ndarray):  #check if terrain_angle is an array
            if (terrain_angle < -75).any() or (terrain_angle > 75).any():   #test if any value in terrain_angle is not between positive and negative 75
                raise Exception('One of the values for the terrain angle is not between -75 and 75 degrees.')
            Fn = get_mass(rover)*planet['g']*np.cos(terrain_angle*np.pi/180)    #calculate normal force
        else: raise Exception('The second argument is neither a scalar nor a vector.') 
    else: raise Exception('The first argument is neither a scalar nor a vector.')
    if not np.isscalar(Crr) or isinstance(Crr, str):    #raise exception if Crr is a string or not a scalar
        raise Exception('The fifth argument is not a scalar.')
    elif Crr <= 0:  #raise exception if Crr is negative
        raise Exception('The fifth argument is not postive.')
    Frrs = -1*Crr*Fn    #calculate rolling resistance force
    omega_out = omega/get_gear_ratio(rover['wheel_assembly']['speed_reducer'])  #calculate the value of the speed reducer omega output
    v_rover = omega_out*rover['wheel_assembly']['wheel']['radius']    #calculate the tangential velocity of the rover wheels
    if isinstance(omega,np.ndarray):
        Frr=[0]
        for i in range(len(omega)):
            Frr = erf(40*v_rover[i])*Frrs   #if the speed of the wheel is close to 0, the rolling resistance value should be 0
    else: Frr = erf(40*v_rover)*Frrs  
    return Frr

def F_net(omega, terrain_angle, rover, planet, Crr):    #define function F_net that receives scalar or array omega and terrain_angle, rover and planet dicts, and scalar Crr
    F=0
    if not isinstance(rover,dict):  #test if rover is not a dict
        raise Exception('The third argument is not a dict.')
    if not isinstance(planet,dict):     #raise exception if planet is not a dict
        raise Exception('The fourth argument is not a dict.')
    if not np.isscalar(Crr) or isinstance(Crr, str):    #raise exception if Crr is a string or not a scalar
        raise Exception('The fifth argument is not a scalar.')
    elif Crr <= 0:     #check if Crr is negative
        raise Exception('The fifth argument is not postive.') 
    if np.isscalar(omega) or isinstance(omega,np.ndarray):     #proceed if omega is a scalar or array
        if np.isscalar(terrain_angle):  #check if terrain_angle is a scalar or array
            if terrain_angle < -75 or terrain_angle > 75:   #raise exception if terrain_angle is not between negative and positive 75
                raise Exception('One of the values for the terrain angle is not between -75 and 75 degrees.')
            F = F_drive(omega,rover) + F_gravity(terrain_angle, rover, planet) + F_rolling(omega, terrain_angle, rover, planet, Crr)    #calculate net force
        elif isinstance(terrain_angle, np.ndarray):
            if terrain_angle.any() < -75 or terrain_angle.any() > 75:   #raise exception if terrain_angle is not between negative and positive 75
                raise Exception('One of the values for the terrain angle is not between -75 and 75 degrees.')
            F = F_drive(omega,rover) + F_gravity(terrain_angle, rover, planet) + F_rolling(omega, terrain_angle, rover, planet, Crr)    #calculate net force
        else: raise Exception('The second argument is neither a scalar nor a vector.')
    else: raise Exception('The first argument is neither a scalar not a vector.')
    return F 
