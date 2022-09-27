import numpy as np
from subfunctions import *
from define_rover import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from basic_bisection_method import *

#import rover and planet dicts from define_rover
rover, planet = define_rover_1()
#define value as radius of wheel
wheel_radius = rover['wheel_assembly']['wheel']['radius']
#define value as output from get_gear_ratio function
gear_ratio=get_gear_ratio(rover['wheel_assembly']['speed_reducer'])
slope_list_deg = np.linspace(-10,35,25) #define values of slope angles and coefficient of resistances
Crr_list = np.linspace(0.01,0.4,25)
omega_max = np.zeros(len(slope_list_deg), dtype = float)    #initialize omega_max as array of zeroes
omega_nl = rover['wheel_assembly']['motor']['speed_noload']

Crr, Slope = np.meshgrid (Crr_list, slope_list_deg)
VMAX = np.zeros(np.shape(Crr), dtype = float)   #initialize VMAX as array of zeros
N = np.shape(Crr)[0]

for i in range(N):
    for j in range(N):
        Crr_sample = float(Crr[i,j])
        slope_sample = float(Slope[i,j])
        #define F_net as a function of omega
        fun = lambda omega: F_net(omega, slope_sample, rover, planet, Crr_sample)
        #get the root, error, interations, and exit flag of basic_bisection
        root,err_est,numIter,exitFlag = basic_bisection(fun,0,omega_nl)
        if exitFlag==0: #exit flag of 0 means function reached maximum iteration number
            VMAX[i,j]=np.nan        #so there is no solution
        else:
            VMAX[i,j]=root*wheel_radius/gear_ratio #convert the value of omega to speed of the rover
        
#3D graph of terrain angle and Crr vs maximum speed
fig = plt.figure()
ax = Axes3D(fig)
ax.plot_surface(Crr, Slope, VMAX)
ax.set_title("Max Rover Speed:Varying Terrain angles and Rolling Resistances")
ax.set_xlabel('Crr')
ax.set_ylabel('Terrain Angle [deg]')
ax.set_zlabel('Max Rover Speed [m/s]')
