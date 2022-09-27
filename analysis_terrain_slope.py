import numpy as np
from subfunctions import *
from define_rover import *
from scipy.optimize import root_scalar
import matplotlib.pyplot as plt

#import rover and planet dicts from define_rover
rover, planet = define_rover_1()
wheel_radius = rover['wheel_assembly']['wheel']['radius']
gear_ratio=get_gear_ratio(rover['wheel_assembly']['speed_reducer'])
Crr = 0.2
#generate values of slope angles
slope_list_deg = np.linspace(-10,35,25)
#initialize maximum omega value as array of zeros
v_max = np.zeros(len(slope_list_deg), dtype = float)
#retrieve the no load speed of the motor
omega_nl = rover['wheel_assembly']['motor']['speed_noload']

# find where F_net == 0
for i in range(len(slope_list_deg)):
    fun = lambda omega: F_net(omega, float(slope_list_deg[i]), rover, planet, Crr)
    sol = root_scalar(fun, method='bisect', bracket=[0, omega_nl])
    v_max[i] = sol.root*wheel_radius/gear_ratio

#graph maximum rover speeds at every terrain angle
plt.plot(slope_list_deg,v_max)
plt.xlabel('Terrain Angle [deg]')
plt.ylabel('Max Rover Speed [m/s]')
plt.show()
