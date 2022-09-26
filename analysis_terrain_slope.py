import numpy as np
from subfunctions import *
from define_rover import *
from scipy.optimize import root_scalar
import matplotlib.pyplot as plt

rover, planet = define_rover_1()
Crr = 0.2
slope_list_deg = np.linspace(-10,35,25)
omega_max = np.zeros(len(slope_list_deg), dtype = float)
omega_nl = rover['wheel_assembly']['motor']['speed_noload']

# find where F_net == 0
for i in range(len(slope_list_deg)):
    fun = lambda omega: F_net(omega, float(slope_list_deg[i]), rover, planet, Crr)
    sol = root_scalar(fun, method='bisect', bracket=[0, omega_nl])
    omega_max[i] = sol.root

plt.plot(slope_list_deg,omega_max)
plt.xlabel('Terrain Angle [deg]')
plt.ylabel('Max Rover Speed [m/s]')
plt.show()
