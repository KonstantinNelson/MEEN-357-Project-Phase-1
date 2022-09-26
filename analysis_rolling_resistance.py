import numpy as np
from subfunctions import *
from define_rover import *
from scipy.optimize import root_scalar
import matplotlib.pyplot as plt


rover, planet = define_rover_1()
slope = 0
Crr_array = numpy.linspace(0.01,0.4,25)
v_max = np.zeros(len(Crr_array), dtype = float)




for i in range(len(Crr_array)):
    
    

plt.plot(Crr_array,v_max)
plt.xlabel('Coeficient of rolling resistance [degrees]')
plt.ylabel('Maximum velocity [m/s]')
plt.show()

#old code below---------------------------------------------------------------------------------
omega_max = np.zeros(len(slope_list_deg), dtype = float)
omega_nl = rover['wheel_assembly']['motor']['speed_noload']

# find where F_net == 0
for i in range(len(slope_list_deg)):
    fun = lambda omega: F_net(omega, float(slope_list_deg[i]), rover, planet, Crr)
    sol = root_scalar(fun, method='bisect', bracket=[0, omega_nl])
    omega_max[i] = sol.root


