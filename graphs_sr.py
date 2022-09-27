import numpy as np
from math import erf
import matplotlib.pyplot as plt
from define_rover import *
from subfunctions import tau_dcmotor, get_gear_ratio

#generate values of omega
omega = np.linspace(0,3.8,25)
#import rover and planet dicts from define_rover
rover, planet = define_rover_1()
#calculate values of tau of the speed reducer output
tau = tau_dcmotor(omega,rover['wheel_assembly']['motor'])*get_gear_ratio(rover['wheel_assembly']['speed_reducer'])
#redefine omega as speed reducer omega values
omega=omega/get_gear_ratio(rover['wheel_assembly']['speed_reducer'])
#calculate the output power of the speed reducer
P = omega*tau

#graphs
f = plt.figure(figsize=(8,7))
ax=f.add_subplot(311,xlabel='Speed Reducer Torque [Nm]',ylabel='Speed Reducer Speed [rad/s]')
ax2=f.add_subplot(312,xlabel='Speed Reducer Torque [Nm]',ylabel='Speed Reducer Power [W]')
ax3=f.add_subplot(313,xlabel='Speed Reducer Speed [rad/s]',ylabel='Speed Reducer Power [W]')
ax.plot(tau,omega)
ax2.plot(tau,P)
ax3.plot(omega,P)
plt.tight_layout()
