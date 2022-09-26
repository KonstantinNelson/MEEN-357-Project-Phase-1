
#not sure if the graphs are correct
import numpy as np
from math import erf
import matplotlib.pyplot as plt
from define_rover import *
from subfunctions import tau_dcmotor, get_gear_ratio

omega = np.linspace(0,3.8,25)

rover, planet = define_rover_1()
tau = tau_dcmotor(omega,rover['wheel_assembly']['motor'])*get_gear_ratio(rover['wheel_assembly']['speed_reducer'])
motor = rover['wheel_assembly']['motor']
P = -(motor['speed_noload']/motor['torque_stall'])*tau**2 + motor['speed_noload']*tau
omega=omega/get_gear_ratio(rover['wheel_assembly']['speed_reducer'])

f = plt.figure(figsize=(8,7))
ax=f.add_subplot(311,xlabel='Speed Reducer Torque [Nm]',ylabel='Speed Reducer Speed [rad/s]')
ax2=f.add_subplot(312,xlabel='Speed Reducer Torque [Nm]',ylabel='Speed Reducer Power [W]')
ax3=f.add_subplot(313,xlabel='Speed Reducer Speed [rad/s]',ylabel='Speed Reducer Power [W]')
ax.plot(tau,omega)
ax2.plot(tau,P)
ax3.plot(omega,P)
plt.tight_layout()
