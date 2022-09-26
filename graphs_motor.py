import numpy as np
from math import erf
import matplotlib.pyplot as plt
from define_rover import *
from subfunctions import tau_dcmotor

#generate np array of omega values
omega = np.linspace(0,3.8,25)

#import rover and planet dicts from define_rover
rover, planet = define_rover_1()
#calculate tau values 
tau = tau_dcmotor(omega,rover['wheel_assembly']['motor'])
motor = rover['wheel_assembly']['motor']
#calculate the value(s) of motor power
P = -(motor['speed_noload']/motor['torque_stall'])*tau**2 + motor['speed_noload']*tau

#graphs
f = plt.figure(figsize=(8,7))
ax=f.add_subplot(311,xlabel='Motor Shaft Torque [Nm]',ylabel='Motor Shaft Speed [rad/s]')
ax2=f.add_subplot(312,xlabel='Motor Shaft Torque [Nm]',ylabel='Motor Power [W]')
ax3=f.add_subplot(313,xlabel='Motor Shaft Speed [rad/s]',ylabel='Motor Power [W]')
ax.plot(tau,omega)
ax2.plot(tau,P)
ax3.plot(omega,P)
plt.tight_layout()
