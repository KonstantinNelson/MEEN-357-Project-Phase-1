import numpy as np
from math import erf
import matplotlib.pyplot as plt
from define_rover import *
from subfunctions import tau_dcmotor

omega = np.linspace(0,3.8,25)

rover, planet = define_rover_1()
tau = tau_dcmotor(omega,rover['wheel_assembly']['motor'])
motor = rover['wheel_assembly']['motor']
P = -(motor['speed_noload']/motor['torque_stall'])*tau**2 + motor['speed_noload']*tau


#still need to adjust graphs
plt.subplot(3,1,1)
plt.plot(tau,omega)
plt.xlabel('Motor Shaft Torque (Nm)')
plt.ylabel('Motor Shaft Speed (rad/s)')

plt.subplot(3,1,2)
plt.plot(tau,P)
plt.xlabel('Motor Shaft Torque (Nm)')
plt.ylabel('Motor Shaft Speed (rad/s)')

plt.subplot(3,1,3)
plt.plot(omega,P)
plt.xlabel('Motor Shaft Speed (rad/s)')
plt.ylabel('Motor Shaft Power (W)')
plt.tight_layout()
plt.show()
