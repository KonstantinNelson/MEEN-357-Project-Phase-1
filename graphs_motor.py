import numpy as np
from math import erf
import matplotlib.pyplot as plt
from define_rover import *

omega = np.linspace(0,3.8,25)

def tau_dcmotor(omega,motor):
    tau=0
    if np.isscalar(omega): 
        tau = motor['torque_stall'] - ((motor['torque_stall']
            - motor['torque_noload'])/motor['speed_noload'])*omega
        if omega > motor['speed_noload']:
            tau = 0
        elif omega < 0:
            tau = motor['torque_stall']
    elif isinstance(omega,np.ndarray):
        tau = motor['torque_stall'] - ((motor['torque_stall']
            - motor['torque_noload'])/motor['speed_noload'])*omega
        for i in range(len(omega)):
            if omega[i] > motor['speed_noload']:
                tau[i] = 0
            elif omega[i] < 0:
                tau[i] = motor['torque_stall']
    else: raise Exception('The first argument is neither a scalar nor a vector.')
    if not isinstance(motor, dict):
        raise Exception('The second argument is not a dict.')
    return tau

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
