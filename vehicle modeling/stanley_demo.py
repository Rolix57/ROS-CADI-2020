''' 
EXAMPLE: Lateral vehicle model
'''

import numpy as np
from matplotlib import pyplot as plt

class Vehicle():
    # initialize model
    def __init__(self):
        self.deltat = 0.01              # [s] Time step
        # Vehicle
        self.l = 1.0                    # [m] Wheelbase
        # Input
        self.delta =  0                 # [rad] Steering angle
        self.deltamax = np.deg2rad(25.0)# [rad] Steering angle bound
        self.v = 2.0                   # [m/s] Longitudinal speed
        # States
        self.x1 = 0                     # [m] Front axle x position
        self.y1 = 0                     # [m] Front axle y position
        self.psi = np.deg2rad(120.0)    # [rad/s] Yaw angle
                
    def calcStep(self):
        self.x1 += (self.v * np.cos(self.psi + self.delta)) * self.deltat
        self.y1 += (self.v * np.sin(self.psi + self.delta)) * self.deltat
        self.psi += (self.v * np.sin(self.delta) / self.l) * self.deltat
 
class Stanley():
    # initialize model
    def __init__(self):
        self.K = 2.5                    # [-] Proportional gain
        self.epsilon = 0.0              # [m] Cross-track error
        self.psi_hat = 0.0              # [rad] Yaw angle wrt trajectory
        self.delta = 0.0                # [rad] Steering angle
        self.deltasat = 0.0             # [rad] Saturated steering angle
                
    def calcStep(self, v, deltamax):
        self.delta = self.psi_hat + np.arctan2(self.K * self.epsilon, v)
        self.deltasat = np.max([np.min([deltamax,self.delta]),-deltamax])
       
test_car = Vehicle()
control = Stanley()

x = np.array([])
y = np.array([])
psi = np.array([])
delta_unsat = np.array([])
delta = np.array([])

t = np.array([])
wp_x = np.array([])
wp_y = np.array([])

while test_car.x1 < 40.0:
    # Calculate front axle kinematics
    test_car.calcStep()
    
    # Calculate yaw angle relative to path
    control.psi_hat = -test_car.psi
    
    # Calculate cross-track error
    if test_car.x1 < 20.0:
        control.epsilon = 0 - test_car.y1
        wp_y = np.append(wp_y, 0)
    else:
        control.epsilon = 5.0 - test_car.y1
        wp_y = np.append(wp_y, 5.0)
    wp_x = np.append(wp_x, test_car.x1)
    
    # Compute control law and apply steering angle to the vehicle
    control.calcStep(test_car.v, test_car.deltamax)
    test_car.delta = control.deltasat
    
    x = np.append(x, test_car.x1)
    y = np.append(y, test_car.y1)
    psi = np.append(psi, test_car.psi)
    delta_unsat = np.append(delta_unsat, control.delta)
    delta = np.append(delta, control.deltasat)
    t = np.append(t, test_car.deltat)

t = np.cumsum(t)

plt.figure(dpi=400, figsize=[6,6])
plt.subplot(2,1,1)
plt.plot(x, y)
plt.axis('equal')
plt.grid()
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.plot(wp_x, wp_y, 'r--')

downsample = int(150.0 / test_car.v)
xa = x[::downsample]
ya = y[::downsample]
psia = psi[::downsample]

for i in range(len(xa)):
    plt.arrow(xa[i], ya[i], test_car.l * np.cos(psia[i]), test_car.l * np.sin(psia[i]),
              fc="g", ec="g", head_width=0.5, head_length=1.0)

plt.subplot(2,1,2)
plt.plot(x, np.rad2deg(-psi), label='$\hat\psi$ alignment')
plt.plot(x, np.rad2deg(delta_unsat + psi), label='$\epsilon$ elimination')
plt.plot(x, np.rad2deg(delta_unsat), label='$\delta$ unsaturated')
plt.plot(x, np.rad2deg(delta), '--', label='$\delta$ saturated')
plt.grid()
plt.legend()
plt.xlabel('x [m]')
plt.ylabel('Steering angle [deg]')
