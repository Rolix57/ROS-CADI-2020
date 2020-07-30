''' 
EXAMPLE: Longitudinal vehicle model
'''

import numpy as np
from matplotlib import pyplot as plt

class Vehicle():
    # initialize model
    def __init__(self):
        self.deltat = 0.01              # [s] Time step
        # Tires
        self.Rl = 16.0 * 25.4e-3        # [m] Wheel radius
        # Aerodynamics
        self.rho = 1.275                # [kg/m^3] Air density
        self.Cx = 0.4                   
        # [-] Aerodynamic coefficient, x axis
        self.S = 2.2                    # [m^2] Vehicle projection area
        # Rolling resistance
        self.cr = 0.01                  # [Ns/m] Rolling resistance coefficient
        # Road grade
        self.alpha = 0                  # [rad] Road slope angle 
        self.g = 9.81                   # [m/s^2] Gravity constant
        # Powertrain
        self.m = 1800.0                 # [kg] Vehicle mass
        self.ma = 3000.0                # [kg] Total equivalent inertia
        self.taug = 0.3                 # [-] Gearbox ratio
        self.taup = 0.1                 # [-] Final transmission ratio
        self.T = 60.0                   # [Nm] Input torque
        self.Tmax = 100.0               # [Nm] Maximum allowable torque
        self.etat = 0.8                 # [-] Transmission efficiency
        # States
        self.v = 0.0                    # [m/s] Vehicle speed
        
    def calcStep(self):
        Fpwt = self.etat * self.T / (self.Rl * self.taup * self.taug)
        Faero = 1/2 * self.rho * self.v ** 2 * self.Cx * self.S
        Froad = self.m * self.g * np.sin(self.alpha)
        Froll = self.cr * self.v
        Fsum = Fpwt - Faero - Froad - Froll
        self.v = self.v + Fsum / self.ma * self.deltat
        
class LongControl():
    def __init__(self):
        self.Kp = 100.0
        self.u = 0
        self.usat = 0
        self.umax = 0
        self.umin = 0
        self.ref = 35.0
        self.y = 0
        self.err = 0
    
    def calcStep(self):
        err = self.ref - self.y
        self.u = self.Kp * err
        self.usat = np.max([np.min([self.umax,self.u]),self.umin])

test_car = Vehicle()
ctrl = LongControl()
ctrl.umax = test_car.Tmax
ctrl.umin = -test_car.Tmax
   
# Load NEDC
import csv

with open('nedc.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    tcycle = np.array([0])
    vcycle = np.array([0])
    for row in csv_reader:
        if line_count == 0:
            pass
        else:
            tseg = np.arange(test_car.deltat, float(row[3]), test_car.deltat)
            vseg = np.linspace(float(row[0]), float(row[1]), len(tseg))
            tcycle = np.append(tcycle,tseg + tcycle[-1])
            vcycle = np.append(vcycle,vseg)
        line_count += 1

vcycle = vcycle / 3.6
vout = np.array([])
Treq = np.array([])
for i in range(len(tcycle)):
    ctrl.ref = vcycle[i]
    ctrl.y = test_car.v
    ctrl.calcStep()
    test_car.T = ctrl.usat
    test_car.calcStep()
    vout = np.append(vout, test_car.v)
    Treq = np.append(Treq, test_car.T)

Pmot = np.multiply(Treq, vout) / (test_car.Rl * test_car.taug * test_car.taup)
    
plt.figure(dpi=400, figsize=[6,5])
plt.subplot(3,1,1)
plt.plot(tcycle, vout)
plt.plot(tcycle, vcycle)
plt.ylabel('long. speed [km/h]')
plt.grid()

plt.subplot(3,1,2)
plt.plot(tcycle, Treq)
plt.ylabel('torque req. [Nm]')
plt.grid()

plt.subplot(3,1,3)
plt.plot(tcycle, Pmot / 1e3)
plt.xlabel('time [s]')
plt.ylabel('motor power [kW]')
plt.grid()

print('Average drawn power [W]: ', np.mean(Pmot[Pmot>0]))
print('Average harvested power [W]: ', np.mean(Pmot[Pmot<0]))