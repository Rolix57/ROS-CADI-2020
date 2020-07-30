import numpy as np
import matplotlib.pyplot as plt
import csv

# %% Load data from data log csv file

# CSV file structure: [t,ax,ay,az,wx,wy,wz,Bx,By,Bz]

with open('./sensorlog.csv', mode='r') as csvfile:
	data = np.array(list(csv.reader(csvfile))).astype(np.float)

# Time
t = data[:,0]
Ts = t[1]-t[0]
fs = 1/Ts
# Accelerometer measurements [m/s^2]
ax = data[:,1]
ay = data[:,2]
az = data[:,3]
# Gyroscope measurements [rad/s]
wx = data[:,4]
wy = data[:,5]
wz = data[:,6]
# Magnetometer measurements [uT]
Bx = data[:,7]
By = data[:,8]
Bz = data[:,9]

# Conventional angle synthesis
gyro_roll = np.cumsum(wx/fs)
gyro_pitch = np.cumsum(wy/fs)
gyro_yaw = np.cumsum(wz/fs)
acc_roll = np.unwrap(np.arctan2(ay, np.hypot(ax, az)))
acc_pitch = -np.unwrap(np.arctan2(ax, np.hypot(ay, az)))
mag_yaw = -np.unwrap(np.arctan2(By, Bx))

# %% Kalman filter initialization
N = 6

# x = [phi, wx, theta, wy, psi, wz]
x = np.zeros(N)

F = np.array([[1., Ts, 0., 0., 0., 0.],
             [0., 1., 0., 0., 0., 0.],
             [0., 0., 1., Ts, 0., 0.],
             [0., 0., 0., 1., 0., 0.],
             [0., 0., 0., 0., 1., Ts],
             [0., 0., 0., 0., 0., 1.]])
P = np.eye(N)
sigma_w = 1.
Q = np.array([[1/4*Ts ** 4, 1/2*Ts ** 3, 0., 0., 0., 0.],
             [1/2*Ts ** 3, Ts ** 2, 0., 0., 0., 0.],
             [0., 0., 1/4*Ts ** 4, 1/2*Ts ** 3, 0., 0.],
             [0., 0., 1/2*Ts ** 3, Ts ** 2, 0., 0.],
             [0., 0., 0., 0., 1/4*Ts ** 4, 1/2*Ts ** 3],
             [0., 0., 0., 0., 1/2*Ts ** 3, Ts ** 2]]) * sigma_w ** 2

# z = [acc_roll, wx, acc_pitch, wy, mag_yaw, wz]
H = np.eye(N)
sigma_r_sq = 0.05 ** 2
R = np.array([[sigma_r_sq, 0., 0., 0., 0., 0.],
             [0., sigma_r_sq, 0., 0., 0., 0.],
             [0., 0.,sigma_r_sq, 0., 0., 0.],
             [0., 0., 0., sigma_r_sq, 0., 0.],
             [0., 0., 0., 0., sigma_r_sq, 0.],
             [0., 0., 0., 0., 0., sigma_r_sq]])

x_arr = np.zeros([N, len(t)])
mag_yaw_corr = 0.
for i in range(len(t)):
    Bx_hat = Bx[i] * np.cos(x[2]) + By[i] * np.sin(x[2]) * np.sin(x[0]) + \
        Bz[i] * np.sin(x[2]) * np.cos(x[0])
    By_hat = By[i] * np.cos(x[0]) - Bz[i] * np.sin(x[0])
    mag_yaw_corr_new = -np.arctan2(By_hat, Bx_hat)
    mag_yaw_corr = np.unwrap(np.array([mag_yaw_corr, mag_yaw_corr_new]))[-1]
    z = np.array([acc_roll[i], wx[i], acc_pitch[i], wy[i], mag_yaw_corr, wz[i]])
    x = F.dot(x)
    P = F.dot(P.dot(F.T)) + Q
    K = P.dot(H.T).dot(np.linalg.inv(H.dot(P.dot(H.T)) + R))
    x += K.dot(z - H.dot(x))
    P = P - K.dot(H.dot(P))
    x_arr[:,i] = x

# %% Plot data

plt.figure(dpi=400, figsize=[6,10])
plt.subplot(3,1,1)
plt.plot(t, np.rad2deg(gyro_roll), label='gyroscope')
plt.plot(t, np.rad2deg(acc_roll), label='accelerometer')
plt.plot(t, np.rad2deg(x_arr[0,:]), label='Kalman')
plt.legend()
plt.grid()
plt.ylabel('Roll angle [deg]')

plt.subplot(3,1,2)
plt.plot(t, np.rad2deg(gyro_pitch), label='gyroscope')
plt.plot(t, np.rad2deg(acc_pitch), label='accelerometer')
plt.plot(t, np.rad2deg(x_arr[2,:]), label='Kalman')
plt.legend()
plt.grid()
plt.ylabel('Pitch angle [deg]')

plt.subplot(3,1,3)
plt.plot(t, np.rad2deg(gyro_yaw), label='gyroscope')
plt.plot(t, np.rad2deg(mag_yaw), label='magnetometer')
plt.plot(t, np.rad2deg(x_arr[4,:]), label='Kalman')
plt.legend()
plt.grid()
plt.ylabel('Yaw angle [deg]')
plt.xlabel('Time [s]')