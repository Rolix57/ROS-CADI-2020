clc
clear
close all

load('sensorlog.mat')

%% Accelerometer [m/s^2]
t_a = Acceleration.Timestamp.Minute*60+Acceleration.Timestamp.Second;
t_a = t_a-t_a(1);
ax = Acceleration.X;
ay = Acceleration.Y;
az = Acceleration.Z;

%% Gyroscope [rad/s]
t_w = AngularVelocity.Timestamp.Minute*60+AngularVelocity.Timestamp.Second;
t_w = t_w-t_w(1);
wx = AngularVelocity.X;
wy = AngularVelocity.Y;
wz = AngularVelocity.Z;

%% Magnetometer [T]
t_m = MagneticField.Timestamp.Minute*60+MagneticField.Timestamp.Second;
t_m = t_m-t_m(1);
Bx = MagneticField.X;
By = MagneticField.Y;
Bz = MagneticField.Z;



return

%% Resample signals
fs = 100;
ax = resample(ax,t_a,fs);
ay = resample(ay,t_a,fs);
az = resample(az,t_a,fs);

wx = resample(wx,t_w,fs);
wy = resample(wy,t_w,fs);
wz = resample(wz,t_w,fs);

Bx = resample(Bx,t_m,fs);
By = resample(By,t_m,fs);
Bz = resample(Bz,t_m,fs);

len = min([length(ax),length(wx),length(Bx)]);
t = (0:len-1)'/fs;
ax = ax(1:len);
ay = ay(1:len);
az = az(1:len);
wx = wx(1:len);
wy = wy(1:len);
wz = wz(1:len);
Bx = Bx(1:len);
By = By(1:len);
Bz = Bz(1:len);

data = [t,ax,ay,az,wx,wy,wz,Bx,By,Bz];

csvwrite('sensorlog.csv', data);