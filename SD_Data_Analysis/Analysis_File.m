clc; close all; clear;
format long;

% importing our data
pitch = readmatrix("pitch.csv");
roll = readmatrix("roll.csv");
gyro_x = readmatrix('gyro_x.csv');
gyro_y = readmatrix('gyro_y.csv');
gyro_z = readmatrix('gyro_z.csv');
time = readmatrix('time.csv');

time = time/10^6;

hold on;
plot(time, gyro_x);
plot(time, gyro_y);
plot(time, gyro_z);
ylabel("rad/s");
xlabel("Time (s)");

legend("Gyro X", "Gyro Y", "Gyro Z");