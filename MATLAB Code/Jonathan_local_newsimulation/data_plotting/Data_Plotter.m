clc; clear;
close all;
data = readtable('192728374_motor_data.csv');
data.Time_us = (data.Time_us - data.Time_us(1))*10^-6;
Tx = -(cos(params().alpha)*(data.T1*cos(params().beta)) ...
    - data.T2*sin(params().beta + pi/6) ...
    + data.T3*sin(params().beta - pi/6));

Ty = cos(params().alpha)*(-data.T1*sin(params().beta)) ...
- data.T2*cos(params().beta + pi/6) + data.T3*cos(params().beta - pi/6);
%%
figure
subplot(2,1,1)
plot(data.Time_us(1:400), rad2deg(data.Roll(1:400)), data.Time_us(1:400), ...
    rad2deg(data.Pitch(1:400)))
legend("Roll (rad)","Pitch (rad)")
grid
subplot(2,1,2)
plot(data.Time_us(1:400), Tx(1:400), data.Time_us(1:400), Ty(1:400))
legend("Tx (Nm)", "Ty (Nm)")
grid
%%
figure
subplot(7,1,1)
plot(data.Time_us, data.Roll, data.Time_us, data.Pitch)
legend("Roll (rad)","Pitch (rad)")
subplot(7,1,3)
plot(data.Time_us, data.GyroX, data.Time_us, data.GyroY, data.Time_us, data.GyroZ)
legend("Roll rate (rad/s)", "Pitch rate (rad/s)", "Yaw rate (rad/s)")
subplot(7,1,4)
plot(data.Time_us, data.PhiDX, data.Time_us, data.PhiDY)
legend("Phi dot X (rad/s)", "Phi dot Y (rad/s)")
subplot(7,1,5)
plot(data.Time_us, data.T1, data.Time_us, data.T2, data.Time_us, data.T3)
legend("T1 (Nm)", "T2 (Nm)", "T3 (Nm)")
subplot(7,1,2)
plot(data.Time_us, Tx, data.Time_us, Ty)
legend("Tx (Nm)", "Ty (Nm)")


subplot(7,1,6)
plot(data.Time_us, data.motor_vel_1,data.Time_us, data.motor_vel_2,data.Time_us, data.motor_vel_3)
subplot(7,1,7)
plot(data.Time_us, data.motor_pos_1,data.Time_us, data.motor_pos_2,data.Time_us, data.motor_pos_3)

%%
figure
plot(data.Time_us, data.Roll, data.Time_us, Tx./10)
legend("Roll","Tx (Nm)")

