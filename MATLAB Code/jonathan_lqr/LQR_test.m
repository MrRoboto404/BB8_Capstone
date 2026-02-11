clearvars; clc;
% close all;

% %% This script is the first attempt to implement LQR control %%

% Parameters
bb = params;
[A,B,C,D] = get_linearized_matrices(bb);
 %m
% LQR Controller
Q = [1  0   0   0;              % State weighting matrix
     0  1   0   0;
     0  0   1   0;
     0  0   0   1];
R = 1;                  % Control weighting matrix
[K, S, E] = lqr(A, B, Q, R);    % Compute the state feedback gain using LQR

Kr = -1 / (C * inv(A - B*K) * B);

% %%
% Simulate
T_sim = 5; % s
% [\phi, \theta, \dot{\phi}, \dot{\theta}]
ic = [0; 0.175; 0; 0];  % initial condition
ref = [0;0; 0; 0];     % reference
%%
simout = sim(   'LQR_sim', ...
                'Solver','ode45', ...
                'RelTol','auto', ...
                'AbsTol','auto', ...
                'MaxStep','T_sim/500');
% Extract the simulation results
phi = squeeze(simout.logsout.get('phi').Values.Data);
theta = squeeze(simout.logsout.get('theta').Values.Data);
psi_virt = squeeze(simout.logsout.get('psi').Values.Data);
dotphi = squeeze(simout.logsout.get('dotphi').Values.Data);
dottheta = squeeze(simout.logsout.get('dottheta').Values.Data);
dotpsi_virt = squeeze(simout.logsout.get('dotpsi').Values.Data);

% Motor speeds
rpm = 30/pi;
psidot_omni_1 = dotpsi_virt * cos(bb.alpha) * rpm;
psidot_omni_2 = -0.5 * dotpsi_virt * cos(bb.alpha) * rpm;
psidot_omni_3 = psidot_omni_2;

t = simout.get('tout');
u = simout.logsout.get("u").Values.Data;
uvirt = squeeze(u)./bb.i_Gear; % virt wheel torque


[T1, T2, T3] = real_motor_torques_from_virtual(uvirt,0,0,bb);
% 
%figure('Name', 'Original', 'NumberTitle', 'off')
figure;
subplot(5,1,1)
plot(t,squeeze(theta))
legend('theta')
ylabel('Angle (rad)')
grid

subplot(5,1,2)
plot(t,squeeze(phi),t,squeeze(theta))
legend('phi','theta')
ylabel('Angle (rad)')
grid

subplot(5,1,3)
plot(t,squeeze(dotphi),t,squeeze(dottheta))
ylabel('Angle rate (rad/s)')
legend('dotphi','dottheta')
grid

subplot(5,1,4)
plot(t,T1,t,T2,'g',t,T3,'r--')
legend('T1','T2','T3')
xlabel('Time (s)')
ylabel('Motor Torque (Nm)')
grid

subplot(5,1,5)
plot(t,psidot_omni_1,t,psidot_omni_2,'g',t,psidot_omni_3,'r--')
legend('Omni 1','Omni 2','Omni 3')
xlabel('Time (s)')
ylabel('Motor Speed (rpm)')
grid
%%
% theta_max(abs(min(theta)))