clearvars; clc;
% close all;

% %% This script is the second attempt to implement LQR control for all 3 planes %%

% Parameters
bb = params;
test = ['Test I (Note: \beta = 0)'];
%% LQR Controller
%% Fullstate feedback
% yz and xz planes
[A_vert,B_vert,C_vert,D_vert] = get_linearized_matrices_vertical(bb);
Q = diag([1 1 1 1]);              % State weighting matrix
R = 1;                  % Control weighting matrix
[K_vert, S_vert, E_vert] = lqr(A_vert,B_vert, Q, R);    % Compute the state feedback gain using LQR
% xy planes
[A_xy,B_xy,C_xy,D_xy] = statespace_xy(bb);
Q = diag([1 1]);              % State weighting matrix
R = 1;                  % Control weighting matrix
[K_xy, S_xy, E_xy] = lqr(A_xy,B_xy, Q, R);    % Compute the state feedback gain using LQR
%% Velocity control (make robot move at constant speed)
% Vertical Planes
% Only feedback [theta; dot(phi), dot(theta)]
A_vert_vel = A_vert(2:4,2:4);             % Remove the row and collumn contain phi
B_vert_vel = B_vert(2:4);
C_vert_vel = [0 1 0];
K_vert_vel = K_vert(2:4);
N_vert = -1 / (C_vert_vel * inv(A_vert_vel - B_vert_vel*K_vert_vel) * B_vert_vel);   % DC gain for if the ref is the output

dotphi_x_ref = 0.2;
dotphi_y_ref = 0.1;
K_vert(1) = 0;                  % Disable phi feedback
%% Simulate
T_sim = 5; % s
% [\phi, \theta, \dot{\phi}, \dot{\theta}]
ic_xz = [0; 0.175; 0; 0];  % initial condition
ic_yz = [0; 0.3; 0; 0];
ic_xy = [pi; 0];

%%
simout = sim(   'LQR_test_sim', ...
                'Solver','ode45', ...
                'RelTol','auto', ...
                'AbsTol','auto', ...
                'MaxStep','T_sim/500');
% Extract the simulation results
phi_x = squeeze(simout.logsout.get('phi_x').Values.Data);
theta_x = squeeze(simout.logsout.get('theta_x').Values.Data);
% psi_virt_x = squeeze(simout.logsout.get('psi_x').Values.Data);
dotphi_x = squeeze(simout.logsout.get('dotphi_x').Values.Data);
dottheta_x = squeeze(simout.logsout.get('dottheta_x').Values.Data);
% dotpsi_virt_x = squeeze(simout.logsout.get('dotpsi_x').Values.Data);

phi_y = squeeze(simout.logsout.get('phi_y').Values.Data);
theta_y = squeeze(simout.logsout.get('theta_y').Values.Data);
psi_virt_y = squeeze(simout.logsout.get('psi_y').Values.Data);
dotphi_y = squeeze(simout.logsout.get('dotphi_y').Values.Data);
dottheta_y = squeeze(simout.logsout.get('dottheta_y').Values.Data);
dotpsi_virt_y = squeeze(simout.logsout.get('dotpsi_y').Values.Data);

phi_z = squeeze(simout.logsout.get('phi_z').Values.Data);
theta_z = squeeze(simout.logsout.get('theta_z').Values.Data);
% psi_virt_z = squeeze(simout.logsout.get('psi_z').Values.Data);
dotphi_z = squeeze(simout.logsout.get('dotphi_z').Values.Data);
dottheta_z = squeeze(simout.logsout.get('dottheta_z').Values.Data);
% dotpsi_virt_z = squeeze(simout.logsout.get('dotpsi_z').Values.Data);


t = simout.get('tout');
Tx = simout.logsout.get("Tx").Values.Data;
Ty = simout.logsout.get("Ty").Values.Data;
Tz = simout.logsout.get("Tz").Values.Data;

T1 = simout.logsout.get("T1").Values.Data;
T2 = simout.logsout.get("T2").Values.Data;
T3 = simout.logsout.get("T3").Values.Data;
% 
%figure('Name', 'Original', 'NumberTitle', 'off')
figure;
subplot(3,3,1)
plot(t,theta_x)
legend('theta')
ylabel('Angle (rad)')
title(test)
grid
subplot(3,3,2)
plot(t,theta_y)
legend('theta')
ylabel('Angle (rad)')
title(test)
grid
subplot(3,3,3)
plot(t,squeeze(theta))
legend('theta')
ylabel('Angle (rad)')
title(test)
grid
subplot(3,3,2)
plot(t,squeeze(phi),t,squeeze(theta))
legend('phi','theta')
ylabel('Angle (rad)')
grid

% subplot(5,1,3)
% plot(t,squeeze(dotphi),t,squeeze(dottheta))
% ylabel('Angle rate (rad/s)')
% legend('dotphi','dottheta')
% grid

subplot(5,1,4)
plot(t,T1,t,T2,'g',t,T3,'r--')
legend('T1','T2','T3')
xlabel('Time (s)')
ylabel('Motor Torque (Nm)')
grid

% subplot(5,1,5)
% plot(t,psidot_omni_1,t,psidot_omni_2,'g',t,psidot_omni_3,'r--')
% legend('Omni 1','Omni 2','Omni 3')
% xlabel('Time (s)')
% ylabel('Motor Speed (rpm)')
% grid
%%
% theta_max(abs(min(theta)))