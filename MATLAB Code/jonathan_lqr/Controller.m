clearvars; clc;
% close all;

% %% This script is the second attempt to implement LQR control for all 3 planes %%

% Parameters
bb = params;
test = ['Test I (Note: \beta = 0)'];
% %% LQR Controller
% %% Fullstate feedback
% yz and xz planes
[A_vert,B_vert,C_vert,D_vert] = get_linearized_matrices_vertical(bb);
Q = diag([2 15 1 5]);              % State weighting matrix
R = 10;                  % Control weighting matrix
[K_vert, S_vert, E_vert] = lqr(A_vert,B_vert, Q, R);    % Compute the state feedback gain using LQR

% xy planes
[A_xy,B_xy,C_xy,D_xy] = statespace_xy(bb);
Q = diag([1 1]);              % State weighting matrix
R = 1;                  % Control weighting matrix
[K_xy, S_xy, E_xy] = lqr(A_xy,B_xy, Q, R);    % Compute the state feedback gain using LQR
K_vert(1) = 0;    
% %% Initial conditions
% [\phi, \theta, \dot{\phi}, \dot{\theta}]
ic_xz = [0; deg2rad(10); 0; 0];  % initial condition
ic_yz = [0; deg2rad(10); 0; 0];
ic_xy = [0; 0];
% %% Velocity control (make robot move at constant speed)
% % Vertical Planes
% % Only feedback [theta; dot(phi), dot(theta)]
% A_vert_vel = A_vert(2:4,2:4);             % Remove the row and collumn contain phi
% B_vert_vel = B_vert(2:4);
% C_vert_vel = [0 1 0];
% K_vert_vel = K_vert(2:4);
% N_vert = -1 / (C_vert_vel * inv(A_vert_vel - B_vert_vel*K_vert_vel) * B_vert_vel);   % DC gain for if the ref is the output
% 
% dotphi_x_ref = 20;
% dotphi_y_ref = 0;
% K_vert(1) = 0;                              % Disable phi feedback


%% Simulation
T_sim = 5; % s
%% Nonlinear
simout = sim(   'LQR_nonlinear_sim', ...
                'Solver','ode45', ...
                'RelTol','auto', ...
                'AbsTol','auto', ...
                'MaxStep','T_sim/500');
% Extract the simulation results
phi_x = squeeze(simout.logsout.get('phi_x').Values.Data);
theta_x = squeeze(simout.logsout.get('theta_x').Values.Data);
dotphi_x = squeeze(simout.logsout.get('dotphi_x').Values.Data);
dottheta_x = squeeze(simout.logsout.get('dottheta_x').Values.Data);

phi_y = squeeze(simout.logsout.get('phi_y').Values.Data);
theta_y = squeeze(simout.logsout.get('theta_y').Values.Data);
dotphi_y = squeeze(simout.logsout.get('dotphi_y').Values.Data);
dottheta_y = squeeze(simout.logsout.get('dottheta_y').Values.Data);

theta_z = squeeze(simout.logsout.get('theta_z').Values.Data);
dottheta_z = squeeze(simout.logsout.get('dottheta_z').Values.Data);

T1 = simout.logsout.get("T1").Values.Data;
T2 = simout.logsout.get("T2").Values.Data;
T3 = simout.logsout.get("T3").Values.Data;
omega_1 = simout.logsout.get("psidot_omni_1").Values.Data;
omega_2 = simout.logsout.get("psidot_omni_2").Values.Data;
omega_3 = simout.logsout.get("psidot_omni_3").Values.Data;

t = simout.get('tout');


% figure('Name', 'Original', 'NumberTitle', 'off')
figure
subplot(3,3,1)
plot(t,theta_x)
legend('theta_x')
ylabel('Angle (rad)')
title('yz plane')
grid

subplot(3,3,2)
plot(t,theta_y)
legend('theta_y')
ylabel('Angle (rad)')
title('xz plane')
grid

subplot(3,3,3)
plot(t,theta_z)
legend('theta_z')
ylabel('Angle (rad)')
title('xy plane')
grid

subplot(3,3,4)
plot(t,phi_x)
legend('phi_x',Location='southeast')
ylabel('Angle (rad)')
grid

subplot(3,3,5)
plot(t,phi_y)
legend('phi_y',Location='southeast')
ylabel('Angle (rad)')
grid

subplot(3,3,7)
plot(t,dotphi_x,t,dottheta_x)
ylabel('Angle rate (rad/s)')
legend('dotphi x','dottheta x')
grid

subplot(3,3,8)
plot(t,dotphi_y,t,dottheta_y)
ylabel('Angle rate (rad/s)')
legend('dotphi y','dottheta y')
grid

subplot(3,3,6)
plot(t,dottheta_z)
ylabel('Angle rate (rad/s)')
legend('dottheta z')
grid

figure
subplot(2,1,1)
plot(t,T1,t,T2,'g',t,T3,'r--')
legend('T1','T2','T3')
xlabel('Time (s)')
ylabel('Motor Torque (Nm)')
title('Motor spec');
grid

subplot(2,1,2)
plot(t,omega_1,t,omega_2,'g',t,omega_3,'r--')
legend('omega 1','omega 2','omega 3')
xlabel('Time (s)')
ylabel('Motor speed (rpm)')
grid

% %%
% simout = sim(   'LQR_linearized_sim', ...
%                 'Solver','ode45', ...
%                 'RelTol','auto', ...
%                 'AbsTol','auto', ...
%                 'MaxStep','T_sim/500');
% 
% phi_x_l = squeeze(simout.logsout.get('phi_x').Values.Data);
% theta_x_l = squeeze(simout.logsout.get('theta_x').Values.Data);
% dotphi_x_l = squeeze(simout.logsout.get('dotphi_x').Values.Data);
% T1_l = simout.logsout.get("T1").Values.Data;
% omega_1_l = simout.logsout.get("psidot_omni_1").Values.Data;
% 
% figure
% subplot(5,1,1)
% plot(t,theta_x,t,theta_x_l,'--')
% legend('Nonlinear','Linearized')
% ylabel('theta_x (rad)')
% title('xz plane linear vs nonlinear')
% grid
% 
% subplot(5,1,2)
% plot(t,phi_x,t,phi_x_l,'--')
% legend('Nonlinear','Linearized')
% ylabel('phi_x (rad)')
% grid
% 
% subplot(5,1,3)
% plot(t,dotphi_x,t,dotphi_x_l,'--')
% legend('Nonlinear','Linearized')
% ylabel('dotphi_x (rad/s)')
% grid
% 
% subplot(5,1,4)
% plot(t,T1,t,T1_l,'--')
% legend('Nonlinear','Linearized')
% ylabel('Motor Torque 1 (Nm)')
% grid
% 
% subplot(5,1,5)
% plot(t,omega_1,t,omega_1_l,'--')
% legend('Nonlinear','Linearized')
% ylabel('Motor speed 1 (rpm)')
% grid
% 

%%
% theta_max(abs(min(theta)))