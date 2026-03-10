clearvars; clc;
close all;
% plant and controller
[A,B,C,D] = get_linearized_matrices_vertical(params());
K_vert = get_gains_LQR_vertical(1);
C = [1, 0, 0, 0;
     0, 1, 0, 0;
     0, 0, 0, 0;
     0, 0, 0, 1];
% C2 = [1 0 1 0];
% D2 = zeros(size(C2,1),size(B,2));
% ctrb() and obsv()
control = rank(ctrb(A,B))
observe = rank(obsv(A,C))

%%
% Kalman Filter
Vd = .001 * eye(4);
Vn = diag([0.1 0.01 0.001 0.000001]);

% [L,P,E] = lqe(A,Vd,C,Vd,Vn);
Kf = (lqr(A',C',Vd,Vn))';

sysKF = ss(A-Kf*C,[B Kf], eye(4), 0*[B  Kf]);

% Simulation
% Initial condition
T_max = 2.3;      % Maximum Motor Torque from "Napkin Math"
ic_xz = [0; deg2rad(10); 0; 0];  % initial condition
ic_yz = [0; deg2rad(10); 0; 0];
ic_xy = [0; 0];
T_sim = 5; % s

%% Attempt to fix the moving IMU problem
[A,B,C,D] = get_linearized_matrices_vertical(params());
% Define physical constants (replace with your actual robot params)
g = 9.81; 
r = params().r_ball;  
L = params().l;

% y_accel = theta + (r/g)*phi_ddot + (L/g)*theta_ddot (small angle approx
accel_kinematics_A = (r/g) * A(3,:) + (L/g) * A(4,:);
accel_kinematics_B = (r/g) * B(3,:) + (L/g) * B(4,:);

C(2,:) = C(2,:) + accel_kinematics_A;
D(2,:) = D(2,:) + accel_kinematics_B; % Feedthrough from motor torque!

% Augment to 5 States to catch the Gyroscope Bias
% The new state vector is [phi, theta, phidot, thetadot, gyro_bias]^T
A_aug = [A, zeros(4,1); 
         zeros(1,4), 0];
B_aug = [B; 0];

% Augment C and D
% Add a column to C_orig. 
% Row 4 is the Gyro. It measures the true theta_dot (State 4) PLUS the bias (State 5)
C_aug = [C, [0; 0; 0; 1]]; 
D_aug = D;

% Tune the Filter
Vd_aug = diag([0.001, 0.001, 0.001, 0.001, 1e-6]); % 5x5 process noise
Vn = diag([0.001, 0.1, 0.001, 0.001]);             % 4x4 measurement noise

% Calculate Kalman Gain and build Simulink System
Kf_aug = (lqr(A_aug', C_aug', Vd_aug, Vn))';
sysKF_aug = ss(A_aug - Kf_aug * C_aug, [B_aug, Kf_aug], eye(5), zeros(5, 1 + size(C_aug, 1)));