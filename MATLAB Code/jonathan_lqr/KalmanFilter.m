clearvars; clc;
close all;
% plant and controller
[A,B,C,D] = get_linearized_matrices_vertical(params());
K_vert = get_gains_LQR_vertical(1);
C = [1, 0, 0, 0;
     0, 0, 0, 0;
     0, 0, 1, 0;
     0, 0, 0, 0];
% C2 = [1 0 1 0];
% D2 = zeros(size(C2,1),size(B,2));
% ctrb() and obsv()
control = rank(ctrb(A,B))
observe = rank(obsv(A,C))
%%
% Kalman Filter
Vd = .001 * eye(4);
Vn = .1;

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