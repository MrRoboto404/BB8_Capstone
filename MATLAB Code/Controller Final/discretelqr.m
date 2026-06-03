% Discrete controller
%% Vertical planes
clear;
% clc;
Ts = 1/200;
[A,B,C,D] = get_linearized_matrices_vertical(params());
sys_continuous = ss(A, B, C, D);
sys_discrete = c2d(sys_continuous, Ts, 'zoh'); % Zero-Order Hold conversion
% Get your discrete A and B matrices
[Ad, Bd] = ssdata(sys_discrete);
Q = diag([2 150 5 1]);               % State weighting matrix
R = 1;                              % Control weighting matrix
% Solve for the Discrete LQR Gain
% Use the same Q and R weights you used for continuous
K_vert_discrete = lqrd(A, B, Q, R, Ts)
%% xy-plane
[A,B,C,D] = get_statespace_xy(params());
sys_continuous = ss(A, B, C, D);
sys_discrete = c2d(sys_continuous, Ts, 'zoh'); % Zero-Order Hold conversion
Q = diag([1 1]);                    % State weighting matrix
R = 1;                              % Control weighting matrix
% Solve for the Discrete LQR Gain
% Use the same Q and R weights you used for continuous
K_xy_discrete = lqrd(A, B, Q, R, Ts)


% %% Stability check
% % 1. Create the Closed-Loop Matrix
% % This represents the system behavior when the feedback is active
% A_closed = Ad - Bd * K_vert_discrete;
% 
% % 2. Calculate the Eigenvalues
% e = eig(A_closed);
% 
% % 3. Check Magnitude (The "Unit Circle" Test)
% magnitudes = abs(e);
% 
% % 4. Display Results
% disp('Closed-loop Eigenvalues:');
% disp(e);
% disp('Magnitudes (Must be < 1.0 for stability):');
% disp(magnitudes);
% 
% % 5. Visualize on a Z-plane plot
% pzmap(ss(A_closed, Bd, eye(size(Ad)), 0, Ts));
% grid on;