clear; clc; close all;
data = readtable("Test1.txt");
%% Mechanical Parameters
% Replace these with your actual lab parameters/measurements
n = 27;             % Gear ratio
% J_m = 1e-4;       % Motor inertia (N.m.s^2/rad) (0.5 - 5) e-4
% J_L = .048;        % Load inertia (N.m.s^2/rad)
B_eq = 1e-2;       % Equivalent viscous friction (N.m.s/rad)

% Effective inertia seen by the motor
% J_eq = J_m*n^2 + J_L
% 0.1209
J_eq = 0.02

%% Design Pole
Ts = 0.2;                           % s ... design settling time
psi_R = -4 / Ts;                    % real part of design point psi
Z = -20;                            % compensator zero

%% Defining system model 
% Input: Torque (Nm), Output: Motor Velocity (rad/s)
GP = tf([n^2], [J_eq, B_eq]);       % plan TF
H = tf([1], [1]);                   % feedback transfer function

%% Proportional controller
figure(1);
K_test = linspace(0, 1, 5000); 
rlocus(GP * H,K_test);                     
title('Uncompensated Root Locus');
xlim([-40,0]);

K1 = 0.000547;       % CHANGE THIS: see gain at Z = -20 on fig 1

%% Integral compensation
s = tf([1, 0], [1]);                
CI_sans = (s-Z) / s;               % compensator sans gain

% Compensated root locus
figure(2);
rlocus(CI_sans * K1 * GP * H);
title('Compensated Root Locus');
    
K2 = 2;           %  CHANGE THIS: see gain at Z = -20 on fig 1
GC = K1 * K2 * CI_sans;             % Continuous PI controller

%% Change plant
% To se the result of the controller on different plan
% J_eq = J_L + (n^2)*J_m
% GP = tf([n^2], [J_eq, B_eq]);       % plan TF
%% Closing the loop and discretization
GCL = GC * GP / (1 + GC * GP * H);  % closed-loop tf   
T = 1/160;                          % s ... sample period
GCd = c2d(GC, T, 'Tustin');         % using Tustin's method
GPd = c2d(GP, T, 'Tustin');
Hd  = c2d(H, T, 'Tustin');
GCLd = GCd * GPd / (1 + GCd * GPd * Hd); % discrete closed-loop tf

%% Output Gains
KP = K1 * K2;
KI = -Z * KP;
fprintf('\n--- Gains for Teensy C++ Code ---\n');
fprintf('float Kp = %.5f;\n', KP);
fprintf('float Ki = %.5f;\n', KI);
fprintf('---------------------------------\n\n');

%% Simulation and evaluation
V_R_rpm = 20;                        
V_R_rads = V_R_rpm * 2 * pi / 60;     % rad/s target step
V_init_rads = -(0 * 2 * pi / 60);   % rad/s initial state
t = 0:T:1.25;                         

Omega = V_R_rads * step(GCL, t) + V_init_rads;      % continuous response
Omegad = V_R_rads * step(GCLd, t) + V_init_rads;    % discrete response

U_R = GCd / (1 + GCd * GPd * Hd);     % Control effort cltf
u_torque = 27 * V_R_rads * step(U_R, t);   % Torque command in Nm

time_offset = data.Time(1);
figure(3);
subplot(2,1,1);
plot(t, Omega*27, 'LineWidth', 1.5); hold on;
plot(t, Omegad*27, '--', 'LineWidth', 1.5);
plot(data.Time-time_offset,data.rad_a_1,'.-','LineWidth', 1.5)
title('Motor Velocity Response Simulation');
xlabel('Time (s)'); ylabel('Velocity (rad/s)');
legend('Continuous', 'Discrete (Tustin)','Actual', 'Location', 'Southeast');
grid on;

subplot(2,1,2);
plot(t, u_torque, 'LineWidth', 1.5); hold on;
plot(data.Time - time_offset, data.T_cmd_1,'LineWidth', 1.5)

title('Control Output Torque Command (ODrive Input)');
xlabel('Time (s)'); ylabel('Torque (Nm)');
legend('Design', 'Actual', 'Location', 'Northeast');
grid on;