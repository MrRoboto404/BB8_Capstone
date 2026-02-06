%% Clean up
clear; clc; close all;


% Input parameters and configurations into simulink model
mdl = "multibody_test";
icxy = [0; pi/2; 0; 0]; % rad and rad/s
icxz = [0; 0.3; 0; -0.7];
icyz = [0; 0.3; 0; 0];
motor_torque = [-5, 5, 0]; % FL FR B legs

% create ballbot sim model
bb8 = ballbot_system(); % make object
bb8 = bb8.initial_conditions(icxy, icxz, icyz, motor_torque); % load ICs
[bb8, simIn] = bb8.create_sim(mdl); % sim model
bb8 = bb8.load_to_sim(mdl); % load params to simulink model

%% Run sim & vis
disp("Loading simulation...")
out = sim(bb8.simIn);

%% MANIPULATE DATA WITHOUT RERUNNING SIM
clc; close all; 
% Get data
t = out.tout;
theta_x = out.theta_x.Data * 180/pi; % rad to deg
phi_x = out.phi_x.Data * 180/pi;

hold on;
plot(t, theta_x, 'b');
plot(t, phi_x, 'r');
legend('\theta_x','\phi_x')
xlabel('Time (s)')
ylabel('Rotation (deg)')
hold off;