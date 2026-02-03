% Last updated: 1/28

% Purpose: run simulink models via typical matlab file

% Clean up
clear; clc; close all;

% Run linear model------------------------------------------------------



% Input parameters and configurations into simulink model
mdl = "main_sim";
icxy = [0; 0; 0; 0];
icxz = [0; 0.3; 0; -0.7];
icyz = [0; 0; 0; 0];

% create object off of ICs
%[bb8, simIn] = ballbot_system(mdl, icxy, icxz, icyz);
bb8 = ballbot_system(icxy, icxz, icyz);
[bb8, simIn] = bb8.create_sim(mdl);
% simulation configurations
bb8 = bb8.load_to_sim(mdl); % load params to simulink model

%% Run sim & vis
disp("Loading simulation...")
out = sim(bb8.simIn);

% Get data
t = out.tout;
theta_rot = out.theta_rot; % rad
%% MANIPULATE DATA WITHOUT RERUNNING SIM

theta_x = squeeze(out.theta_rot(:, 1, :)); % rad
theta_x = theta_x * 180/pi; % deg

% plot data
plot(t, theta_x)
xlabel('Time (s)');
ylabel('\theta_x, (deg)');