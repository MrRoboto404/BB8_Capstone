% Last updated: 1/28

% Purpose: run simulink models via typical matlab file

% Clean up
clear; clc; close all;

% Run linear model------------------------------------------------------



% Input parameters and configurations into simulink model
mdl = "main_sim";
simIn = load_simparams(mdl);

set_param(mdl, "EnablePacing", "on"); % slow down pacing to file: 0.8x
simIn = setModelParameter(simIn, "StopTime", "3"); % configure run time (s)

%% Run sim & vis
out = sim(simIn);

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