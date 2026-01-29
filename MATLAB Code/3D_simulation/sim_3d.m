% Last updated: 1/28

% Purpose: run simulink models via typical matlab file

% Clean up
clear; clc; close all;

% Run linear model------------------------------------------------------

% Construct & load params
[A_yz, B_yz, C_yz, D_yz, ~] = make_ballbot();
ics_yz = [0; -0.3; 0; 0]; % initial conditions in minimal coords


% Input parameters into simulink model
mdl = "main_sim";
simIn = Simulink.SimulationInput(mdl);

set_param(mdl, "EnablePacing", "on"); % slow down pacing to file: 0.8x
simIn = setModelParameter(simIn, "StopTime", "10"); % configure run time (s)

simIn = simIn.setVariable("A", A_yz);
simIn = simIn.setVariable("B", B_yz);
simIn = simIn.setVariable("C", C_yz);
simIn = simIn.setVariable("D", D_yz);
simIn = simIn.setVariable("ics_yz", ics_yz);

% Run sim & vis
out = sim(simIn);
