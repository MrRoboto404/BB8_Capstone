% Last updated: 1/28

% Purpose: run simulink models via typical matlab file

% Clean up
clear; clc; close all;

% Run linear model------------------------------------------------------

% Construct & load params
[A, B, C, D, ~] = make_ballbot();
ics = [0; 0.3; 0; 0]; % initial conditions in minimal coords


% Input parameters into simulink model
mdl = "main_sim";
simIn = Simulink.SimulationInput(mdl);

set_param(mdl, "EnablePacing", "on"); % slow down pacing to file: 0.8x
simIn = simIn.setVariable("A", A);
simIn = simIn.setVariable("B", B);
simIn = simIn.setVariable("C", C);
simIn = simIn.setVariable("D", D);
simIn = simIn.setVariable("ics", ics);

% Run sim & vis
out = sim(simIn);