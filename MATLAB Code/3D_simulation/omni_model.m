%% Clean up
clear; clc; close all;


% Input parameters and configurations into simulink model
mdl = "directional_force_fullfric";

% Get model workspace (required here, can't pass obj.workspace)
load_system(mdl);  % ensure loaded
workspace = get_param(mdl, 'ModelWorkspace');
simIn = Simulink.SimulationInput(mdl);
mw1 = workspace;

%%
disp("Loading fullfric simulation")
outputs1 = sim(simIn);
disp("Finished fullfric model")

%% 

% pull data
fx = squeeze(outputs1.fric_resultant.Data(1,:,:));
fy = squeeze(outputs1.fric_resultant.Data(2,:,:));
t = outputs1.fric_resultant.Time;

% Input parameters and configurations into simulink model
mdl = "directional_force_test";

fx_ts = timeseries(fx, t);
fy_ts = timeseries(fy, t);

% Get model workspace (required here, can't pass obj.workspace)
load_system(mdl);  % ensure loaded
workspace = get_param(mdl, 'ModelWorkspace');
simIn = Simulink.SimulationInput(mdl);
mw = workspace;
mw.assignin("fx", fx_ts);
mw.assignin("fy", fy_ts);

%%
disp("Loading directional simulation")
outputs = sim(simIn);
disp("Finished directional model")