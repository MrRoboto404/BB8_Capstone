%% Clean up
clear; clc; close all;


% Input parameters and configurations into simulink model
mdl = "multibody_test";
icxy = [0; deg2rad(0); 0; deg2rad(0)]; % rad and rad/s
icxz = [0; deg2rad(10); 0; deg2rad(0)];
icyz = [0; deg2rad(0); 0; deg2rad(0)];
motor_torque = [0, 0, 0]; % FL FR B legs

% get controller specs from jonathan's controller
K_vert = get_gains_LQR_vertical(1);
K_xy = get_gains_LQR_xy;
T_max = 5;      % Maximum Motor Torque
ic_xz = icxz;
ic_yz = icyz;
ic_xy = icxy;

% create ballbot sim model
bb8 = ballbot_system(); % make object, complete with matrices and parameters
bb8 = bb8.initial_conditions(icxy, icxz, icyz, motor_torque); % load ICs
[bb8, simIn] = bb8.create_sim(mdl); % sim model
bb8 = bb8.load_to_sim(mdl); % load params to simulink model

%% Run sim & vis
disp("Loading simulation...")
outputs = sim(bb8.simIn);

%% MANIPULATE DATA WITHOUT RERUNNING SIM
clc; close all; 

% Get data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t = outputs.tout;

%___Rotational, in rads___
phi_x = (outputs.phi_x.Data);
theta_x = (outputs.theta_x.Data);
phidot_x = (outputs.phi_dot_x.Data);
thetadot_x = (outputs.theta_dot_x.Data);

%___Translational, in meters___
body_y = outputs.body_posy.Data;
body_z = outputs.body_posz.Data;
ball_y = outputs.ball_posy.Data;
ball_z = outputs.ball_posz.Data;



% Graph data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure();

hold on;
subplot(1, 4, 1)
plot(t, phi_x, 'r', t, theta_x, 'c');
grid on;
ylabel("Displacement (rad)");
xlabel("Time (s)");
legend("\phi_x", "\theta_x");
title("Angular Positions")

subplot(1, 4, 2)
plot(t, phidot_x, 'r', t, thetadot_x, 'c');
grid on;
ylabel("Speed (rad/s)");
xlabel("Time (s)");
legend("$\dot{\phi}_x$", "$\dot{\theta}_x$", 'Interpreter', 'latex');
title("Angular Velocities")

subplot(1, 4, 3)
plot(t, ball_y, 'y', t, body_y, 'g');
grid on;
ylabel("Distance (m)");
xlabel("Time (s)");
legend("Ball", "Body");
title("Y Positions")

subplot(1, 4, 4)
plot(t, ball_z, 'y', t, body_z, 'g');
grid on;
ylabel("Distance (m)");
xlabel("Time (s)");
legend("Ball", "Body");
title("Z Positions")


sgtitle("Simulation Results of YZ-Plane Plots for ICs of: " + mat2str(icyz));
hold off;