clearvars; clc;
close all;

%% Actual gains applied to the ball bot
K_vert = [0, -100, -0.03, -2];
K_xy = [0, -0.6];

%% Simulation
% Initial condition [\phi, \theta, \dot{\phi}, \dot{\theta}]

% Data for large angle case
% ...\data_plotting\after_expo_1gearset\report_result_large.csv
% ic_xz = [0; deg2rad(-5.5577); 0; deg2rad(0)];  % initial condition deg
% ic_yz = [0; deg2rad(-0.0630); 0; 0];
% ic_xy = [0; deg2rad(0)];

% Data for small angle case
% ...\data_plotting\after_expo_1gearset\report_result_small.csv
ic_xz = [0; deg2rad(-0.779); 0; deg2rad(0)];  % initial condition deg
ic_yz = [0; deg2rad(2.53); 0; 0];
ic_xy = [0; deg2rad(0)];
T_sim = 2; % s
rate_lim = 9; % Nm/s
T_max = 0.23;

simout = sim(   "LQR_limited_torque", ...
                'StopTime', num2str(T_sim), ...
                'Solver','ode45', ...
                'RelTol','auto', ...
                'AbsTol','auto', ...
                'MaxStep',num2str(T_sim/500));
% Extract the simulation results
px = squeeze(simout.logsout.get('phi_x').Values.Data);
tx = squeeze(simout.logsout.get('theta_x').Values.Data);
dpx = squeeze(simout.logsout.get('dotphi_x').Values.Data);
dtx = squeeze(simout.logsout.get('dottheta_x').Values.Data);

py = squeeze(simout.logsout.get('phi_y').Values.Data);
ty = squeeze(simout.logsout.get('theta_y').Values.Data);
dpy = squeeze(simout.logsout.get('dotphi_y').Values.Data);
dty = squeeze(simout.logsout.get('dottheta_y').Values.Data);

tz = squeeze(simout.logsout.get('theta_z').Values.Data);
dtz = squeeze(simout.logsout.get('dottheta_z').Values.Data);

T1 = squeeze(simout.logsout.get("T1").Values.Data);
T2 = squeeze(simout.logsout.get("T2").Values.Data);
T3 = squeeze(simout.logsout.get("T3").Values.Data);
w1 = simout.logsout.get("psidot_omni_1").Values.Data;
w2 = simout.logsout.get("psidot_omni_2").Values.Data;
w3 = simout.logsout.get("psidot_omni_3").Values.Data;
i1 = T1/(params().K_T * params().i_Gear);
i2 = T2/(params().K_T * params().i_Gear);
i3 = T3/(params().K_T * params().i_Gear);
q1 = trapz(i1); % in Amp-seconds
q2 = trapz(i2); % in Amp-seconds
q3 = trapz(i3); % in Amp-seconds
qtotal = abs(q1) + abs(q2) + abs(q3); % in Amp-seconds

t = simout.get('tout');
%% Read data
data = readtable('report_result_small.csv');
data.Time_us = (data.Time_us - data.Time_us(1))*10^-6;
%% Translate Torques
[Tx_sim, Ty_sim] = Tmotors_to_Tplane(T1,T2,T3);
[Tx, Ty] = Tmotors_to_Tplane(data.motor_torq_1,data.motor_torq_2,data.motor_torq_3);
[Tx_cmd,Ty_cmd] = Tmotors_to_Tplane(data.T1,data.T2,data.T3);

% Top Subplot: Pitch/Attitude
subplot(2,1,1)
plot(data.Time_us(1:400), rad2deg(data.Pitch(1:400)), 'LineWidth', 2)
hold on;
plot(t, rad2deg(ty), '--', 'LineWidth', 2)
hold off;

title('Pitch Angle Comparison', 'FontSize', 14)
xlabel('Time (s)', 'FontSize', 12)
ylabel('Angle (deg)', 'FontSize', 12)
legend('Pitch Data', 'Pitch Sim', 'Location', 'best', 'FontSize', 11)
grid on;
box on;
set(gca, 'FontSize', 11) % Increases axis tick font size

% Bottom Subplot: Torque
subplot(2,1,2)
plot(data.Time_us(1:400), Ty(1:400), 'LineWidth', 2)
hold on;
plot(t, Ty_sim, '--', 'LineWidth', 2)
plot(data.Time_us(1:400), Ty_cmd(1:400), ':', 'LineWidth', 2)
hold off;

title('Torque (Ty) Comparison', 'FontSize', 14)
xlabel('Time (s)', 'FontSize', 12)
ylabel('Torque (Nm)', 'FontSize', 12)
legend('Ty Data', 'Ty Sim', 'Ty Command', 'Location', 'best', 'FontSize', 11)
grid on;
box on;
set(gca, 'FontSize', 11) % Increases axis tick font size