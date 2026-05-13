clearvars; clc;
close all;

% This script is the second attempt to implement LQR control for all 3 planes %%
gain_mod = 1;
rate_lim = 1.2; % Nm/s
K_vert = get_gains_LQR_vertical(0)
% K_vert(2) = -250
K_xy = get_gains_LQR_xy
K_xy(1) = 0;
% K_xy = [0, 0]
% Initial conditions
%  [\phi, \theta, \dot{\phi}, \dot{\theta}]

% Simulation
% Initial condition
% T_max = 6.9;      % Maximum Motor Torque from "Napkin Math"
ic_xz = [0; deg2rad(10); 0; 0];  % initial condition deg
ic_yz = [0; deg2rad(10); 0; 0];
ic_xy = [0; 0];
T_sim = 5; % s
%%
T_max = 5000;
% Nonlinear
simulink_filename = ["LQR_nonlinear_sim","LQR_2"];
graph_title = ["Nonlinear Unlimited Torque","Nonlinear Limited Torque"];
simout = sim(   "LQR_2", ...
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

% Current analysis

disp("___Current Results: " + graph_title(1) + "___")
fprintf("Total Charge Used: %f (A*h)\n", qtotal/3600)


%________________________PLOTS________________________
% figure('Name', 'Original', 'NumberTitle', 'off')
figure
sgtitle(graph_title(1));
subplot(3,3,1)
plot(t,tx)
legend('theta_x')
ylabel('Angle (rad)')
title('yz plane')
grid

subplot(3,3,2)
plot(t,ty)
legend('theta_y')
ylabel('Angle (rad)')
title('xz plane')
grid

subplot(3,3,3)
plot(t,tz)
legend('theta_z')
ylabel('Angle (rad)')
title('xy plane')
grid

subplot(3,3,4)
plot(t,px)
legend('phi_x',Location='southeast')
ylabel('Angle (rad)')
grid

subplot(3,3,5)
plot(t,py)
legend('phi_y',Location='southeast')
ylabel('Angle (rad)')
grid

subplot(3,3,7)
plot(t,dpx,t,dtx)
ylabel('Angle rate (rad/s)')
legend('dotphi x','dottheta x')
grid

subplot(3,3,8)
plot(t,dpy,t,dty)
ylabel('Angle rate (rad/s)')
legend('dotphi y','dottheta y')
grid

subplot(3,3,6)
plot(t,dtz)
ylabel('Angle rate (rad/s)')
legend('dottheta z')
grid

figure % Torque and velocity and current responses
sgtitle(graph_title(1));
subplot(3,1,1)
plot(t,T1,t,T2,'g',t,T3,'r--')
legend('T_1','T_2','T_3')
xlabel('Time (s)')
ylabel('Motor Torque (Nm)')
title('Motor spec');
grid

subplot(3, 1, 2)
plot(t, i1, t, i2, 'g', t, i3, 'r--')
legend('I_1', 'I_2', 'I_3');
xlabel('Time (s)')
ylabel('Current (A)');
title('Current Response')
grid


subplot(3,1,3)
plot(t,w1,t,w2,'g',t,w3,'r--')
legend('\omega_1','\omega_2','\omega_3')
xlabel('Time (s)')
ylabel('Motor speed (rpm)')
grid
%% GET DATA
t1 = t;
tx1 = tx;
px1 = px;
dtx1 = dtx;
dpx1 = dpx;
T11 = T1; T21 = T2; T31 = T3;
%% --------------------------------------------
T_max = 6.9;
% simulink_filename = ["LQR_nonlinear_sim","LQR_2"];
simout = sim(   "LQR_2", ...
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

% Current analysis

disp("___Current Results: " + graph_title(2) + "___")
fprintf("Total Charge Used: %f (A*h)\n", qtotal/3600)


%________________________PLOTS________________________
% figure('Name', 'Original', 'NumberTitle', 'off')
figure
sgtitle(graph_title(2));
subplot(3,3,1)
plot(t,tx)
legend('theta_x')
ylabel('Angle (rad)')
title('yz plane')
grid

subplot(3,3,2)
plot(t,ty)
legend('theta_y')
ylabel('Angle (rad)')
title('xz plane')
grid

subplot(3,3,3)
plot(t,tz)
legend('theta_z')
ylabel('Angle (rad)')
title('xy plane')
grid

subplot(3,3,4)
plot(t,px)
legend('phi_x',Location='southeast')
ylabel('Angle (rad)')
grid

subplot(3,3,5)
plot(t,py)
legend('phi_y',Location='southeast')
ylabel('Angle (rad)')
grid

subplot(3,3,7)
plot(t,dpx,t,dtx)
ylabel('Angle rate (rad/s)')
legend('dotphi x','dottheta x')
grid

subplot(3,3,8)
plot(t,dpy,t,dty)
ylabel('Angle rate (rad/s)')
legend('dotphi y','dottheta y')
grid

subplot(3,3,6)
plot(t,dtz)
ylabel('Angle rate (rad/s)')
legend('dottheta z')
grid

figure % Torque and velocity and current responses
sgtitle(graph_title(2));
subplot(3,1,1)
plot(t,T1,t,T2,'g',t,T3,'r--')
legend('T_1','T_2','T_3')
xlabel('Time (s)')
ylabel('Motor Torque (Nm)')
title('Motor spec');
grid

subplot(3, 1, 2)
plot(t, i1, t, i2, 'g', t, i3, 'r--')
legend('I_1', 'I_2', 'I_3');
xlabel('Time (s)')
ylabel('Current (A)');
title('Current Response')
grid


subplot(3,1,3)
plot(t,w1,t,w2,'g',t,w3,'r--')
legend('\omega_1','\omega_2','\omega_3')
xlabel('Time (s)')
ylabel('Motor speed (rpm)')
grid

% Plot torque-vel plots

figure;
sgtitle("Saturated System Results")

subplot(2, 1, 1)
plot(T1, w1, 'r', T2, w2, 'g', T3, w3, 'c');
grid on;
yline(0);
xline(0);
legend("T1", "T2", "T3")
ylabel("Angular Velocity (RPM)")
xlabel ("Torque (Nm)")

subplot(2, 1, 2)
P1 = T1.*w1/9.5488;
P2 = T2.*w2/9.5488;
P3 = T3.*w3/9.5488;
plot(t, P1, 'r', t, P2, 'g', t, P3, 'c')
grid on;
legend("Power of Motor 1", "Power of Motor 2", "Power of Motor 3")
ylabel("Power (W)")
xlabel ("Time (s)")

t2 = t;
tx2 = tx;
px2 = px;
dtx2 = dtx;
dpx2 = dpx;
T12 = T1; T22 = T2; T32 = T3;
%%
figure
subplot(7,1,1)
plot(t1,px1,t2,px2,LineWidth=1.5)
ylabel('$\varphi_x$ (rad/s)', Interpreter='latex')
legend('Unlimited Torque','Limited Torque', Interpreter='latex')
grid on;
set(gca, 'FontSize', 14,'FontName','Palatino');

subplot(7,1,2)
plot(t1,tx1,t2,tx2,LineWidth=1.5)
ylabel('$\vartheta_x$ (rad/s)', Interpreter='latex')
% legend('Unlimited Torque','Limited Torque', Interpreter='latex')
grid on;
set(gca, 'FontSize', 14,'FontName','Palatino');

subplot(7,1,3)
plot(t1,dpx1,t2,dpx2,LineWidth=1.5)
ylabel('$\dot\varphi_x$ (rad/s)', Interpreter='latex')
% legend('Unlimited Torque','Limited Torque', Interpreter='latex')
grid on;
set(gca, 'FontSize', 14,'FontName','Palatino');

subplot(7,1,4)
plot(t1,dtx1,t2,dtx2,LineWidth=1.5)
ylabel('$\dot\vartheta_x$ (rad/s)', Interpreter='latex')
% legend('Unlimited Torque','Limited Torque', Interpreter='latex')
grid on;
set(gca, 'FontSize', 14,'FontName','Palatino');

subplot(7,1,5)
plot(t1,T11,t2,T12,LineWidth=1.5)
ylabel('$T_1$ (Nm)', Interpreter='latex')
% legend('Unlimited Torque','Limited Torque', Interpreter='latex')
grid on;
set(gca, 'FontSize', 14,'FontName','Palatino');

subplot(7,1,6)
plot(t1,T21,t2,T22,LineWidth=1.5)
ylabel('$T_2$ (Nm)', Interpreter='latex')
% legend('Unlimited Torque','Limited Torque', Interpreter='latex')
grid on;
set(gca, 'FontSize', 14,'FontName','Palatino');

subplot(7,1,7)
plot(t1,T31,t2,T32,LineWidth=1.5)
ylabel('$T_3$ (Nm)', Interpreter='latex')
% legend('Unlimited Torque','Limited Torque', Interpreter='latex')
grid on;
set(gca, 'FontSize', 14,'FontName','Palatino');
