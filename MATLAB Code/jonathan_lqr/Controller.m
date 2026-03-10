clearvars; clc;
close all;

% This script is the second attempt to implement LQR control for all 3 planes %%

K_vert = get_gains_LQR_vertical(1);
K_xy = get_gains_LQR_xy;
% Initial conditions
%  [\phi, \theta, \dot{\phi}, \dot{\theta}]

% Simulation
% Initial condition
T_max = 10;      % Maximum Motor Torque from "Napkin Math"
ic_xz = [0; deg2rad(10); 0; 0];  % initial condition
ic_yz = [0; deg2rad(10); 0; 0];
ic_xy = [0; 0];
T_sim = 10; % s

% Nonlinear
for (i = [1,2])
    simulink_filename = ["LQR_nonlinear_sim","LQR_2"];
    graph_title = ["Nonlinear Unlimited Torque","Nonlinear Limited Torque"];
    simout = sim(   simulink_filename(i), ...
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
    
    disp("___Current Results: " + graph_title(i) + "___")
    fprintf("Total Charge Used: %f (A*h)\n", qtotal/3600)

    
    %________________________PLOTS________________________
    % figure('Name', 'Original', 'NumberTitle', 'off')
    figure
    sgtitle(graph_title(i));
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
    sgtitle(graph_title(i));
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
end

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

% Linearized Model
t = linspace(0,T_sim,500);
[A,B,C,D] = get_linearized_matrices_vertical(params());
sys_linearized_vert = ss(A - B*K_vert,B,C,D);
qx = initial(sys_linearized_vert, ic_yz, t);
qy = initial(sys_linearized_vert, ic_xz, t);
[A,B,C,D] = get_statespace_xy(params());
sys_linearized_xy = ss(A - B*K_xy,B,C,D);
qz = initial(sys_linearized_xy, ic_xy, t);


figure
sgtitle('Linear model sanity check');
subplot(3,3,1)
plot(t,qx(:,2))
legend('theta_x')
ylabel('Angle (rad)')
title('yz plane')
grid

subplot(3,3,2)
plot(t,qy(:,2))
legend('theta_y')
ylabel('Angle (rad)')
title('xz plane')
grid

subplot(3,3,3)
plot(t,qz(:,1))
legend('theta_z')
ylabel('Angle (rad)')
title('xy plane')
grid

subplot(3,3,4)
plot(t,qx(:,1))
legend('phi_x',Location='southeast')
ylabel('Angle (rad)')
grid

subplot(3,3,5)
plot(t,qy(:,1))
legend('phi_y',Location='southeast')
ylabel('Angle (rad)')
grid

subplot(3,3,7)
plot(t,qx(:,3),t,qx(:,4))
ylabel('Angle rate (rad/s)')
legend('dotphi x','dottheta x')
grid

subplot(3,3,8)
plot(t,qy(:,3),t,qy(:,4))
ylabel('Angle rate (rad/s)')
legend('dotphi y','dottheta y')
grid

subplot(3,3,6)
plot(t,qz(:,2))
ylabel('Angle rate (rad/s)')
legend('dottheta z')
grid



