%% EXPLINATION
clc; clear; close all;
% Initial goal is to build a planar instantiation of the model as outlined
% by Fankhauser et. al in "Modeling and Control of a Ballbot" Build yz
% planar model.

[A, B, C, D, ~] = make_ballbot(); % determine matricies
bb = params();
% Make state-space system
sys = ss(A,B,C,D);

% Pole placement design for Motor sizing

%Goal - figure out what necessary motor torques we need based on the
%targets for our system

%----------key inputs------------
Ts = 1.15; %Settling time, seconds
POS = 5; %Percent overshoot,

%Initial State  
x0 = [ .0; deg2rad(10); 0; 0];
%--------------------------------

zeta = -log(POS/100) / (sqrt(pi^2+ (log(POS/100))^2 )); %Damping ratio of target closed loop system poles
wn = 4/(Ts*zeta); %natural frequency

sigma = zeta*wn;
wd    = wn*sqrt(1 - zeta^2);
p_dom = [-sigma + 1j*wd, -sigma - 1j*wd]; %DOMINANT CLOSED LOOP POLES

p_minor = [-2*sigma, -3*sigma];

poles = [p_dom, p_minor]; % Combine dominant and minor poles for placement


K = place(A, B, poles); % State feedback gain

% quick sanity check
Acl = A - B*K;
sys_cl = ss(Acl, B, C, D);
t  = 0:0.001:3;
[y,t,x_out] = initial(sys_cl, x0, t);
figure

% ----plot time response ---
plot_body_angles(t,y,x0, "Closed loop system Response");
hold on;
plot_body_angular_rates(t,y);


%----Calculate Virtual motor Control efforts ----
u = -(x_out * K.')/bb.i_Gear;
u_peak = max(abs(u), [], 1);
u_rms  = rms(u, 1);


%-----Calculate Real motor Control efforts -----
Tx = u;
Tz = 0;
Ty = 0;
[T1, T2, T3] = real_motor_torques_from_virtual(Tx,Ty,Tz,bb);





%Plot pole zero map for closed loop system.
p = pole(sys_cl);
z = zero(sys_cl);

figure
plot(real(p), imag(p), 'rx', 'MarkerSize',12, 'LineWidth',2)
hold on
plot(real(z), imag(z), 'bo', 'MarkerSize',12, 'LineWidth',2)

grid on
axis equal
sgrid
ylim([-5,5]);
xlabel('Real Axis')
ylabel('Imag Axis')
title('Ballbot Closed Loop Poles')


%---- Plot Virtual Motor Torques 
figure; 
plot(t,u,'LineWidth',1.5); grid on;
xlabel('Time (s)'); ylabel('u(t), Nm'); title('Virtual (MOTOR) Nm required to recover from initial conditions');

%--Plot real motor torques
figure; 
plot(t, T1, 'LineWidth', 1.5); 
hold on
plot(t, T2, 'LineWidth', 1.5);
plot(t, T3, 'LineWidth', 1.5); 
grid on
hold off; 

legend("T1", "T2", "T3");
xlabel('Time (s)');
ylabel('u(t), Nm');
title('Real Motor Torque required in recovery from initial conditions');

%% Plant Validation - zero input response
% Display results
disp('A ='); disp(A);
disp('B ='); disp(B);
disp('C ='); disp(C);
disp('D ='); disp(D);

%Validation of model
pzmap(sys); %Plots poles and zeros - should see a rhp pole.
x0 = [ .0,.3,0,-1];        % initial state
t  = 0:0.001:1;      % time vector

[y,t,x] = initial(sys, x0, t);

figure
plot_body_angles(t,y);
plot_body_angular_rates(t,y);

%% Helper functions 
% --- Top plot: angles ---
function plot_body_angles(t,y,x0, titleStr)
    subplot(2,1,1)
    plot(t, y(:,1), 'LineWidth', 1.5); hold on
    plot(t, y(:,2), 'LineWidth', 1.5)
    yline(0,'k','LineWidth',1);
    ylabel('Angle (rad)')
    title( ...
    sprintf(['%s, Initial State: ' ...
             '$x_0 = [%.3f,\\ %.3f,\\ %.3f,\\ %.3f]^T$'], ...
             titleStr, x0(1), x0(2), x0(3), x0(4)), ...
    'Interpreter','latex')
    legend({'$\phi$','$\theta$'}, 'Interpreter','latex')
    grid on
end

% --- Bottom plot: angular rates ---
function plot_body_angular_rates(t,y)
    subplot(2,1,2)
    plot(t, y(:,3), '--', 'LineWidth', 1.5); hold on
    plot(t, y(:,4), '--', 'LineWidth', 1.5)
    yline(0,'k','LineWidth',1)
    ylabel('Angular rate (rad/s)')
    xlabel('Time (s)')
    legend({'$\dot{\phi}$','$\dot{\theta}$'}, 'Interpreter','latex')
    grid on
end

function [T1, T2, T3] = real_motor_torques_from_virtual(Tx, Ty, Tz, bb)
alpha = bb.alpha;
beta  = bb.beta;

cosa = cos(alpha);
cb = cos(beta);
sb = sin(beta);

T1 = (1/3) * ( ...
      Tz + (2/cosa) * (Tx*cb - Ty*sb) );

T2 = (1/3) * ( ...
      Tz + (1/cosa) * ( sb*(-sqrt(3)*Tx + Ty) ...
                      - cb*( Tx + sqrt(3)*Ty ) ) );

T3 = (1/3) * ( ...
      Tz + (1/cosa) * ( sb*( sqrt(3)*Tx + Ty) ...
                      + cb*(-Tx + sqrt(3)*Ty ) ) );

end
