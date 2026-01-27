%% EXPLINATION
clc; clear; close all;
% Initial goal is to build a planar instantiation of the model as outlined
% by Fankhauser et. al in "Modeling and Control of a Ballbot" Build yz
% planar model.

[A, B, C, D, ~] = make_ballbot(); % determine matricies

% Make state-space system
sys = ss(A,B,C,D);

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

% --- Top plot: angles ---
subplot(2,1,1)
plot(t, y(:,1), 'LineWidth', 1.5); hold on
plot(t, y(:,2), 'LineWidth', 1.5)
yline(0,'k','LineWidth',1)
ylim([-1, 1])
ylabel('Angle (rad)')
title( ...
    sprintf(['Zero-Input Response, Initial State: ' ...
             '$x_0 = [%.3f,\\ %.3f,\\ %.3f,\\ %.3f]^T$'], ...
             x0(1), x0(2), x0(3), x0(4)), ...
    'Interpreter','latex')
legend({'$\phi$','$\theta$'}, 'Interpreter','latex')
grid on

% --- Bottom plot: angular rates ---
subplot(2,1,2)
plot(t, y(:,3), '--', 'LineWidth', 1.5); hold on
plot(t, y(:,4), '--', 'LineWidth', 1.5)
yline(0,'k','LineWidth',1)
ylabel('Angular rate (rad/s)')
xlabel('Time (s)')
legend({'$\dot{\phi}$','$\dot{\theta}$'}, 'Interpreter','latex')
grid on


