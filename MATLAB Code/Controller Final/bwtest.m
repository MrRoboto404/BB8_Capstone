clear;clc;
% close all;

[A,B,C,D] = get_linearized_matrices_vertical(params());

[K, N] = get_gains_LQR_vertical(1); % Mode 1 for Velocity Tracking

% Reconstruct the closed-loop system
% The input is  u = -Kx + N*r
sys = ss(A,B,C,D);
sys_cl = ss(A - B*K, B * N, C, D);

% Now test bandwidth for the velocity output (Output 2 in your Q matrix?)
% bw_vel = bandwidth(sys_cl(2))

figure;
for i = 1:4
    subplot(2,2,i);
    bodemag(sys_cl(i,1)); 
    grid on;
    title(['Magnitude Response: Output ', num2str(i)]);
end

% 1. Check your Closed-Loop Poles
poles = eig(A - B*K);
disp('Closed-Loop Poles:');
disp(poles);

% 2. Look at the Impulse Response (The "Push" test)
% This simulates hitting the robot and seeing how it stabilizes.
sys_cl = ss(A - B*K, B, C, D);
figure;
impulse(sys_cl); 
title('Impulse Response: Reaction to a Disturbance');