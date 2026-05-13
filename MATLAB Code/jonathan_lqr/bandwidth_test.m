clear; close all; clc;
[A,B,C,D] = get_linearized_matrices_vertical(params());
K = get_gains_LQR_vertical(0);
A_cl = A - B*K;
[wn, z, poles] = damp(A-B*K);
wn_max = max(wn);
f_max = wn_max / (2*pi);
f_controller = 30 * f_max

%% Open loop
% Define the open-loop system L(s) cut at the plant input
D_open = zeros(size(K,1), size(B,2)); 
sys_open = ss(A, B, K, D_open);

figure;
margin(sys_open);
grid on;
title('Open-Loop Bode Plot');
%% Closed loop
sys_cl = ss(A_cl, B, C, D);
state_names = {'Phi (Ball Angle)', 'Theta (Body Tilt)', 'Phi Dot (Ball Velocity)', 'Theta Dot (Tilt Velocity)'};

for i = 1:4
    figure;
    sys_single_state = sys_cl(i, 1);  % sys(i,1) to inspect the i_th state
    bode(sys_single_state);
    grid on;
    title(sprintf('Closed-Loop Bode Plot: %s', state_names{i}));
end
%% Open loop (no controller)
% Define the open-loop system L(s) cut at the plant input
D_open = zeros(size(K,1), size(B,2)); 
sys_open_2 = ss(A, B, C, D);

figure;
bode(sys_open_2);
grid on;
title('Open-Loop Bode Plot');
