%% LQR Controller Vertical Planes
% control_mode: 0 - Full State Feedback
%               1 - Velocity Control
function [K, N] = get_gains_LQR_vertical(control_mode)

[A,B,C,D] = get_linearized_matrices_vertical(params());
Q = diag([2 150 1 5]);               % State weighting matrix
R = 1;                              % Control weighting matrix
[K, S, E] = lqr(A,B, Q, R);    % Compute the state feedback gain using lqr()

%% Velocity control (make robot move at constant speed)
% Only feedback [theta; dot(phi), dot(theta)]
if (control_mode == 1)
    A_vel = A(2:4,2:4);             % Remove the row and collumn contain phi
    B_vel = B(2:4);
    C_vel = [0 1 0];
    K_vel = K(2:4);
    N = -1 / (C_vel * inv(A_vel - B_vel*K_vel) * B_vel);   % DC gain for if the ref is the output
    K(1) = 0;                              % Disable phi feedback
end


