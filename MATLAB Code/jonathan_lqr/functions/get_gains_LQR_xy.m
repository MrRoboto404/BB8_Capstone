%% LQR Controller xy Plane
% Can be tuned further but leave as indentity for now
function [K, S, E] = get_gains_LQR_xy()
[A,B,C,D] = get_statespace_xy(params());
Q = diag([1 1]);                    % State weighting matrix
R = 1;                              % Control weighting matrix
[K, S, E] = lqr(A,B, Q, R);    % Compute the state feedback gain using LQR