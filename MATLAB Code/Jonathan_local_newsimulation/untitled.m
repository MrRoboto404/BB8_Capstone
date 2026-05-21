[A,B,C,D] = get_linearized_matrices_vertical(params())

Q = diag([.04 3 0.1 0.000001]);               % State weighting matrix
R = 0.25;                              % Control weighting matrix
[K, S, E] = lqr(A,B, Q, R)   % Compute the state feedback gain using lqr()

K(4) = -15.1371;
A_cl = A - B * K;
B_cl = B; 
C_cl = C;
D_cl = D;

% Create the closed-loop state-space system object
sys_cl = ss(A_cl, B_cl, C_cl, D_cl);

pzmap(sys_cl)

p = pole(sys_cl)
z = tzero(sys_cl)
