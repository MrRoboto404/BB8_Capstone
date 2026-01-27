function [A, B, C, D, bb] = make_ballbot()
% build with initial parameters
bb = params();

% Equilibrium (upright) - this is where we will linearize around.
x_eq = [0; 0; 0; 0];   % [phi; theta; phi_dot; theta_dot]
u_eq = 0;              % motor torque input

% Nonlinear dynamics handle: f(x,u) -> xdot
f = @(x,u) ballbot_planar_f(0, x, u, bb);

% Linearize: xdot ≈ A(x-x_eq) + B(u-u_eq)
[A,B] = linearize_fd(f, x_eq, u_eq);

% Output = full state, y = Cx + Du. y= x
C = eye(4);
D = zeros(4,1);

end

% Below function programs by Martin 1/26

function xdot = ballbot_planar_f(~, x, u, p)
    %This function maps how the ballbot evolves over time. Building out
    % x_dot =f(t,x,u).
    %Ball bot system plant doesn't depend on time, but solvers will often
    %call using t as an input, so ~ included as first argument
    % x = [phi; theta; phi_dot; theta_dot]
    q1  = x(1);
    q2  = x(2);
    q1d = x(3);
    q2d = x(4);
    
    %build state to send to helper function
    q = [q1; q2];
    qdot = [q1d; q2d];
    
    [M, Cvec, Gvec, B] = ballbot_planar_MCG(q, qdot, p); %%use helper function to compute instantaneous values of the matricies.
    
    F_np = B*u;                      % 2x1
    %use manipulator equation to return qdoubledot (eq. 2.21)
    qdd = M \ (F_np - Cvec - Gvec);  % 2x1
    
    
    xdot = [ q1d;
             q2d;
             qdd(1);
             qdd(2) ];
end

function [M, Cvec, Gvec, B] = ballbot_planar_MCG(q, qdot, p)
    
    %This function allows the computation of the general "manipulator" form
    %matricies at any time given the state vector as input as well as the
    %ballbot parameters. It calculates the M C G and F_np matricies.
    
    phi = q(1);   % not used, (M,C,G depend on theta only)
    theta = q(2);
    phi_d = qdot(1);
    theta_d = qdot(2);
    
    m_tot = p.m_k + p.m_A + p.m_w;
    r_tot = p.r_k + p.r_w;
    gamma = p.l*p.m_A + (p.r_k + p.r_w)*p.m_w;
    
    % M matrix (Eq 2.22)
    M11 = m_tot*p.r_k^2 + p.Theta_k + (p.r_k/p.r_w)^2 * p.Theta_w;
    M12 = -(p.r_k/p.r_w^2)*r_tot*p.Theta_w + gamma*p.r_k*cos(theta);
    M21 = M12;
    M22 = (r_tot^2/p.r_w^2)*p.Theta_w + p.Theta_A + p.m_A*p.l^2 + p.m_w*r_tot^2;
    
    M = [M11 M12;
         M21 M22];
    
    % C vector (Eq 2.23)
    Cvec = [ -p.r_k*gamma*sin(theta)*theta_d^2;
              0 ];
    
    % G vector (Eq 2.24)
    Gvec = [ 0;
            -p.g*sin(theta)*gamma ];
    
    % Non-potential forces (Eq 2.17 + 2.18 combined):
    %This eventually gets multiplied by the input to make F_np
    a = p.r_k / p.r_w;
    B = [ a;
         -a ];
end

function [A,B] = linearize_fd(f, x0, u0)
% Finite-difference linearization of xdot = f(x,u)
% A = dfdx |(x0,u0), B = dfdu |(x0,u0)
    
    epsx = 1e-6;
    epsu = 1e-6;
    
    n = numel(x0);
    m = numel(u0);
    
    f0 = f(x0,u0);
    
    A = zeros(n,n);
    for i = 1:n
        dx = zeros(n,1);
        dx(i) = epsx;
        A(:,i) = (f(x0 + dx, u0) - f0) / epsx;
    end
    
    B = zeros(n,m);
    for j = 1:m
        du = zeros(m,1);
        du(j) = epsu;
        B(:,j) = (f(x0, u0 + du) - f0) / epsu;
    end
end