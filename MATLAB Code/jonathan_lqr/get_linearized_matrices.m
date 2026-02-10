function [A, B, C, D] = get_linearized_matrices(p)
    % GET_LINEARIZED_MATRICES Linearizes the Ballbot dynamics about the
    % ICs: (theta = 0, theta_dot = 0) (upright equilibrium)
    %
    % Inputs:
    %   p - Struct containing physical parameters (from params.m)
    %
    % Outputs:
    %   A, B, C, D - State Space matrices for the system:
    %                xdot  = Ax + Bu
    %                y     = Cx + Du    
    %
    % State Vector: x = [phi; theta; phi_dot; theta_dot]
    % Input:        u = M (Motor Torque)
    % Output:       y = x = [phi; theta; phi_dot; theta_dot] (states themself)


    m_tot = p.m_k + p.m_A + p.m_w;
    r_tot = p.r_k + p.r_w;
    gamma = p.l*p.m_A + (p.r_k + p.r_w)*p.m_w;

    %% M Matrix
    M11 = m_tot * p.r_k^2 + p.Theta_k + (p.r_k / p.r_w)^2 * p.Theta_w;
    M12 = -(p.r_k / p.r_w^2) * r_tot * p.Theta_w + gamma * p.r_k; 
    M22 = (r_tot^2 / p.r_w^2) * p.Theta_w + p.Theta_A + ...
           p.m_A * p.l^2 + p.m_w * r_tot^2;
    M21 = M12;
    M = [M11, M12;
         M21, M22];
    %% G Matrix
    % 
    G_matrix = [0, 0; 
         0, -p.g * gamma]; 
    %% f_NP = S*u(t)
    S = [ (p.r_k / p.r_w); 
         -(p.r_k / p.r_w)];

    %% Solve for State Space Matrices
    % The linearized equation of motion is: M * q_ddot + G_grad * q = S * u
    % Rearranging for acceleration: q_ddot = -M_inv * G_grad * q + M_inv * S * u
    M_inv = inv(M);
    
    % --- Matrix A (4x4) ---
    % Structure: [        0            I ]
    %            [ [-M_inv*G_matrix]   0 ]
    A = zeros(4,4);
    % qdot defines itself
    A(1,3) = 1; 
    A(2,4) = 1;
    % Doubledot terms
    A(3:4, 1:2) = -M_inv * G_matrix; 
    
    % --- Matrix B (4x1) ---
    % Structure: [     0     ]
    %            [ M_inv * S ]
    B = zeros(4,1);
    B(3:4) = M_inv * S;
    
    % Matrix C (4x4) 
    C = eye(4);
    % Matrix D (4x1)
    D = zeros(4,1);
    
end