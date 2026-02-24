function [A, B, C, D] = statespace_xy(p)
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
    % Input:        u = T (Motor Torque)
    % Output:       y = x = [phi; theta; phi_dot; theta_dot] (states themself)


    m_tot = p.m_ball + p.m_body + p.m_virt;
    r_tot = p.r_ball + p.r_omni;
    gamma = p.l*p.m_body + (p.r_ball + p.r_omni)*p.m_virt;

    %% 
    % --- Matrix A (4x4) ---
    A = zeros(2,2);
    A(1,2) = 1;
  
    % --- Matrix B (4x1) ---
    B = zeros(2,1);
    B(2) = -(p.r_ball*p.r_omni*sin(p.alpha))/((p.r_omni)^2*p.Theta_body_xy+(p.r_ball)^2*p.Theta_virt_xy*(sin(p.alpha))^2);
    
    % Matrix C (4x4) 
    C = eye(2);
    % Matrix D (4x1)
    D = zeros(2,1);
    
end