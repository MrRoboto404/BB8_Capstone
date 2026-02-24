function [A, B, C, D] = get_statespace_xy(p)
    % With stiction assumtion, phi_z and phidot_z is always zero
    % the xy planes only have two states theta and dottheta
    %
    % Inputs:
    %   p - Struct containing physical parameters (from params.m)
    %
    % Outputs:
    %   A, B, C, D - State Space matrices for the system:
    %                xdot  = Ax + Bu
    %                y     = Cx + Du    
    %
    % State Vector: x = [theta; theta_dot]
    % Input:        u = T (Virtual Torque)
    % Output:       y = x = [theta; theta_dot] (states themself)


    m_tot = p.m_ball + p.m_body + p.m_virt;
    r_tot = p.r_ball + p.r_omni;
    gamma = p.l*p.m_body + (p.r_ball + p.r_omni)*p.m_virt;

    %% 
    % --- Matrix A (2x2) ---
    A = zeros(2,2);
    A(1,2) = 1;
  
    % --- Matrix B (2x1) ---
    B = zeros(2,1);
    B(2) = -(p.r_ball*p.r_omni*sin(p.alpha))/((p.r_omni)^2*p.Theta_body_xy+(p.r_ball)^2*p.Theta_virt_xy*(sin(p.alpha))^2);
    
    % Matrix C (2x2) 
    C = eye(2);
    % Matrix D (2x1)
    D = zeros(2,1);
    
end