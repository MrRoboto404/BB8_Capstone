function p = params()
% BALLBOT_PARAMS  Parameters for planar ballbot model (1D)
%USE SI UNITS - starting 

% VARIABLE properties %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%_______Masses_______
p.m_k = 2.29;   %kg, Mass of the Ball
p.m_A = 9.2;    %kg, Mass of the body
p.m_omni = 4; %kg, reverse engineered from inertia given

%_______Lengths_______
p.r_k = .125;   %m, Radius of the ball
p.r_w = .06;    %m, radius of the OMNIWHEELS, NOT VIRTUAL
p.r_A = 0.1;     %m, radius of the body (cylinder)

p.h_A = 0.894;  %m, height of the body (cylinder). Reverse engineered from interia calcs
p.l = 0.339;     %m, Height of the center of gravity of body from center of ball

%_______Angle of wheels on ball_______
p.alpha = deg2rad(45); %rad
p.beta = deg2rad(0); %rad, taken from +X axis (adds intervals for other locations)

%_______Misc_______
p.g = 9.81;     % m/s^2, Gravitational acceleration
p.i_Gear = 260;  % -, Gear ratio

%_______Inertia Constants_______
p.Theta_m = 3.33 * 10^-6; % rotor intertia (real small)

% CALCULATIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% _________Inertias___________
p.Theta_k = 2/3 * p.m_k * p.r_k^2; %kgm^2 Inertia of the ball
p.Theta_omni = 1/2 * p.m_omni * p.r_w^2; % for the actual omniwheels
%           Vertical Planes
p.Theta_A = 0.25*p.m_A*(p.r_A^2) + 0.5*p.m_A*(p.h_A^2) + p.m_A*p.l^2;      % kg*m^2, Inertia of the body in vertical planes
p.Theta_w = 3/2 *cos(p.alpha)^2*(p.Theta_omni + (p.i_Gear^2)*p.Theta_m); % YZ and XZ

% _______Masses____________
p.m_w = 3/2 * cos(p.alpha)^2 * p.m_omni;

%           XY Plane
p.Theta_w_xy = 3*(p.Theta_omni + (p.i_Gear^2)*p.Theta_m);   % kg*m^2, Inertia of actuating wheel in XY plane
p.Theta_A_xy = 0.5 * (p.m_A + p.m_w) * p.r_A^2; % kg*m^2, interia of body in XY plane


end