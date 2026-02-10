function p = params()
% BALLBOT_PARAMS  Parameters for planar ballbot model (1D)
%USE SI UNITS - starting 
% Constants ----------------------------------------------------
%Masses
p.m_k = 2.29;   %kg, Mass of the Ball
p.m_w = 1000;      %kg, Mass of the virtual actuating wheel
p.m_A = 9.2;    %kg, Mass of the body
p.m_omni = 0.5; %kg, reverse engineered from inertia given


%Radius
p.r_k = .125;   %m, Radius of the ball
p.r_w = .06;    %m, radius of the OMNIWHEELS, NOT VIRTUAL
p.r_A = .1;     %m, radius of the body (Cylinder)

% Angle of wheels on ball
p.alpha = 45; % deg
p.beta = 120; %deg 

% Body distance
p.l = .339;     %m, Height of the center of gravity

% Gravity
p.g = 9.81;               % m/s^2, Gravitational acceleration

% Gear train
p.i_Gear = 26;            % -, Gear ratio

% Inertias
p.Theta_m = 3.33 * 10^-6; % rotor intertia (real small)
p.Theta_A    = 4.76;      % kg*m^2, Inertia of the body
p.Theta_A_xy = 0.092;     % kg*m^2, Inertia of the body in xy-plane

% Calculations ------------------------------------------------------
% Dims

% Inertias
p.Theta_k = 2/3 * p.m_k * p.r_k^2; %kgm^2 Inertia of the ball
p.Theta_omni = 1/2 * p.m_omni * p.r_w^2; % for the actual omniwheels

% Virtual wheels
p.Theta_w = 3/2 *cos(deg2rad(p.alpha))^2*(p.Theta_omni + (p.i_Gear^2)*p.Theta_m); % YZ and XZ
p.Theta_w_xy = 3*(p.Theta_omni + (p.i_Gear^2)*p.Theta_m);   % kg*m^2, Inertia of actuating wheel in xy-plane

%p.virt_mass = 2*(2*p.Theta_w + p.Theta_w_xy)/(p.r_w^2);
p.virt = (2 * p.Theta_w)/((1/3*p.r_w)^2);
end