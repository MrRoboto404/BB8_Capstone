function p = params()
% BALLBOT_PARAMS  Parameters for planar ballbot model (1D)
%USE SI UNITS - starting 

%Masses
p.m_k = 2.29;   %kg, Mass of the Ball
p.m_w = 3;      %kg, Mass of the virtual actuating wheel
p.m_A = 9.2;    %kg, Mass of the body

%Radius
p.r_k = .125;   %m, Radius of the ball
p.r_w = .06;    %m, radius of the omniwheels
p.r_A = .1;     %m, radius of the body (Cylinder)


p.l = .339;     %m, Height of the center of gravity

% Inertias
p.Theta_k = .0239;        %kgm^2 Inertia of the ball
p.Theta_w    = 0.00236;   % kg*m^2, Inertia of actuating wheel in yz-/xz-plane
p.Theta_w_xy = 0.00945;   % kg*m^2, Inertia of actuating wheel in xy-plane

p.Theta_A    = 4.76;      % kg*m^2, Inertia of the body
p.Theta_A_xy = 0.092;     % kg*m^2, Inertia of the body in xy-plane

% Gravity
p.g = 9.81;               % m/s^2, Gravitational acceleration

% Gear train
p.i_Gear = 26;            % -, Gear ratio

%Motor inclination angles
p.alpha = deg2rad(45);
p.beta = deg2rad(0);

end