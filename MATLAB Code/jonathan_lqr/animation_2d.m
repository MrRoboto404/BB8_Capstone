%% Ball-Bot 2D Planar Animation (yz plane
clear; clc; close all;
%% 0. Simulink for data
K_vert = get_gains_LQR_vertical(1);
K_xy = get_gains_LQR_xy;
% Initial conditions

% Simulation
% Initial condition

fps = 30;

T_max = 5;      % Maximum Motor Torque
ic_xz = [0; deg2rad(0); 0; 0];  % initial condition
ic_yz = [0; deg2rad(10); 0; 0];
ic_xy = [0; 0];
T_sim = 3; % s

simout = sim(   "LQR_2", ...
                'StopTime', num2str(T_sim), ...
                'Solver','ode45', ...
                'RelTol','auto', ...
                'AbsTol','auto', ...
                'MaxStep',num2str(1/fps));
%% 1. Data
phi   = squeeze(simout.logsout.get('phi_x').Values.Data);         
theta = squeeze(simout.logsout.get('theta_x').Values.Data); 
psi   = squeeze(simout.logsout.get('psi_x').Values.Data); 
[num_frames,~] = size(phi);

p = params();
% Rb = p.r_ball;      %m
% Rw = p.r_omni;      %m
L  = p.l;           %m

Rb = 0.108;      
Rw = 0.048;      


%% 2.
% Pre-calculate circle shapes for drawing
circ_th = linspace(0, 2*pi, 50);
circ_x = cos(circ_th);
circ_y = sin(circ_th);

% Setup Figure
figure('Name', 'Ballbot Simulation','Position', [100, 100, 800, 800]);
F(num_frames) = struct('cdata',[],'colormap',[]);

% 3. ANIMATION LOOP
for i = 1:num_frames
    
    % Get current angles for this frame
    curr_phi   = phi(i);
    curr_theta = theta(i);
    curr_psi   = psi(i);
    % Kinematics (B. Planar system modelling, pg. 58 on ETHZurich paper)
    % 1. Rolling ball
    % No-slip condition: horizontal position = radius * rolling angle
    xb = Rb * curr_phi; 
    yb = Rb; % Ball center height is constant
    
    % 2. Build robot on top of ball
    % Virtual Wheel Center (stacked on ball, tilted by theta)
    xw = xb + (Rb + Rw) * sin(curr_theta);
    yw = yb + (Rb + Rw) * cos(curr_theta);
    
    % CoM / Body (stacked on ball, tilted by theta)
    x_com = xb + L * sin(curr_theta);
    y_com = yb + L * cos(curr_theta);
    
    % 3. ROTATION DOTS
    % Ball rolls clockwise as it moves right
    dot_b_x = xb + Rb * cos(pi/2-ic_yz(2) - curr_phi);
    dot_b_y = yb + Rb * sin(pi/2-ic_yz(2) - curr_phi);
    
    % Virtual wheel dot (Assuming psi is absolute lab-frame angle for now)
    dot_w_x = xw + Rw * cos(-(pi/2+ic_yz(2)) + curr_psi);
    dot_w_y = yw + Rw * sin(-(pi/2+ic_yz(2)) + curr_psi);

    % --- DRAWING ---
    cla; hold on;
    
    % Draw a long, fixed ground line
    plot([-1, 1], [0, 0], 'k-', 'LineWidth', 2); 
    
    % Draw Body, Ball, Wheel
    plot([xb, x_com], [yb, y_com], 'Color', [1, 0.5, 0], 'LineWidth', 3);
    plot(xb + Rb*circ_x, yb + Rb*circ_y, 'r-', 'LineWidth', 3);
    plot(xw + Rw*circ_x, yw + Rw*circ_y, 'g-', 'LineWidth', 3);
    
    % Draw Dots & CoM
    plot(dot_b_x, dot_b_y, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 6);
    plot(dot_w_x, dot_w_y, 'mo', 'MarkerFaceColor', 'm', 'MarkerSize', 6);
    plot(x_com, y_com, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 10);
    plot(x_com, y_com, 'w+', 'LineWidth', 3, 'MarkerSize', 8);
    
    % Lab frame camera
    axis equal;
    % Define a static viewing window: [xmin, xmax, ymin, ymax]
    axis([-0.5, 0.7, 0, 0.5]); 
    axis off;
    
    % drawnow;
    F(i) = getframe(gcf);
end

% 4. playback
movie(gcf, F, 1, fps);

%% --- SAVE THE ANIMATION AS AN MP4 ---

% 1. Name your file and specify the format ('MPEG-4' makes an .mp4)
video_filename = 'Ballbot_Simulation.mp4';
myVideo = VideoWriter(video_filename, 'MPEG-4'); 

% 2. Set the playback speed (frames per second)
% If your video looks too fast or slow, adjust this number
myVideo.FrameRate = 30; 

% 3. Open the file, write the frames, and close it
open(myVideo);          
writeVideo(myVideo, F); % F is the array where we stored getframe
close(myVideo);         

% Print a success message so you know it's done
disp(['Saved ', video_filename]);