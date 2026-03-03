%{
GOAL: take calculated states from controller, and view planar responses
1. first make a rectangle spin bout the center circle in accordance to a vector
2. make the ball move
%}

%________________________PARAM SETUP________________________

clc; close all; %clear;
% Load visual-related parameters for ease-of-use
r_ball = params().r_ball;
h_body = 0.1*params().h_body;
w_body = 0.25*h_body; % 25% of body height, m. made up number

% Load time-based resposnes of control system (XZ plane)
t; % time (seconds)
theta_y = ty; % theta_y (rad)

%________________________DRAWING SETUP________________________
figure;
axis equal;
hold on;

axis_scale = 3;
xlim([-1*axis_scale*r_ball, axis_scale*r_ball]);
ylim([-0.4, 0.4]);

% Define the ball
draw_ball_angle = linspace(0, 2*pi, 100); % Angle for the circle
ball_x = r_ball * cos(draw_ball_angle); % X coordinates of the circle
ball_y = r_ball * sin(draw_ball_angle) + r_ball; % Y coordinates of the circle
plot(ball_x, ball_y, 'm'); % Plot the circle

% Create body points: x, y = [TL, TR, BR, BL] vertices
body_x = [-0.5*w_body, 0.5*w_body, 0.5*w_body, -0.5*w_body];
body_y = 2*r_ball + [h_body, h_body, 0, 0]; % ensure offset

%________________________ANIMATION LOOP________________________
nFrames = length(t); % Number of frames for the movie
sim_sec = 10; % duration of control simulation
FPS = floor(nFrames/sim_sec); % typically like 101.1 fps = 101

frames(nFrames) = struct('cdata', [], 'colormap', []); % Structure to store frames

for i = 1:nFrames % animate every timestep of the system
    clf; % Clear figure for new frame
    hold on;
    axis equal;
    xlim([-1*axis_scale*r_ball, axis_scale*r_ball]);
    ylim([-0.4, 0.4]);


    % Plot the ball again
    plot(ball_x, ball_y, 'm');
    
    % Rotate the body in accordance to this timestep's angle 
    %angle = 2 * pi * k / nFrames; % Calculate current rotation angle
    angle = theta_y(i);
    rotation_matrix = [cos(angle), -sin(angle); sin(angle), cos(angle)]; % Rotation matrix
    
    % Apply the rotation to the triangle points
    rotated_points = rotation_matrix * [body_x; body_y];
    
    % Plot the rotated triangle
    fill(rotated_points(1,:), rotated_points(2,:), 'r'); % Plot the triangle in red
    
    % Store the frame
    frames(i) = getframe(gcf); % Capture the current frame
end

pause;
% Create the movie
movie(frames, 1, FPS); % Play the movie a single time