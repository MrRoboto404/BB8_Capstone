function ballbot_movie(phi, psi, theta_x, r_ball, r_virt, l)

t_total = length(phi);

% --- Safety check ---
if length(psi) ~= t_total || length(theta_x) ~= t_total
    error('phi, psi, and theta_x must be the same length.');
end

% --- Figure setup ---
figure;
axis equal
hold on
grid on

xlim([-2 2])
ylim([-0.1 2])

% Ground line
plot([-100 100], [0 0], 'k', 'LineWidth', 2)

% Circle parameterization
ang = linspace(0,2*pi,200);

% Horizontal translation state
x_ball = 0;

for k = 1:t_total
    
    cla
    hold on
    plot([-100 100], [0 0], 'k', 'LineWidth', 2)
    
    % --- Horizontal translation (pure rolling assumption optional) ---
    % Here: translate based on ball rotation
    x_ball = r_ball * phi(k);  
    
    % Ball center
    ball_center = [x_ball; r_ball];
    
    % --- Ball circle ---
    x_ball_circ = ball_center(1) + r_ball*cos(ang);
    y_ball_circ = ball_center(2) + r_ball*sin(ang);
    plot(x_ball_circ, y_ball_circ, 'b', 'LineWidth', 2)
    
    % --- Ball rotation marker ---
    ball_marker = ball_center + r_ball * ...
        [cos(phi(k)); sin(phi(k))];
    plot(ball_marker(1), ball_marker(2), 'ro', ...
        'MarkerSize',8,'MarkerFaceColor','r')
    
    % --- Rod direction ---
    rod_dir = [sin(theta_x(k)); cos(theta_x(k))];
    
    % Wheel center (distance r_ball + r_virt along rod)
    wheel_center = ball_center + (r_ball + r_virt) * rod_dir;
    
    % --- Wheel circle ---
    x_wheel_circ = wheel_center(1) + r_virt*cos(ang);
    y_wheel_circ = wheel_center(2) + r_virt*sin(ang);
    plot(x_wheel_circ, y_wheel_circ, 'k', 'LineWidth', 2)
    
    % --- Wheel rotation marker ---
    wheel_marker = wheel_center + r_virt * ...
        [cos(psi(k)); sin(psi(k))];
    plot(wheel_marker(1), wheel_marker(2), 'go', ...
        'MarkerSize',8,'MarkerFaceColor','g')
    
    % --- Rod to CG ---
    CG = ball_center + l * rod_dir;
    plot([ball_center(1) CG(1)], ...
         [ball_center(2) CG(2)], ...
         'm','LineWidth',2)
    
    % CG marker
    plot(CG(1), CG(2), 'ks', ...
        'MarkerSize',8,'MarkerFaceColor','y')
    
    % Axis limits follow translation
    xlim([x_ball-2, x_ball+2])
    ylim([-0.1 2])
    
    drawnow
end
end