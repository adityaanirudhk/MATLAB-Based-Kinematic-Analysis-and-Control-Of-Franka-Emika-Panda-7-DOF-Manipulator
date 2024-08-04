% Define DH parameters for Franka Emika Panda (Modified DH Parameters)
a = [0, 0, 0, 0.0825, -0.0825, 0, 0.088, 0];
alpha = [0, pi/2, 0, pi/2, -pi/2, pi/2, 0, 0];
d = [0.333, 0, 0.316, 0, 0.384, 0, 0.107, 0.015];
theta = [0, -pi/2, pi/2, 0, 0, 0, 0, 0]; % Initial joint angles

% Number of joints
n = length(a);

% Function to compute DH transformation matrix
function T = dh_transform(a, alpha, d, theta)
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),            cos(alpha),            d;
         0,           0,                     0,                     1];
end

% Function to plot the robot configuration
function plot_robot(theta, a, alpha, d)
    n = length(a);
    T_ee_0 = eye(4);
    joint_positions = zeros(n+1, 3); 
    joint_positions(1, :) = [0, 0, 0];
    
    % Define colors for each segment
    colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k', 'w'];

    figure;
    hold on;
    for i = 1:n
        T_i = dh_transform(a(i), alpha(i), d(i), theta(i));
        T_ee_0 = T_ee_0 * T_i;
        joint_positions(i+1, :) = T_ee_0(1:3, 4)';
        
        % Plot the segment
        plot3([joint_positions(i, 1), joint_positions(i+1, 1)], ...
              [joint_positions(i, 2), joint_positions(i+1, 2)], ...
              [joint_positions(i, 3), joint_positions(i+1, 3)], ...
              'LineWidth', 2, 'Color', colors(mod(i-1, length(colors))+1), 'DisplayName', ['Link ' num2str(i)]);
    end

    % Plot end-effector position
    plot3(joint_positions(end, 1), joint_positions(end, 2), joint_positions(end, 3), 'ko', 'MarkerFaceColor', 'k', 'DisplayName', 'End-Effector');

    % Set plot properties
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Franka Emika Panda Robot Segments in 3D by Aditya');
    grid on;
    axis equal;
    legend;
    hold off;
end

% Define initial and target joint configurations
initial_q = [0, -pi/2, pi/2, 0, 0, 0, 0, 0];
target_q = [pi/6, -pi/4, pi/3, -pi/6, pi/4, 0, pi/6, 0];

% Number of animation steps
n_steps = 100;

% Create the animation
figure;
for step = 1:n_steps
    % Interpolate joint angles
    q_current = initial_q + (target_q - initial_q) * (step / n_steps);
    
    % Plot the robot
    plot_robot(q_current, a, alpha, d);
    
    % Pause for a short time to create animation effect
    pause(0.1);
end
