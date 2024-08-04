% Define DH parameters for Franka Emika Panda (Modified DH Parameters)
a = [0, 0, 0.0825, -0.0825, 0, 0.088, 0];
alpha = [0, pi/2, pi/2, -pi/2, pi/2, pi/2, 0];
d = [0.333, 0, 0.316, 0, 0.384, 0, 0.107];
theta = [0, -pi/2, pi/2, 0, 0, 0, 0]; % Initial joint angles

% Number of joints
n = length(a);

% Initialize arrays to store joint trajectories
joint_trajectories = cell(n+1, 1);
for i = 1:n+1
    joint_trajectories{i} = [];
end

% Set up the figure for the animation
figure;
hold on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Franka Emika Panda Robot Animation with Joint Trajectories');
grid on;
axis equal;
xlim([-1, 1]);
ylim([-1, 1]);
zlim([0, 1]);

% Animation loop
for step = 1:360
    % Update the joint angles by 0.1 degrees (converted to radians)
    theta = theta + deg2rad(0.1);
    
    % Initialize the transformation matrix and joint positions
    ee_M_0 = eye(4);
    joint_positions = zeros(n+1, 3); 
    joint_positions(1, :) = [0, 0, 0]; 

    % Compute the transformation matrix and joint positions
    for i = 1:n
        T_i = dh_transform(a(i), alpha(i), d(i), theta(i));
        ee_M_0 = ee_M_0 * T_i;
        joint_positions(i+1, :) = ee_M_0(1:3, 4)';
    end
    
    % Store the current joint positions in the trajectories
    for i = 1:n+1
        joint_trajectories{i} = [joint_trajectories{i}; joint_positions(i, :)];
    end
    
    % Clear the previous plot
    cla;
    
    % Plot the robot segments in 3D
    for i = 1:n
        % Plot each segment with a different color
        plot3([joint_positions(i, 1), joint_positions(i+1, 1)], ...
              [joint_positions(i, 2), joint_positions(i+1, 2)], ...
              [joint_positions(i, 3), joint_positions(i+1, 3)], ...
              'LineWidth', 2, 'Color', rand(1,3), 'DisplayName', ['Link ' num2str(i)]);
    end
    
    % Plot end-effector position
    plot3(joint_positions(end, 1), joint_positions(end, 2), joint_positions(end, 3), 'ko', 'MarkerFaceColor', 'k', 'DisplayName', 'End-Effector');
    
    % Plot joint trajectories
    for i = 1:n+1
        plot3(joint_trajectories{i}(:, 1), joint_trajectories{i}(:, 2), joint_trajectories{i}(:, 3), '--');
    end
    
    % Pause for 0.5 seconds to create the animation effect
    pause(0.1);
end

hold off;

% Function to compute DH transformation matrix
function T = dh_transform(a, alpha, d, theta)
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),            cos(alpha),            d;
         0,           0,                     0,                     1];
end

