% Define DH parameters for Franka Emika Panda (Modified DH Parameters)
a = [0, 0, 0.0825, -0.0825, 0, 0.088, 0];
alpha = [0, pi/2, pi/2, -pi/2, pi/2, pi/2, 0];
d = [0.333, 0, 0.316, 0, 0.384, 0, 0.107];
theta = [0, -pi/2, pi/2, 0, 0, 0, 0]; % Initial joint angles

% Number of joints
n = length(a);

% Define different q configurations
q_configs = [
    -pi/4, -pi/6, pi/6, -pi/6, pi/6, 0, pi/6;
    pi/6,  pi/4, -pi/4, -pi/4, -pi/6, pi/6, -pi/6;
    pi/3, -pi/4, pi/3, -pi/2, pi/3, -pi/3, pi/4;
];

% Initialize the transformation matrix
ee_M_0 = eye(4);

% Initialize the Jacobian matrix M
M = zeros(6, n);

% Initialize array to store joint positions
joint_positions = zeros(n+1, 3); 
joint_positions(1, :) = [0, 0, 0]; 

% Compute the transformation matrix, joint positions, and M matrix
for i = 1:n
    T_i = dh_transform(a(i), alpha(i), d(i), theta(i));
    ee_M_0 = ee_M_0 * T_i;
    joint_positions(i+1, :) = ee_M_0(1:3, 4)';
    
    % Calculate the rotation matrix
    R = ee_M_0(1:3, 1:3);
    
    % Calculate the z-axis
    z = ee_M_0(1:3, 3);
    
    % Calculate the position vector from the base to the end-effector
    p = ee_M_0(1:3, 4);
    
    % Calculate the Jacobian M matrix (using the position and rotation)
    M(1:3, i) = cross(z, p - joint_positions(i, :)'); % Linear velocity component
    M(4:6, i) = z; % Angular velocity component
end

% Display the final transformation matrix
disp('The final transformation matrix ee_M_0 is:');
disp(ee_M_0);

% Display the Jacobian matrix
disp('The Jacobian matrix M is:');
disp(M);

% Plotting the robot segments in 3D
figure;
hold on;

for i = 1:n
    % Plot each segment with a different color
    plot3([joint_positions(i, 1), joint_positions(i+1, 1)], ...
          [joint_positions(i, 2), joint_positions(i+1, 2)], ...
          [joint_positions(i, 3), joint_positions(i+1, 3)], ...
          'LineWidth', 2, 'Color', rand(1,3), 'DisplayName', ['Link ' num2str(i)]);
end

% Plot end-effector position for the initial configuration
plot3(joint_positions(end, 1), joint_positions(end, 2), joint_positions(end, 3), 'ko', 'MarkerFaceColor', 'k', 'DisplayName', 'End-Effector');

% Calculate and plot the end-effector positions for each q configuration
for j = 1:size(q_configs, 1)
    q = q_configs(j, :);
    
    % Reset the transformation matrix for each configuration
    ee_M_0 = eye(4);
    for i = 1:n
        T_i = dh_transform(a(i), alpha(i), d(i), q(i));
        ee_M_0 = ee_M_0 * T_i;
    end
    
    % Extract and plot the end-effector position
    end_effector_position = ee_M_0(1:3, 4)';
    plot3(end_effector_position(1), end_effector_position(2), end_effector_position(3), 'o-', 'LineWidth', 2, 'DisplayName', ['Config ' num2str(j)]);
end

% Set plot properties
xlabel('X');
ylabel('Y');
zlabel('Z');
title("Aditya's way of plotting the Franka Emika");
grid on;
axis equal;

% Add legend
legend;

hold off;

%%computing the inverse of the Jacobian matrix
J_pseudo_inv=pinv(M);
disp('The inverse of the Jacobian matrix J is:');
disp(J_pseudo_inv)
% Function to compute DH transformation matrix
function T = dh_transform(a, alpha, d, theta)
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),            cos(alpha),            d;
         0,           0,                     0,                     1];
end