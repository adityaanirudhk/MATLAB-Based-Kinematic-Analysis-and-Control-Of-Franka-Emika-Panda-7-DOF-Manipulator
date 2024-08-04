# Kinematic-Simulation-and-Control-of-Frank-Emika-Panda-7-DOF-Manipulator
This repository contains MATLAB scripts for the kinematic analysis, Jacobian computation, and control of the Franka Emika Panda robotic arm. It includes detailed implementations of the DH parameter transformations, joint space and Cartesian space control, and animation of the robot's movements. The project also explores the use of the pseudo-inverse Jacobian for redundancy resolution in 7-DOF configurations.

![56992707-f02b0e80-6b9a-11e9-99fd-58a31f114d0e](https://github.com/user-attachments/assets/7e23e830-2c94-4922-9be7-c265ca64c6f9)

GIF in courtesy of https://github.com/cpezzato/panda_simulation


1) DEFINING THE DH PARAMETERS-
The first step involves defining the Denavit-Hartenberg (DH) parameters for the Franka Emika Panda robot.
 ```MATLAB
a = [0, 0, 0, 0.0825, -0.0825, 0, 0.088];
alpha = [0, pi/2, 0, pi/2, -pi/2, pi/2, 0];
d = [0.333, 0, 0.316, 0, 0.384, 0, 0.107];
theta = [0, -pi/2, pi/2, 0, 0, 0, 0]; % Initial joint angles
```

2) COMPUTING TRANSFORMATION MATRICES-
Next, the transformation matrices for each joint are computed using the DH parameters.
```MATLAB
% Function to compute DH transformation matrix
function T = dh_transform(a, alpha, d, theta)
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),            cos(alpha),            d;
         0,           0,                     0,                     1];
end

% Initialize the transformation matrix and joint positions
ee_M_0 = eye(4);
joint_positions = zeros(n+1, 3); 
joint_positions(1, :) = [0, 0, 0]; 
```

3) PLOTTING THE ROBOT SEGMENTS IN 3D-
After computing the transformation matrices, the robot segments are plotted in 3D, with each segment represented in a different color.
```MATLAB
% Plot the robot segments in 3D
figure;
hold on;

for i = 1:n
    plot3([joint_positions(i, 1), joint_positions(i+1, 1)], ...
          [joint_positions(i, 2), joint_positions(i+1, 2)], ...
          [joint_positions(i, 3), joint_positions(i+1, 3)], ...
          'LineWidth', 2, 'Color', colors(mod(i-1,length(colors))+1), 'DisplayName', ['Link ' num2str(i)]);
end

% Set plot properties
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Franka Emika Panda Robot Segments in 3D');
grid on;
axis equal;
legend;
hold off;
```

4) COMPUTING THE JACOBIAM MATRIX-
The Jacobian matrix is computed to relate the joint velocities to the end-effector velocity and static forces.
```MATLAB
% Initialize the Jacobian matrix M
M = zeros(6, n);

% Compute the Jacobian matrix
for i = 1:n
    % Code to calculate each column of the Jacobian...
end

% Display the Jacobian matrix
disp('Jacobian matrix M:');
disp(M);
```

5) CONTROLLING THE ROBOT IN JOINT SPACE AND CARTESIAN SPACE-
The robot is controlled in both joint space and Cartesian space, with the corresponding joint velocities and end-effector velocities being calculated and updated.
```MATLAB
% Example code snippet for controlling in joint space
q_dot = ...; % Define joint velocity vector
x_dot = M * q_dot; % Calculate end-effector velocity

% Example code snippet for controlling in Cartesian space
q_dot = pinv(M) * x_dot; % Calculate joint velocities from desired end-effector velocity
```

6) ANIMATING THE ROBOT'S MOVEMENT-
The robot's movement is animated, with updates to the joint values at each time step to visualize the trajectory.
```MATLAB
% Animation loop
while true
    for i = 1:length(q_configs)
        % Update joint angles
        theta = theta + delta_theta; % Update joint angles

        % Recalculate and plot robot configuration
        % ...
        
        pause(0.1); % Pause for animation effect
    end
end
```
7) IMPLEMENTING THE PSEUDO-INVERSE FOR THE MANIPULATOR-
```MATLAB
% Compute the pseudo-inverse of the Jacobian
J_pseudo_inv = pinv(M);

% Use the pseudo-inverse for control
q_dot = J_pseudo_inv * x_dot;
```
![image](https://github.com/user-attachments/assets/c094f5c1-a410-4c68-8374-c3ae8a4dccdc)
