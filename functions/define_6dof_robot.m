function robot = define_6dof_robot()
% DEFINE_6DOF_ROBOT - Creates a struct with a 6-DOF arm's parameters.
%
%   This defines a generic 6-DOF arm, similar in structure to many
%   industrial arms (e.g., a PUMA or Stanford arm).
%
%   Outputs:
%       robot - A struct containing all robot parameters.

    % Define Link Lengths (in mm)
    robot.L.L1 = 100; % Base height
    robot.L.L2 = 80;  % Shoulder to elbow
    robot.L.L3 = 70;  % Elbow to wrist
    robot.L.L4 = 20;  % Wrist segment 1
    robot.L.L5 = 20;  % Wrist segment 2
    robot.L.L6 = 10;  % End effector

    % Degrees of Freedom
    robot.dof = 6;

    % Denavit-Hartenberg (D-H) Parameters
    % (Based on a standard all-revolute "elbow" manipulator)
    
    % alpha (twist) - The angle about the new x-axis
    robot.alpha = [-pi/2, 0, -pi/2, pi/2, -pi/2, 0];
    
    % a (link length) - The distance along the new x-axis
    robot.a = [0, robot.L.L2, robot.L.L3, 0, 0, 0];
    
    % d (link offset) - The distance along the previous z-axis
    robot.d = [robot.L.L1, 0, 0, robot.L.L4 + robot.L.L5, 0, robot.L.L6];

end
