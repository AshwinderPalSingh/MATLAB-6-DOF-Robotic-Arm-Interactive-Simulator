% MAIN SCRIPT: 6-DOF ROBOTIC ARM INTERACTIVE SIMULATOR
%
% Description:
% This script launches a Graphical User Interface (GUI) to control
% a 6-DOF robotic arm. It uses MATLAB's UI Figure to create sliders
% for each joint and a 3D plot for visualization.


%% 1. Environment Setup
clc;
clear;
close all;

% Add the 'functions' folder to the MATLAB path
addpath('functions');

% Load the robot's parameters
robot = define_6dof_robot();

% Define initial joint angles (degrees)
initial_angles = [0, 0, 0, 0, 0, 0];

%% 2. Create the GUI Figure
app.UIFigure = uifigure('Name', '6-DOF Arm Simulator', 'Position', [100, 100, 1000, 600]);

% Create 3D Plotting Area (Axes)
app.UIAxes = uiaxes(app.UIFigure, 'Position', [20, 20, 560, 560]);
app.UIAxes.View = [120, 20];
axis(app.UIAxes, 'equal');
grid(app.UIAxes, 'on');
xlabel(app.UIAxes, 'X-axis (mm)');
ylabel(app.UIAxes, 'Y-axis (mm)');
zlabel(app.UIAxes, 'Z-axis (mm)');
title(app.UIAxes, '6-DOF Robot Arm');

% Create a panel for controls
app.ControlPanel = uipanel(app.UIFigure, 'Title', 'Joint Controls', 'Position', [600, 20, 380, 560]);

% Create sliders and labels
app.Sliders = gobjects(robot.dof, 1);
app.AngleLabels = gobjects(robot.dof, 1);
slider_y_pos = 500; % Starting Y position for the first slider

for i = 1:robot.dof
    % Joint Label (e.g., "Q1:")
    uilabel(app.ControlPanel, 'Text', ['Q', num2str(i), ':'], 'Position', [20, slider_y_pos, 40, 22]);
    
    % Angle Value Label (e.g., "0 deg")
    app.AngleLabels(i) = uilabel(app.ControlPanel, 'Text', [num2str(initial_angles(i)), ' deg'], 'Position', [300, slider_y_pos, 70, 22]);
    
    % Joint Slider
    app.Sliders(i) = uislider(app.ControlPanel, ...
        'Limits', [-180, 180], ...
        'Value', initial_angles(i), ...
        'Position', [60, slider_y_pos + 10, 230, 3], ...
        'MajorTicks', [-180, -90, 0, 90, 180]);
    
    % DO NOT set callback function inside this loop
    
    % Decrement Y position for the next set of controls
    slider_y_pos = slider_y_pos - 60;
end

% Set slider callbacks AFTER the 'app' struct is fully populated
% This is the fix: It ensures the 'app' struct passed to the
% callback function is complete and contains all 6 sliders.
for i = 1:robot.dof
    app.Sliders(i).ValueChangedFcn = @(src, event) update_robot_plot(app, robot);
end

% Create panel for End-Effector Position
app.PosPanel = uipanel(app.ControlPanel, 'Title', 'End-Effector Position (mm)', 'Position', [20, 20, 340, 130]);

% X Position
uilabel(app.PosPanel, 'Text', 'X:', 'Position', [20, 80, 20, 22], 'FontWeight', 'bold');
app.XPos = uieditfield(app.PosPanel, 'numeric', 'Value', 0, 'Position', [50, 80, 270, 22], 'Editable', 'off');

% Y Position
uilabel(app.PosPanel, 'Text', 'Y:', 'Position', [20, 50, 20, 22], 'FontWeight', 'bold');
app.YPos = uieditfield(app.PosPanel, 'numeric', 'Value', 0, 'Position', [50, 50, 270, 22], 'Editable', 'off');

% Z Position
uilabel(app.PosPanel, 'Text', 'Z:', 'Position', [20, 20, 20, 22], 'FontWeight', 'bold');
app.ZPos = uieditfield(app.PosPanel, 'numeric', 'Value', 0, 'Position', [50, 20, 270, 22], 'Editable', 'off');

%% 3. Initial Plot
% Call the update function once to draw the robot in its initial position
update_robot_plot(app, robot);


%% 4. CALLBACK FUNCTION (The "Brain" of the GUI)
% This function is defined locally within the main script.

function update_robot_plot(app, robot)
% UPDATE_ROBOT_PLOT - Reads slider values, computes FK, and updates the plot.
%
%   Inputs:
%       app   - The GUI object, containing all UI elements
%       robot - The robot parameter struct

    % 1. Read all slider values
    q_deg = zeros(1, robot.dof);
    for i = 1:robot.dof
        q_deg(i) = app.Sliders(i).Value;
        % Update the text label for the angle
        app.AngleLabels(i).Text = [num2str(q_deg(i), '%.1f'), ' deg'];
    end
    
    % 2. Compute Forward Kinematics
    [T_final, all_positions] = compute_fk(robot, q_deg);
    
    % 3. Get End-Effector Position
    end_effector_pos = all_positions(:, end);
    
    % 4. Update the X, Y, Z text fields
    app.XPos.Value = round(end_effector_pos(1), 4);
    app.YPos.Value = round(end_effector_pos(2), 4);
    app.ZPos.Value = round(end_effector_pos(3), 4);
    
    % 5. Update the 3D Plot
    % Clear the specific axes for the new plot
    cla(app.UIAxes);
    
    % Extract x, y, z
    x = all_positions(1, :);
    y = all_positions(2, :);
    z = all_positions(3, :);
    
    % Plot links and joints
    plot3(app.UIAxes, x, y, z, '-o', ...
          'Color', [0, 0.4470, 0.7410], ...
          'LineWidth', 2.5, ...
          'MarkerSize', 8, ...
          'MarkerFaceColor', 'r');
      
    % Add a black square for the base
    hold(app.UIAxes, 'on');
    plot3(app.UIAxes, 0, 0, 0, 'ks', 'MarkerSize', 12, 'MarkerFaceColor', 'k');
    hold(app.UIAxes, 'off');
    
    % Set plot limits dynamically to keep arm in view
    % (with a minimum size)
    max_range = max([max(abs(x)), max(abs(y)), max(abs(z)), 200]); % 200mm min range
    axis(app.UIAxes, [-max_range, max_range, -max_range, max_range, 0, max_range * 1.5]);
    
    % Re-apply labels and grid
    grid(app.UIAxes, 'on');
    xlabel(app.UIAxes, 'X-axis (mm)');
    ylabel(app.UIAxes, 'Y-axis (mm)');
    zlabel(app.UIAxes, 'Z-axis (mm)');
    title(app.UIAxes, '6-DOF Robot Arm');
    
end
