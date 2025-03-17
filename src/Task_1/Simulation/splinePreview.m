% MATLAB script to generate a clothoid path between waypoints using MATLAB's built-in Clothoid functions

clc;
clear;
close all;

% Define waypoints
x = [ 0, 40, 66.5 * sin(pi / 4) + 40, 66.5 + 40, 106, 106, 100 * sin(pi / 4) + 6, 6, -100 * sin(pi / 4) + 6, -96, -96, -96, 66.5 * sin(pi / 4) - 96 - 40 * cos(pi / 4), -40, 0];

y = [0, 0, 66.5 * (-cos(pi / 4) + 1), 66.5, 116, 266, 266 + 100 * sin(pi / 4), 366, 266 + 100 * sin(pi / 4), 266, 116, 66.5, 66.5 * (-cos(pi / 4) + 1), 0, 0];

% Compute angles between waypoints
theta = [0, 0, pi / 4, pi / 2, pi / 2, pi / 2, pi / 4 + pi / 2, pi, pi + pi / 4, 3*pi/2, 3*pi/2, 3*pi/2, 3*pi/2 + pi/4, 2*pi, 0];

% Initialize clothoid parameters
x_vals = [];
y_vals = [];

figure;
hold on;
for i = 1:length(theta)-1
    % Generate clothoid using MATLAB built-in function
    clothoidSegment = clothoid(x(i), y(i), theta(i), 0, 0, sqrt((x(i+1)-x(i))^2 + (y(i+1)-y(i))^2));
    
    % Sample points along the clothoid
    s_vals = linspace(0, clothoidSegment.Length, 100);
    x_segment = arrayfun(@(s) clothoidSegment.x(s), s_vals);
    y_segment = arrayfun(@(s) clothoidSegment.y(s), s_vals);
    
    x_vals = [x_vals, x_segment];
    y_vals = [y_vals, y_segment];
    
    plot(x_segment, y_segment, 'b');
end

% Plot waypoints
scatter(x, y, 'ro', 'filled');
xlabel('X Position');
ylabel('Y Position');
title('Clothoid Path using MATLAB Built-in Function');
grid on;
axis equal;
hold off;
