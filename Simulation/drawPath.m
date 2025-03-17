x = out.path(1).data(:, 1);
y = out.path(1).data(:, 2);
psi = out.path(1).data(:, 3);  % Orientation angles
steer = out.path(1).data(:, 4);  % Steering angles (in radians)

% Create the figure
figure;
hold on
plot(x, y, 'r--', 'LineWidth', 2)  % Plot the vehicle path
plot(xFine, yFine, 'b')  % Plot the planned path
legend("Vehicle Path", "Planned Path", "Vehicle", "Steering")
grid on
axis equal

% Vehicle dimensions
vehicleWidth = 10;
vehicleLength = l;  % Length of the vehicle

% Set up the vehicle rectangle (initial position)
vehiclePlot = patch('XData', [], 'YData', [], 'FaceColor', 'g', 'EdgeColor', 'g');

% Set up the arrow (initial position)
arrowLength = 15;  % Length of the steering arrow
arrowPlot = quiver(0, 0, 0, 0, 0, 'MaxHeadSize', 2, 'Color', 'g', 'LineWidth', 1);

% Animation loop
for k = 1:length(x)
    % Get the current orientation
    theta = psi(k);  % Orientation angle (in radians)
    
    % Define the 4 corners of the rectangle (before rotation)
    rectX = [0, vehicleLength, vehicleLength, 0];
    rectY = [-vehicleWidth/2, -vehicleWidth/2, vehicleWidth/2, vehicleWidth/2];
    
    % Rotation matrix
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    
    % Apply the rotation to the corners
    rotatedCorners = R * [rectX; rectY];
    
    % Get the new X and Y positions of the rotated rectangle corners
    xRot = rotatedCorners(1, :) + x(k);
    yRot = rotatedCorners(2, :) + y(k);
    
    % Update the rectangle (vehicle) position
    set(vehiclePlot, 'XData', xRot, 'YData', yRot);
    
    % Calculate the arrow (steering direction)
    steerAngle = steer(k);  % Steering angle (in radians)
    
    % Calculate the position of the arrow's root (in front of the vehicle)
    arrowX = x(k) + vehicleLength * cos(theta);
    arrowY = y(k) + vehicleLength * sin(theta);
    
    % Calculate the direction of the steering
    steerX = arrowX + arrowLength * cos(theta + steerAngle);
    steerY = arrowY + arrowLength * sin(theta + steerAngle);
    
    % Update the arrow (steering direction)
    set(arrowPlot, 'XData', arrowX, 'YData', arrowY, 'UData', steerX - arrowX, 'VData', steerY - arrowY);
    
    % Update the plot
    drawnow;  
    
    % Pause to create the animation effect
    pause(0.000001);  % Adjust the speed of the animation
end

hold off;
