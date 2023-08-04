%% Generate Trajectory Function to test servo tracking algorithms
% Types of possible trajectories:
% 1. A triquetra pattern however this is not entrely working as each data
% point is in a random spot due to how this is generated. Don't use, not
% worth it. May fix at some point

% 2. A 2D line of 200 points following the equation y = -x + 4

% 3. A 2D circle with radius 400 about the origin

% 4. A 3D circle with radius 400 about the origin and fixed elevation of 40

function traj = generate_trajectory(type)


    if type == 1
        % Define the parameters
        numPoints = 1000;
        velocity = 4.5; % m/s
        boxSize = 200; % mm
        
        % Convert velocity to mm/s
        velocity_mm = velocity * 1000;
        
        % Generate the coordinates
        coordinates = zeros(numPoints, 2);
        angle = linspace(0, 4*pi, numPoints); % Create angles for the figure 8 pattern
        
        for t = 1:numPoints
            % Calculate x and y coordinates using the figure 8 pattern
            x = boxSize * sin(angle(t)) + boxSize * sin(2 * angle(t));
            y = boxSize * cos(angle(t)) - boxSize * cos(2 * angle(t));
        
            % Ensure the point is within the box
            x = max(min(x, boxSize), -boxSize);
            y = max(min(y, boxSize), -boxSize);
        
            % Store the coordinates
            coordinates(t, :) = [x, y];
        
            % Update the angle based on velocity
            angle = angle + (velocity_mm / boxSize);
        end
        
        % Plot the coordinates
    %     figure;
    %     plot(coordinates(:, 1), coordinates(:, 2), '.');
    %     axis([-boxSize boxSize -boxSize boxSize]);
    %     title('Triquetra Pattern');
    %     xlabel('X (mm)');
    %     ylabel('Y (mm)');
    
        
    elseif type == 2
        % trajectory for a single line with negative gradient
        coordinates = zeros(200, 2);

        for x = 1:200
            y = -x + 4;
            coordinates(x,:) = [x,y];
        end
        
    elseif type == 3
        % trajectory for a circle with radius 40 about 0, 0
        r = 400;
        data_points = 200;
        coordinates = zeros(data_points, 2);
        theta = linspace(0, 2*pi, data_points);
        coordinates(:,1) = r * cos(theta);
        coordinates(:,2) = r * sin(theta);
     elseif type == 4
        % trajectory for a circle in 3D space with radius 400 about 0, 0, 0
        % at a fixed elevation
        r = 400;
        data_points = 200;
        coordinates = zeros(data_points, 3);
        theta = linspace(0, 2*pi, data_points);
        coordinates(:,1) = r * cos(theta);
        coordinates(:,2) = r * sin(theta);
        coordinates(:,3) = 800;
     elseif type == 5
        % trajectory for a circle in 3D space with radius 400 about 0, 0, 0
        % at a changing elevation
        r = 400;
        data_points = 200;
        coordinates = zeros(data_points, 3);
        theta = linspace(0, 2*pi, data_points);
        coordinates(:,1) = r * cos(theta);
        coordinates(:,2) = r * sin(theta);
        coordinates(:,3) = 50 + 50 * cos(theta);
     elseif type == 6
        % trajectory for a circle in 3D space with radius 400 about 0, 0, 0
        % at a changing elevation
        r = 400;
        data_points = 400;
        coordinates = zeros(data_points, 4);
        theta = linspace(0, 2*pi, data_points);
        coordinates(:,1) = r * cos(theta);
        coordinates(:,2) = r * sin(theta);
        coordinates(:,3) = 45 + 50 * cos(theta);
        coordinates(:,4) = linspace(0, 20, data_points);
      elseif type == 7
        % trajectory for a circle in 3D space with radius 400 about 0, 0, 0
        % at a fixed elevation
        r = 400;
        data_points = 200;
        coordinates = zeros(data_points, 3);
        theta = linspace(0, 200, data_points);
        coordinates(:,1) = theta;
        coordinates(:,2) = -theta + 4;
        coordinates(:,3) = theta./2 + 60;
      elseif type == 8
        % trajectory for a circle in 3D space with radius 400 about 0, 0, 0
        % at a changing elevation
        r = 400;
        data_points = 10;
        coordinates = zeros(data_points, 4);
        theta = linspace(0, 2*pi, data_points);
        coordinates(:,1) = r * cos(theta);
        coordinates(:,2) = r * sin(theta);
        coordinates(:,3) = 45 + 50 * cos(theta);
        coordinates(:,4) = linspace(0, 20, data_points);
    end


    traj = coordinates;
end