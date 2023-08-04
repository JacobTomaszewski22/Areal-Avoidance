%% Establishing object orientations
% These values will come from MOCAP
assembly = [-30, -20, 2; 0, 0, 0];     %The 6 dof position of the servo assembly
                                        % the assembly can only rotate about pitch (2,1)
                                        % and yaw (2,3)
target = [-15, 10, 5; 0, 0, 0];

%% Yaw angle on the XY Plane using origin as third point
%finding distances of the angles
TO = sqrt(target(1,1)^2 + target(1,2)^2);
SO = sqrt(assembly(1,1)^2 + target(1,2)^2);
TS = sqrt((assembly(1,1) - target(1,1))^2 + (assembly(1,2) - target(1,2))^2);

inside_angle = acosd((SO^2 + TS^2 - TO^2)/2 * SO * TS);

%% Yaw angle on XY plane using a right angle triangle
desired_angle = asind((target(1,2)-assembly(1,2))/sqrt((target(1,1)-assembly(1,1))^2 + (target(1,2)-assembly(1,2))^2));

%% Plotting 
figure(1)
axis([-40 40 -40 40])
grid on
hold on
plot(0, 0, '.')
plot(assembly(1, 1), assembly(1, 2), '*')
plot(target(1, 1), target(1, 2), '*b')
line([target(1, 1) assembly(1, 1)], [target(1, 2) assembly(1, 2)])

%% Time going forward
t = 0:0.01:10;
Angular_Velocity = 352.94; % degrees per second
gain = 0.8;
threshold = 0.005;
error = 0;
previous_error = 0;
random_range = [-4 0.2];

for i = 1:length(t)
    %Step 1: If on first movement, use feedforward control. Here we
    %introduce noise to simulate real world
    if i == 1
        % This is sent to the servos
        assembly(2,3) = desired_angle + (random_range(1)) + ((random_range(2)) - (random_range(1))) .* rand(1,1);
    end
    
    % Calculate error between angles. The angle of servo is gathered from MOCAP
    error = desired_angle - assembly(2,3)  %error = desired angle - sensed angle
    
    if abs(error) <= threshold
        break
    end

    %reduces the noise incrementally for simulation's sake
    %random_range = random_range./1.1;
    
    % sets the yaw value to be the error, here is where it would be 'sent'
    % to the servos
    assembly(2,3) = error + assembly(2,3)+ (random_range(1)) + ((random_range(2)) - (random_range(1))) .* rand(1,1);
    line([assembly(1, 1) assembly(1, 1)+cosd(assembly(2,3))], [assembly(1, 2) assembly(1, 2)+sind(assembly(2,3))], 'Color', 'red', 'LineStyle', '--')
    pause(0.01);
end



