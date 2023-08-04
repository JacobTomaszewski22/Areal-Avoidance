%% Position Tracking Simulation Yaw Plane - Started: 22/06/23
%% Initialise Begining Parameters
% Start by instatiating the positions, rotations and trajectory
clf
clear

trajectory = generate_trajectory(3);
% set the servo position
S = [-180, -180];

%Constants for ease of use
X = 1;
Y = 2;

quadrant = 0;
%% Calibration simulation
% This would in practice be captured by the MOCAP system, but we need to
% provide these features
% B is the angle on the servo at 1500 ms - The mid point
% We provide it as a angle from 0 degrees in cartesian/euclidean space
B = 50;

% We know that the two other limits, at 1000ms and 2000ms are + & - 60
A = B - 60;     % Bottom limit
C = B + 60;     % Upper limit


%% Figure display
f = figure(1);
boxSize = 200; % mm
%axis([-boxSize boxSize -boxSize boxSize]);
hold on
grid on
fig_traj = plot(trajectory(:, 1), trajectory(:, 2), '.');
fig_servo = plot(S(1), S(2), '*r');
line_length = 100;
fig_top_bound = line([S(1, 1) S(1, 1) + line_length*cosd(C)], [S(1, 2) S(1, 2) + line_length*sind(C)], 'Color', 'red', 'LineStyle', '--');
fig_bottom_bound = line([S(1, 1) S(1, 1) + line_length*cosd(A)], [S(1, 2) S(1, 2) + line_length*sind(A)], 'Color', 'red', 'LineStyle', '--');
legend('Trajectory','Servo','Limits', ' ', 'Servo Direction')


%% Simulation
% Step through the trajectory
for i = 1:length(trajectory)

    % Pull off one trajecotry coordinate
    T = [trajectory(i,1), trajectory(i,2)];

    % Find the euclidean angle between the coordinate and the target
    % angle = asind((T(Y)-S(Y))/sqrt((T(X)-S(X))^2 + (T(Y)-S(Y))^2));

    % Convert that angle into a yaw angle for the servo ensuring the
    % quarters of where the target is compared to the servo is taken into
    % account
    % Also important to use the absolute value of A as we only want the
    % angle from begining (1000ms) to the angle
    % This method assumes the servos are placed in the bottom left quadrant
    % of the environment space
%     if T(Y) > S(Y) && T(X) > S(X) %quadrant 1
%         angle = asind((T(Y)-S(Y))/sqrt((T(X)-S(X))^2 + (T(Y)-S(Y))^2));
%         yawAngle = abs(A) + angle;
%         quadrant = 1;
%     elseif T(Y) < S(Y) && T(X) > S(X)   %quadrant 3
%         angle = asind(-(T(Y)-S(Y))/sqrt((T(X)-S(X))^2 + (T(Y)-S(Y))^2));
%         yawAngle = abs(A) - angle;
%         quadrant = 3;
%     elseif T(Y) > S(Y) && T(X) < S(X)   %quadrant 2
%         angle = 180 - asind((T(Y)-S(Y))/sqrt((T(X)-S(X))^2 + (T(Y)-S(Y))^2));
%         yawAngle = abs(A) + angle;
%         quadrant = 2;
%     elseif T(Y) > S(Y) && T(X) == S(X)
%         angle = asind((T(Y)-S(Y))/sqrt((T(X)-S(X))^2 + (T(Y)-S(Y))^2));
%         yawAngle = abs(A) + 90;
%     elseif T(Y) == S(Y) && T(X) > S(X)
%         angle = asind((T(Y)-S(Y))/sqrt((T(X)-S(X))^2 + (T(Y)-S(Y))^2));
%         yawAngle = abs(A);
%     end


%  Attempting to implement a atan2 method where the servo can be placed
%  anywhere
    angle = rad2deg(atan2(T(Y) - S(Y), T(X) - S(X)));
    yawAngle = abs(A) + angle;

    
    % Check the angle is within the boundary of the servos and then convert
    % to duty cycle value
%     if yawAngle <= 120 && yawAngle >= 0
%         %convert to duty cycle
%         dutyPeriod = convert_to_duty_cycle(yawAngle);
% 
%     elseif yawAngle < 0
%         % send 0 to duty cycle to be closest to it
%         yawAngle = 0;
%         dutyPeriod = convert_to_duty_cycle(yawAngle);
% 
%     elseif yawAngle > 120
%         %send 120 to duty cycle to be closest
%         yawAngle = 120;
%         dutyPeriod = convert_to_duty_cycle(yawAngle);
%     end
    [dutyPeriod, yawAngle] = convert_and_bound(yawAngle);

    %Display what has been calculated on the figure
    posn = [0.9 0.5, 0.1, 0.2];

    if quadrant == 3
    angle = 360 - angle;
    end

    if i == 1
        fig_servo_point = line([S(1, 1) S(1, 1) + sqrt((T(X)-S(X))^2 + (T(Y)-S(Y))^2)*cosd(angle)], [S(1, 2) S(1, 2) + sqrt((T(X)-S(X))^2 + (T(Y)-S(Y))^2)*sind(angle)], 'Color', 'black');
        %written = text(S(1) + 5, S(2) + 5, "Angle: " + angle + ". Yaw angle: " + yawAngle + ". Duty Cycle: " + dutyPeriod);
        ann = annotation('textbox', posn, 'string', "Angle: " + angle + ". Yaw angle: " + yawAngle + ". Duty Cycle: " + dutyPeriod, 'EdgeColor', 'none')
    else
        delete(fig_servo_point);
        fig_servo_point = line([S(1, 1) S(1, 1) + sqrt((T(X)-S(X))^2 + (T(Y)-S(Y))^2)*cosd(angle)], [S(1, 2) S(1, 2) + sqrt((T(X)-S(X))^2 + (T(Y)-S(Y))^2)*sind(angle)], 'Color', 'black');
        %delete(written);
        %written = text(S(1) + 5, S(2) + 5, "Angle: " + angle + ". Yaw angle: " + yawAngle + ". Duty Cycle: " + dutyPeriod);
        delete(ann);
        ann = annotation('textbox', posn, 'string', "Angle: " + angle + ". Yaw angle: " + yawAngle + ". Duty Cycle: " + dutyPeriod, 'EdgeColor', 'none')
    end

    %ann = annotation('textbox', [0, 0, 0, 0], 'string', "Angle: " + angle + ". Yaw angle: " + yawAngle + ". Duty Cycle: " + dutyPeriod);
    
    pause(0.1)
    
end