%% Position Tracking Simulation 3D - Started: 27/06/23
%% Initialise Begining Parameters
% Start by instatiating the positions, rotations and trajectory
clf
clear

trajectory = generate_trajectory(8);
% set the servo position
S = [-180, -180, -10];

%Constants for ease of use
X = 1;
Y = 2;
Z = 3;

quadrant = 0;
%% Calibration simulation
% This would in practice be captured by the MOCAP system, but we need to
% provide these features
% B is the angle on the servo at 1500 ms - The mid point
% We provide it as a angle from 0 degrees in cartesian/euclidean space

% Set the yaw constraints
Yaw_Centre = 50;

% We know that the two other limits, at 1000ms and 2000ms are + & - 60
Yaw_Min = Yaw_Centre - 60;     % Bottom limit
Yaw_Max = Yaw_Centre + 60;     % Upper limit

% Set the pitch constraints
Pitch_Centre = 30;
Pitch_Min = Pitch_Centre - 60;
Pitch_Max = Pitch_Centre + 60;


%% Figure display
f = figure(1);
boxSize = 200; % mm

hold on
grid on
fig_traj = plot3(trajectory(:, 1), trajectory(:, 2), trajectory(:,3),'.');
fig_servo = plot3(S(1), S(2), S(3) ,'*r');
line_length = 100;
fig_top_bound = plot3([S(1, 1) S(1, 1) + line_length*cosd(Yaw_Max)], [S(1, 2) S(1, 2) + line_length*sind(Yaw_Max)], [S(1,3) S(1,3)], 'Color', 'red', 'LineStyle', '--');
fig_bottom_bound = plot3([S(1, 1) S(1, 1) + line_length*cosd(Yaw_Min)], [S(1, 2) S(1, 2) + line_length*sind(Yaw_Min)], [S(1,3) S(1,3)], 'Color', 'red', 'LineStyle', '--');
%fig_pitch_top_bound = plot3([S(1,1) S(1,1)], [S(1,2) S(1,2)+ line_length*cosd(Pitch_Max)], [S(1,3) S(1,3) + line_length*sind(Pitch_Max)], 'Color', 'red', 'LineStyle', '-.');
%fig_pitch_bottom_bound = plot3([S(1,1) S(1,1)], [S(1,2) S(1,2)+ line_length*cosd(Pitch_Min)], [S(1,3) S(1,3) + line_length*sind(Pitch_Min)], 'Color', 'red', 'LineStyle', '-.');
square_size = 100;
square = plot3([S(X) S(X) S(X) S(X) S(X)], ...
               [S(Y) S(Y)+square_size S(Y)+square_size S(Y) S(Y)], ...
               [S(Z) S(Z) S(Z)+square_size S(Z)+square_size S(Z)]);

xlabel('X Position')
ylabel('Y Position')
zlabel('Z Position')

%% Simulation
% Step through the trajectory
%for i = 1:length(trajectory)
i = 1;
angle1 = 0;
while true
    
    % Pull off one trajecotry coordinate
    T = [trajectory(i,1), trajectory(i,2), trajectory(i,3)];

    % Find the euclidean angle between the coordinate and the target
    % angle = asind((T(Y)-S(Y))/sqrt((T(X)-S(X))^2 + (T(Y)-S(Y))^2));

    % Convert that angle into a yaw angle for the servo ensuring the
    % quarters of where the target is compared to the servo is taken into
    % account
    % Also important to use the absolute value of A as we only want the
    % angle from begining (1000ms) to the angle
    % This method assumes the servos are placed in the bottom left quadrant
    % of the environment space
    distance = sqrt((T(X)-S(X))^2 + (T(Y)-S(Y))^2 + (T(Z)-S(Z))^2);
    angle1_prev = angle1;
    angle1 = rad2deg(atan2(T(Y) - S(Y), T(X) - S(X)));
    yawAngle = abs(Yaw_Min) + angle1;

    % Calculate the pitch angle
    
    angle2 = rad2deg(atan2(sqrt((T(X)-S(X))^2 + (T(Y)-S(Y))^2), (T(Z)-S(Z))));
    %angle2 = acosd((T(Z)-S(Z))/distance);

    
    pitchAngle = abs(Pitch_Min) + (90 - angle2);

    
    % Check the angle is within the boundary of the servos and then convert
    % to duty cycle value

    [yawDutyPeriod, yawAngle] = convert_and_bound(yawAngle);
    [pitchDutyPeriod, pitchAngle] = convert_and_bound(pitchAngle);

    %Display what has been calculated on the figure
    posn = [0.9 0.5 0.1 0.2];

%     if T(Y) < S(Y) && T(X) > S(X)   %quadrant 3 on xy
%         angle1 = 360 - angle1;
%     end
%     if T(Z) < S(Z) && T(Y) > S(Y)   % quadrant 3 on zy
%         angle2 - 360 - angle2;
%     end

    if i == 1
        target = plot3(T(X), T(Y), T(Z), '.r');
        fig_servo_point = plot3([S(X) S(X) + distance*cosd(angle1)*sind(angle2)], [S(Y) S(Y) + distance*sind(angle1)*sind(angle2)], [S(Z) S(Z) + distance*cosd(angle2)], 'Color', 'black');
        %written = text(S(1) + 5, S(2) + 5, "Angle: " + angle + ". Yaw angle: " + yawAngle + ". Duty Cycle: " + dutyPeriod);
        ann = annotation('textbox', posn, 'string', "XY Angle: " + angle1 + ". Yaw angle: " + yawAngle + ". Yaw Duty Cycle: " + yawDutyPeriod + ". ZY Angle: " + angle2 + ". Pitch Angle: " + pitchAngle + ". Pitch Duty Cycle: " + pitchDutyPeriod, 'EdgeColor', 'none')
        rotate(square,[0 0 1],angle1-90, S);
    else
        delete(target);
        target = plot3(T(X), T(Y), T(Z), '.r');
        delete(fig_servo_point);
        fig_servo_point = plot3([S(X) S(X) + distance*cosd(angle1)*sind(angle2)], [S(Y) S(Y) + distance*sind(angle1)*sind(angle2)], [S(Z) S(Z) + distance*cosd(angle2)], 'Color', 'black');
        %delete(written);
        %written = text(S(1) + 5, S(2) + 5, "Angle: " + angle + ". Yaw angle: " + yawAngle + ". Duty Cycle: " + dutyPeriod);
        delete(ann);
        ann = annotation('textbox', posn, 'string', "XY Angle: " + angle1 + ". Yaw angle: " + yawAngle + ". Yaw Duty Cycle: " + yawDutyPeriod + ". ZY Angle: " + angle2 + ". Pitch Angle: " + pitchAngle + ". Pitch Duty Cycle: " + pitchDutyPeriod, 'EdgeColor', 'none')
        rotate(square,[0 0 1], angle1-angle1_prev, S);
    end
    legend('Trajectory','Servo','Yaw Limits', ' ', 'Pitch Limits', ' ', 'Servo Direction')

%     if i == 1
%         fig_servo_point = plot3([S(1, 1) S(1, 1) + sqrt((T(X)-S(X))^2 + (T(Y)-S(Y))^2)*cosd(angle1)], [S(1, 2) S(1, 2) + sqrt((T(X)-S(X))^2 + (T(Y)-S(Y))^2)*sind(angle1)], [S(1,3) S(1,3) + sqrt((T(Y)-S(Y))^2 + (T(Z)-S(Z))^2)*sind(angle2)], 'Color', 'black');
%         %written = text(S(1) + 5, S(2) + 5, "Angle: " + angle + ". Yaw angle: " + yawAngle + ". Duty Cycle: " + dutyPeriod);
%         ann = annotation('textbox', posn, 'string', "XY Angle: " + angle1 + ". Yaw angle: " + yawAngle + ". Yaw Duty Cycle: " + yawDutyPeriod + ". ZY Angle: " + angle2 + ". Pitch Angle: " + pitchAngle + ". Pitch Duty Cycle: " + pitchDutyPeriod, 'EdgeColor', 'none')
%     else
%         delete(fig_servo_point);
%         fig_servo_point = line([S(1, 1) S(1, 1) + sqrt((T(X)-S(X))^2 + (T(Y)-S(Y))^2)*cosd(angle1)], [S(1, 2) S(1, 2) + sqrt((T(X)-S(X))^2 + (T(Y)-S(Y))^2)*sind(angle1)], [S(Z) S(Z) + sqrt((T(Y)-S(Y))^2 + (T(Z)-S(Z))^2)*sind(angle2)],'Color', 'black');
%         %delete(written);
%         %written = text(S(1) + 5, S(2) + 5, "Angle: " + angle + ". Yaw angle: " + yawAngle + ". Duty Cycle: " + dutyPeriod);
%         delete(ann);
%         ann = annotation('textbox', posn, 'string', "XY Angle: " + angle1 + ". Yaw angle: " + yawAngle + ". Yaw Duty Cycle: " + yawDutyPeriod + ". ZY Angle: " + angle2 + ". Pitch Angle: " + pitchAngle + ". Pitch Duty Cycle: " + pitchDutyPeriod, 'EdgeColor', 'none')
%     end

    %ann = annotation('textbox', [0, 0, 0, 0], 'string', "Angle: " + angle + ". Yaw angle: " + yawAngle + ". Duty Cycle: " + dutyPeriod);
    
    pause(0.3)

    i = i + 1;
    if i > length(trajectory)
        i = 2;
    end
    
end