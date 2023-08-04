%% Position Tracking Simulation 3D with Velocity and Drag - Started: 04/07/23
% A similar method to Tracking_with_Velocity.m but we use 3D kinematics to
% moddel the velocity of the projectile and the target and implement
% deceleration due to drag
%% Initialise Begining Parameters
% Start by instatiating the positions, rotations and trajectory
clf
clear

trajectory = generate_trajectory(5);
% set the servo position
S = [-180, -180, -10];

%Constants for ease of use
X = 1;
Y = 2;
Z = 3;

%setting velocity magnitude for the projectile
%need negative velocity else the target is behind for some reason
S_dot = -1000;

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

fig = uifigure;
button = uibutton(fig,"state", ...
    "Text","Fire", ...
    "IconAlignment","bottom", ...
    "Position",[100 100 50 50]);


%% Simulation
% Step through the trajectory
%for i = 1:length(trajectory)
i = 1;
while true

    
    if i == 1

        % if it is the first iteration we cannot know the velocity or acceleration
        % so we set these to zero and simply pull off the first T value;
        lastT = [0 0 0];
        Tdot = [0 0 0];
        last_Tdot = [0 0 0];
        T_accel = [0 0 0];
        last_T_accel = [0 0 0];
        T = [trajectory(i,1), trajectory(i,2), trajectory(i,3)];

        %we calcualte the distances from the single T value
        distance = sqrt((T(X)-S(X))^2 + (T(Y)-S(Y))^2 + (T(Z)-S(Z))^2);
        distance_aim = distance;

        % and use the first target location as the target, plugging them in
        % to the standard angle calculations
        angle1 = rad2deg(atan2(T(Y) - S(Y), T(X) - S(X)));
        yawAngle = abs(Yaw_Min) + angle1;

        angle2 = rad2deg(atan2(sqrt((T(X)-S(X))^2 + (T(Y)-S(Y))^2), (T(Z)-S(Z))));
        pitchAngle = abs(Pitch_Min) + (90 - angle2);
    
        %we put these angles through the convert and bound function
        [yawDutyPeriod, yawAngle] = convert_and_bound(yawAngle);
        [pitchDutyPeriod, pitchAngle] = convert_and_bound(pitchAngle);
        
        

    else        

        % Pull off one trajecotry coordinate and store the previous coord
        lastT = T;
        T = [trajectory(i,1), trajectory(i,2), trajectory(i,3)];
            
        % calculate the velocity of the target storing previous
        last_Tdot = Tdot;
        %Tdot_mag = (lastT(Y) - T(Y))/(lastT(X) - T(X));
        Tdot = [lastT(X) - T(X), lastT(Y) - T(Y), lastT(Z) - T(Z)];
        
        %calculate the acceleration of the target, storing the previous
        last_T_accel = T_accel;
        T_accel = Tdot - last_Tdot;

        % Calculating the Interception:
        % step 1: calculate displacement between current position of T and
        % the servo array
        R = T - S;

        % step 2: Estimate time for projectile to reach T
        t1 = sqrt(R(X)^2 + R(Y)^2 + R(Z)^2)/S_dot;

        % step 3: Estimate future position of T at time t1
        T_future = T + Tdot.*t1 + (T_accel.*t1^2)./2;

        % step 4: Calculate the new displacement from s to the future
        % position
        R_future = T_future - S;

        % step 5: Estimate the additional time required for the projectile
        % to reach the future position
        t2 = sqrt(R_future(X)^2 + R_future(Y)^2 + R_future(Z)^2)/S_dot;

        % t1 + t2 is the total lead time it will take for the projectile to
        % reach the target
        lead_time = t1 + t2;

        % step 7: Aim at posn given by the equation
        T_aim = T + Tdot.*lead_time + (T_accel.*lead_time^2)./2;

        %This equation is using meters not milimeters change this in
        %implementation!!!!!!
        %projectile_drag = 0.5 * 0.001436236 * 1.293 * (2 * vpa(p, 30) * 0.0195^2) * S_dot^2;


        % Now we have the position to aim at we put it back into the old
        % tracking method:
    
        % Find the euclidean angle between the coordinate and the target
        distance = sqrt((T(X)-S(X))^2 + (T(Y)-S(Y))^2 + (T(Z)-S(Z))^2);
        distance_aim = sqrt((T_aim(X)-S(X))^2 + (T_aim(Y)-S(Y))^2 + (T_aim(Z)-S(Z))^2);
        angle1_prev = angle1;
        angle1 = rad2deg(atan2(T_aim(Y) - S(Y), T_aim(X) - S(X)));

        % Convert that angle into a yaw angle for the servo
        % Also important to use the absolute value of A as we only want the
        % angle from begining (1000ms) to the angle
        yawAngle = abs(Yaw_Min) + angle1;

        % Calculate the pitch angle
    
        angle2 = rad2deg(atan2(sqrt((T_aim(X)-S(X))^2 + (T_aim(Y)-S(Y))^2), (T_aim(Z)-S(Z))));
        %angle2 = acosd((T(Z)-S(Z))/distance);
        pitchAngle = abs(Pitch_Min) + (90 - angle2);

         % Check the angle is within the boundary of the servos and then convert
        % to duty cycle value
        [yawDutyPeriod, yawAngle] = convert_and_bound(yawAngle);
        [pitchDutyPeriod, pitchAngle] = convert_and_bound(pitchAngle);


    end

    %if the projectile is fired
    if button.value == true
        
    end


    %Display what has been calculated on the figure
    posn = [0.9 0.5 0.1 0.2];

    if i == 1
        target = plot3(T(X), T(Y), T(Z), '.r');
        target_aim = plot3(T(X), T(Y), T(Z), '*r');
        fig_servo_point = plot3([S(X) S(X) + distance*cosd(angle1)*sind(angle2)], [S(Y) S(Y) + distance*sind(angle1)*sind(angle2)], [S(Z) S(Z) + distance*cosd(angle2)], 'Color', 'black');
        %written = text(S(1) + 5, S(2) + 5, "Angle: " + angle + ". Yaw angle: " + yawAngle + ". Duty Cycle: " + dutyPeriod);
        ann = annotation('textbox', posn, 'string', "XY Angle: " + angle1 + ". Yaw angle: " + yawAngle + ". Yaw Duty Cycle: " + yawDutyPeriod + ". ZY Angle: " + angle2 + ". Pitch Angle: " + pitchAngle + ". Pitch Duty Cycle: " + pitchDutyPeriod, 'EdgeColor', 'none')
        rotate(square,[0 0 1],angle1-90, S);
    else
        delete(target);
        target = plot3(T(X), T(Y), T(Z), '.r');
        delete(target_aim);
        target_aim = plot3(T_aim(X), T_aim(Y), T_aim(Z), '*r');
        delete(fig_servo_point);
        fig_servo_point = plot3([S(X) S(X) + distance_aim*cosd(angle1)*sind(angle2)], [S(Y) S(Y) + distance_aim*sind(angle1)*sind(angle2)], [S(Z) S(Z) + distance_aim*cosd(angle2)], 'Color', 'black');
        %delete(written);
        %written = text(S(1) + 5, S(2) + 5, "Angle: " + angle + ". Yaw angle: " + yawAngle + ". Duty Cycle: " + dutyPeriod);
        delete(ann);
        ann = annotation('textbox', posn, 'string', "XY Angle: " + angle1 + ". Yaw angle: " + yawAngle + ". Yaw Duty Cycle: " + yawDutyPeriod + ". ZY Angle: " + angle2 + ". Pitch Angle: " + pitchAngle + ". Pitch Duty Cycle: " + pitchDutyPeriod, 'EdgeColor', 'none')
        rotate(square,[0 0 1], angle1-angle1_prev, S);
%         delete(fig_servo_point);
%         fig_servo_point = line([S(1, 1) S(1, 1) + sqrt((T_aim(X)-S(X))^2 + (T_aim(Y)-S(Y))^2)*cosd(angle)], [S(1, 2) S(1, 2) + sqrt((T_aim(X)-S(X))^2 + (T_aim(Y)-S(Y))^2)*sind(angle)], 'Color', 'black');
%         %delete(written);
%         %written = text(S(1) + 5, S(2) + 5, "Angle: " + angle + ". Yaw angle: " + yawAngle + ". Duty Cycle: " + dutyPeriod);
%         delete(ann);
%         ann = annotation('textbox', posn, 'string', "Angle: " + angle + ". Yaw angle: " + yawAngle + ". Duty Cycle: " + dutyPeriod, 'EdgeColor', 'none')
    end
            %ann = annotation('textbox', [0, 0, 0, 0], 'string', "Angle: " + angle + ". Yaw angle: " + yawAngle + ". Duty Cycle: " + dutyPeriod);
        
    pause(1)

    i = i + 1;
    if i == length(trajectory)
        i = 1;
    end
    
end