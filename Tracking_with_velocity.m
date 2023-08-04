%% Position Tracking Simulation Yaw Plane with Velocity Aiming - Started: 26/06/23
% A similar method to TrackingPosition.m but here the velocity of the
% object is taken into account, along with the velocity of the projectile
% so an impact course can be planned
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

%setting velocity magnitude for the projectile
S_dot = -800;


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
%for i = 1:length(trajectory)
i = 1;
while true

    
    if i == 1

        % if it is the first iteration we cannot know the velocity or acceleration
        % so we set these to zero and simply pull off the first T value;
        lastT = [0 0];
        Tdot = [0 0];
        last_Tdot = [0 0];
        T_accel = [0 0];
        last_T_accel = [0 0];
        T = [trajectory(i,1), trajectory(i,2)];

        angle = 0;
        yawAngle = 0;
        dutyPeriod = 0;

    else        

        % Pull off one trajecotry coordinate and store the previous coord
        lastT = T;
        T = [trajectory(i,1), trajectory(i,2)];
            
        % calculate the velocity of the target storing previous
        last_Tdot = Tdot;
        %Tdot_mag = (lastT(Y) - T(Y))/(lastT(X) - T(X));
        Tdot = [lastT(X) - T(X), lastT(Y) - T(Y)];
        
        %calculate the acceleration of the target storing previous
        last_T_accel = T_accel;
        T_accel = Tdot - last_Tdot;

        % Calculating the Interception:
        % step 1: calculate displacement between current position of T and
        % the servo array
        R = T - S;

        % step 2: Estimate time for projectile to reach T
        t1 = sqrt(R(X)^2 + R(Y)^2)/S_dot;

        % step 3: Estimate future position of T at time t1
        %************ ASSUMES CONSTANT ACCELERATION HERE ******************
        T_future = T + Tdot.*t1 + (T_accel.*t1^2)./2;

        % step 4: Calculate the new displacement from s to the future
        % position
        R_future = T_future - S;

        % step 5: Estimate the additional time required for the projectile
        % to reach the future position
        t2 = sqrt(R_future(X)^2 + R_future(Y)^2)/S_dot;

        % t1 + t2 is the total lead time it will take for the projectile to
        % reach the target
        lead_time = t1 + t2;

        % step 7: Aim at posn given by the equation
        T_aim = T + Tdot.*lead_time + (T_accel.*lead_time^2)./2;


        % Now we have the position to aim at we put it back into the old
        % tracking method:
    
        % Find the euclidean angle between the coordinate and the target
        % angle = asind((T(Y)-S(Y))/sqrt((T(X)-S(X))^2 + (T(Y)-S(Y))^2));
    
        % Convert that angle into a yaw angle for the servo ensuring the
        % quarters of where the target is compared to the servo is taken into
        % account
        % Also important to use the absolute value of A as we only want the
        % angle from begining (1000ms) to the angle
        % This method assumes the servos are placed in the bottom left quadrant
        % of the environment space
%         if T_aim(Y) > S(Y) && T_aim(X) > S(X) %quadrant 1
%             angle = asind((T_aim(Y)-S(Y))/sqrt((T_aim(X)-S(X))^2 + (T_aim(Y)-S(Y))^2));
%             yawAngle = abs(A) + angle;
%             quadrant = 1;
%         elseif T_aim(Y) < S(Y) && T_aim(X) > S(X)   %quadrant 3
%             angle = asind(-(T_aim(Y)-S(Y))/sqrt((T_aim(X)-S(X))^2 + (T_aim(Y)-S(Y))^2));
%             yawAngle = abs(A) - angle;
%             quadrant = 3;
%         elseif T_aim(Y) > S(Y) && T_aim(X) < S(X)   %quadrant 2
%             angle = 180 - asind((T_aim(Y)-S(Y))/sqrt((T_aim(X)-S(X))^2 + (T_aim(Y)-S(Y))^2));
%             yawAngle = abs(A) + angle;
%             quadrant = 2;
%         elseif T_aim(Y) > S(Y) && v(X) == S(X)
%             angle = asind((T_aim(Y)-S(Y))/sqrt((T_aim(X)-S(X))^2 + (T_aim(Y)-S(Y))^2));
%             yawAngle = abs(A) + 90;
%         elseif T_aim(Y) == S(Y) && T_aim(X) > S(X)
%             angle = asind((T_aim(Y)-S(Y))/sqrt((T_aim(X)-S(X))^2 + (T_aim(Y)-S(Y))^2));
%             yawAngle = abs(A);
%         end

        angle = rad2deg(atan2(T_aim(Y) - S(Y), T_aim(X) - S(X)));
        yawAngle = abs(A) + angle;

        %  Attempting to implement a atan2 method where the servo can be placed
        %  anywhere
        %     angle = rad2deg(atan2(T(Y) - S(Y), T(X) - S(X)));
        %     if angle < 0
        %         angle = 360 - abs(angle);
        %     end

         % Check the angle is within the boundary of the servos and then convert
        % to duty cycle value
        if yawAngle <= 120 && yawAngle >= 0
            %convert to duty cycle
            dutyPeriod = convert_to_duty_cycle(yawAngle);
    
        elseif yawAngle < 0
            % send 0 to duty cycle to be closest to it
            yawAngle = 0;
            dutyPeriod = convert_to_duty_cycle(yawAngle);
    
        elseif yawAngle > 120
            %send 120 to duty cycle to be closest
            yawAngle = 120;
            dutyPeriod = convert_to_duty_cycle(yawAngle);
        end
    

    

    end


    %Display what has been calculated on the figure
    posn = [0.9 0.5, 0.1, 0.2];

    if quadrant == 3
        angle = 360 - angle;
    end

    if i == 1
        target = plot(T(X), T(Y), '.r');
        target_aim = plot(T(X), T(Y), '*r');
        fig_servo_point = line([S(1, 1) S(1, 1) + sqrt((T(X)-S(X))^2 + (T(Y)-S(Y))^2)*cosd(angle)], [S(1, 2) S(1, 2) + sqrt((T(X)-S(X))^2 + (T(Y)-S(Y))^2)*sind(angle)], 'Color', 'black');
        %written = text(S(1) + 5, S(2) + 5, "Angle: " + angle + ". Yaw angle: " + yawAngle + ". Duty Cycle: " + dutyPeriod);
        ann = annotation('textbox', posn, 'string', "Angle: " + angle + ". Yaw angle: " + yawAngle + ". Duty Cycle: " + dutyPeriod, 'EdgeColor', 'none')
    else
        delete(target);
        target = plot(T(X), T(Y), '.r');
        delete(target_aim);
        target_aim = plot(T_aim(X), T_aim(Y), '*r');
        delete(fig_servo_point);
        fig_servo_point = line([S(1, 1) S(1, 1) + sqrt((T_aim(X)-S(X))^2 + (T_aim(Y)-S(Y))^2)*cosd(angle)], [S(1, 2) S(1, 2) + sqrt((T_aim(X)-S(X))^2 + (T_aim(Y)-S(Y))^2)*sind(angle)], 'Color', 'black');
        %delete(written);
        %written = text(S(1) + 5, S(2) + 5, "Angle: " + angle + ". Yaw angle: " + yawAngle + ". Duty Cycle: " + dutyPeriod);
        delete(ann);
        ann = annotation('textbox', posn, 'string', "Angle: " + angle + ". Yaw angle: " + yawAngle + ". Duty Cycle: " + dutyPeriod, 'EdgeColor', 'none')
    end
            %ann = annotation('textbox', [0, 0, 0, 0], 'string', "Angle: " + angle + ". Yaw angle: " + yawAngle + ". Duty Cycle: " + dutyPeriod);
        
    pause(0.3)

    i = i + 1;
    if i == length(trajectory)
        i = 1;
    end
    
end