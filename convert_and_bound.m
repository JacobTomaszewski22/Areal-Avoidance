%% Convert And Bound function
% A function decide if a passed in angle is outside the servo ranges and
% to convert this to the duty cycle in ms which is returned
% Started: 27/06/23

function [dutyPeriod, angle] = convert_and_bound(angle)
    % Check the angle is within the boundary of the servos and then convert
    % to duty cycle value
    if angle <= 120 && angle >= 0
        %convert to duty cycle
        dutyPeriod = convert_to_duty_cycle(angle);

    elseif angle < 0
        % send 0 to duty cycle to be closest to it
        angle = 0;
        dutyPeriod = convert_to_duty_cycle(angle);

    elseif angle > 120
        %send 120 to duty cycle to be closest
        angle = 120;
        dutyPeriod = convert_to_duty_cycle(angle);
    end

end