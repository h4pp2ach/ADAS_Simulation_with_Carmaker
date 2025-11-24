function desired_ax = longitudinal_speed_pi_controller_parking_m(ego_status, parking_mission_info)
    persistent error_sum
    dt = 0.05;
    
    target_velocity = 4/3.6;
    
    isclear = parking_mission_info.parking_clear;
    isPathFound = parking_mission_info.isPathFound;

    % disp('---------------')
    % disp(isPathFound)
    % disp(ego_status.ego_X)
    % disp(ego_status.ego_Y)

    if isclear == 1 || isPathFound == 0
        target_velocity = -5/3.6;
        error_sum = single(0.0);
    end
    ego_velocity = ego_status.ego_Speed;

    error = target_velocity - ego_velocity;

    if isempty(error_sum)
        error_sum = single(0.0);
    else
        error_sum = error_sum + error * dt;
    end

    Kp = 0.8;
    Ki = 0.005;

    P_term = Kp*error;
    I_term = Ki*error_sum;
    
    desired_ax = (P_term + I_term);

end
