function desired_ax = longitudinal_speed_pi_controller_driving_m(ego_status, driving_mission_info)
    persistent error_sum
    dt = 0.05;
    
    target_velocity = driving_mission_info.velReference;
    ego_velocity = ego_status.ego_Speed;

    error = target_velocity - ego_velocity;

    if isempty(error_sum)
        error_sum = single(0.0);
    else
        error_sum = error_sum + error * dt;
    end

    Kp = 0.7;
    Ki = 0.005;

    P_term = Kp*error;
    I_term = Ki*error_sum;
    
    desired_ax = (P_term + I_term);

end
