function current_lane_info_out = compute_lateral_info_m(current_lane_info_in, ego_status)
    
    current_lane_info_out = current_lane_info_in;
    poly_coeff = current_lane_info_in.lane_Coeff;

    lookahead_time = 1.6;
    velocity = ego_status.ego_Speed;
    x_lookahead = velocity * lookahead_time;
    degree = length(poly_coeff) - 1;

    y_ref = single(0.0);
    for i = 0:degree
        y_ref = y_ref + poly_coeff(i+1) * x_lookahead^i;
    end

    y_LAD = y_ref;
    LAD = sqrt(x_lookahead^2 + y_LAD^2);
    % disp('---lateral Node---')
    % disp(LAD)
    % disp(y_LAD)
    
    current_lane_info_out.lookAheadDistance = LAD;
    current_lane_info_out.y_lookAheadDistance = y_LAD;
    
end
