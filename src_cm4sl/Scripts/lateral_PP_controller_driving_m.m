function delta  = lateral_PP_controller_driving_m(current_lane_info, ego_status)
    
    L = 2.9;  %wheelbase
    LAD = current_lane_info.lookAheadDistance;
    y_LAD = current_lane_info.y_lookAheadDistance;

    if LAD == 0
        disp('LAD = 0')
        delta = single(0.0);
        return
    end

    delta = 1.5*atan( 2 * L * y_LAD / (LAD^2) );

    % disp('---PP Node---')
    % disp(LAD)
    % disp(y_LAD)
    
    k_h = 0.5;
    k_e = 0.25;

    heading_error = atan(current_lane_info.lane_Coeff(2));
    cross_track_error = current_lane_info.y_lookAheadDistance;
    V = ego_status.ego_Speed;
    
    heading_err_term     = k_h * heading_error;
    cross_track_err_term = atan(k_e * cross_track_error / V);

    delta_s = (heading_err_term + cross_track_err_term);
    
    if V >= 65
        delta = delta_s;
    end

end
