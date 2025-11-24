function delta  = lateral_PP_controller_driving_m(current_lane_info)
    
    L = 2.9;  %wheelbase
    LAD = current_lane_info.lookAheadDistance;
    y_LAD = current_lane_info.y_lookAheadDistance;
    heading_error = current_lane_info.lane_Coeff(2);

    if LAD == 0
        disp('LAD = 0')
        delta = single(0.0);
        return
    end

    delta = 1.07*atan( 2 * L * y_LAD / (LAD^2) );

    % disp('---PP Node---')
    % disp(LAD)
    % disp(y_LAD)
    
end
