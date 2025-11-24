function delta  = lateral_PP_controller_parking_m(parking_mission_info, ego_status)
    
    L = 2.9;  %wheelbase
    LAD = parking_mission_info.LAD;
    y_LAD = parking_mission_info.y_LAD;

    if LAD == 0
        disp('LAD = 0')
        delta = single(0.0);
        return
    end

    delta = 1.0*atan( 2 * L * y_LAD / (LAD^2) );
    % if parking_mission_info.dir < 0
    %     delta = -delta;
    % end

    % disp('---PP Node---')
    % disp(LAD)
    % disp(y_LAD)
end
    