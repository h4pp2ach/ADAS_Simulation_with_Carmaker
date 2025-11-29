function driving_mission_info_out = set_velocity_ref_m(ego_status, driving_mission_info_in)
    
    driving_mission_info_out = driving_mission_info_in;
    target_velocity = driving_mission_info_in.velReference;
    mission_state = driving_mission_info_in.missionState;

    ego_X = ego_status.ego_X;
    ego_Y = ego_status.ego_Y;

    lane_section_idx = get_lane_section_idx(ego_X, ego_Y);
    
    ay_max = 7.2;

    if lane_section_idx == 0
        mission_state = uint8(4);
    end

    if lane_section_idx == 1
        target_velocity = single(sqrt(ay_max/0.008) + 11);
    elseif lane_section_idx == 2
        target_velocity = single(sqrt(ay_max/0.013) + 5);
    elseif lane_section_idx == 3 || lane_section_idx == 4
        target_velocity = single(sqrt(ay_max/0.015));
    elseif lane_section_idx == 5
        target_velocity = single(80/3.6);
    elseif lane_section_idx == 6
        target_velocity = single(sqrt(ay_max/0.039));
    elseif lane_section_idx == 7
        target_velocity = single(sqrt(ay_max/0.02));
    
    % 직선도로
    elseif lane_section_idx == 8
        target_velocity = single(75/3.6);
    end

    
    if mission_state == 2
        target_velocity = single(50/3.6);

    elseif mission_state == 3
        target_velocity = single(45/3.6);

        if lane_section_idx == 9
            target_velocity = single(20/3.6);
        end

    elseif mission_state == 4
        target_velocity = single(0.0);
    end

    driving_mission_info_out.velReference = target_velocity;

end

function lane_section_idx = get_lane_section_idx(ego_x, ego_y)
    % 섹션 1
    section1_x = [-50.6, -50.6, -99.25, -154, -168.38, -169];
    section1_y = [27,11.6, -18.32, -46.77, -39.43,27]; 
    
    % 섹션 2
    section2_x = [-168.38, -154, -151, -166.95, -178.59, -187];
    section2_y = [-39.43, -46.77, -69.31, -85.52, -85.52, -55];
    
    % 섹션 3
    section3_x = [-178.59, -166.95, -104, -36.24, -25.19, -25.19, -178.59];
    section3_y = [-85.52, -85.52, -85.52, -110.78, -115.02, -172, -172];
    
    % 섹션 4
    section4_x = [-36.24, -25.19, 3, 4.57, 0.49, -32];
    section4_y = [-110.78, -115.02, -113, -87.46, -76, -77];
    
    % 섹션 5
    section5_x = [ 0.49, 4.57, 41, 81.24, 81.24, 33];
    section5_y = [-76, -87.46, -85, -78.92, -63.15, -55];
    
    % 섹션 6
    section6_x = [81.24, 81.24, 111.99, 124.34, 124.4];
    section6_y = [-78.92, -63.15, -38.14, -38.14, -78.92];
    
    % 섹션 7
    section7_x = [60, 60, 111.99, 124.34, 124.34];
    section7_y = [27, 11.6, -38.14, -38.14, 27];
    
    % 섹션 8 직선도로
    section8_x = [-50.6, -50.6, 60, 60];
    section8_y = [27, 11.6, 11.6, 27];
    
    % 섹션 9 주차장 입구
    section9_x = [-9.8,5.5,-9.8,5.5];
    section9_y = [11.6, 11.6,-45,-45];

    polygon_list = {
        section1_x,  section1_y;
        section2_x,  section2_y;
        section3_x,  section3_y;
        section4_x,  section4_y;
        section5_x,  section5_y;
        section6_x,  section6_y;
        section7_x,  section7_y;
        section8_x,  section8_y;
        section9_x,  section9_y;
    };
    
    lane_section_idx = uint8(0);

    for i = 1:size(polygon_list,1)
        px = polygon_list{i,1};
        py = polygon_list{i,2};

        inside = inpolygon(ego_x, ego_y, px, py);

        if inside
            lane_section_idx = uint8(i);
            return;
        end
    end

end