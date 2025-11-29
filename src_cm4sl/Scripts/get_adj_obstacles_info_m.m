function adj = get_adj_obstacles_info_m(ego_status, obstacles_info, driving_mission_info)
    % obstacles_info   : Nx matrix or struct (each obstacle row = one vehicle)
    %                    columns assumed: [X, Y, Vx, Vy, Ax, Ay, Yaw, Yawrate]

    % disp('--------')
    % disp(obstacles_info);
    % disp('--------')
    % disp('Vehicle_position')
    % disp(vehicle_position);
    % disp('--------')

    mission_state = driving_mission_info.missionState;
    if mission_state >= 3
        adj = zeros(3, 10);
        return;
    end
    
    dist_threshhold = 50.0;
    
    ego_x = ego_status.ego_X;
    ego_y = ego_status.ego_Y;

    X = obstacles_info(:,1);
    Y = obstacles_info(:,2);

    % 거리 계산
    dx = X - ego_x;
    dy = Y - ego_y;
    dist = dx.^2 + dy.^2;

    % threshold 안에 있는 인덱스
    idx = dist < dist_threshhold^2;

    % 필터링된 장애물 정보
    adj = obstacles_info(idx, :);
    
    % disp(idx);
    % if ~isempty(adj)
    %     disp('-----')
    %     disp(adj(1, :));
    % end
end