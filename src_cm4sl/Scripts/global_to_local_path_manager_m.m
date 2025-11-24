function parking_mission_info_out  = global_to_local_path_manager_m(mission_state, parking_mission_info_in, waypoints, ego_status)
    
    parking_mission_info_out = parking_mission_info_in;
    
    if mission_state ~= 4
        return;
    end

    goalX = parking_mission_info_in.parkingDestinationPos_X;
    goalY = parking_mission_info_in.parkingDestinationPos_Y;
    ego_X = ego_status.ego_X;
    ego_Y = ego_status.ego_Y;

    dx_goal = ego_X - goalX;
    dy_goal = ego_Y - goalY;
    dist_goal = sqrt(dx_goal^2 + dy_goal^2);

    if dist_goal < 1.0
        parking_mission_info_out.parking_clear = uint8(1);
        parking_mission_info_out.LAD = single(0.01);
        parking_mission_info_out.y_LAD = single(0.0);
        disp("Parking!")
        return;
    end
    
    if size(waypoints,1) == 1 && (max(abs(waypoints(:))) == 0)
        parking_mission_info_out.isPathFound = uint8(0);
        parking_mission_info_out.LAD = single(0.0);
        parking_mission_info_out.y_LAD = single(0.0);
        return;
    else
        parking_mission_info_out.isPathFound = uint8(1);
    end
    num_waypoint = 5;

    vehicle_position = [ego_X, ego_Y];
    head = ego_status.ego_YAW;

    dx = waypoints(:,1) - vehicle_position(1);
    dy = waypoints(:,2) - vehicle_position(2);
    dist = dx.^2 + dy.^2;

    % isNotForward = (dx*cos(head) + dy*sin(head)) < 0;
    % dist(isNotForward) = inf;

    [minDist, idx_start] = min(dist);
    
    % disp('---minDist---')
    % disp(minDist)
    % if minDist > 3.0
    %     parking_mission_info_out.LAD = single(0.0);
    %     parking_mission_info_out.y_LAD = single(0.0);
    %     parking_mission_info_out.dir = int8(-1);
    %    return;
    % end

    idx_end = idx_start + num_waypoint - 1;
    idx_end = min(idx_end, size(waypoints,1));
    selected = waypoints(idx_start:idx_end, :);

    L = size(selected,1);
    if L < num_waypoint
        selected = [selected; repmat(selected(end,:), num_waypoint-L, 1)];
    end

    R_inv = [ cos(head),  sin(head);
             -sin(head),  cos(head) ];
    
    selected_xy = selected(:,1:2);
    delta = bsxfun(@minus, selected_xy, vehicle_position);
    local_waypoints_ = (R_inv * delta')';

    poly_coeff  = fit_polynomial_to_waypoints_mfile(local_waypoints_);
    [y_LAD, LAD] = compute_lateral(poly_coeff);
    dir = int8(waypoints(idx_start,4));

    % disp(size(waypoints(:,1)))
    % disp(idx_start)
    % disp(waypoints(idx_start, :))
    
    parking_mission_info_out.LAD = LAD;
    parking_mission_info_out.y_LAD = y_LAD;
    parking_mission_info_out.dir = dir;

end

function  poly_coeff  = fit_polynomial_to_waypoints_mfile(local_waypoints)

    num_degree = 3;
    N = size(local_waypoints, 1);

    x = local_waypoints(:,1);
    b = local_waypoints(:,2);

    A = zeros(N, num_degree+1);
    for i = 0:num_degree
        A(:, i+1) = x.^i;
    end
    
    poly_coeff = pinv(A) * b;
    
end

function [y_LAD, LAD] = compute_lateral(poly_coeff)

    x_lookahead = 2.9 + 0.5;
    degree = length(poly_coeff) - 1;

    y_ref = single(0.0);
    for i = 0:degree
        y_ref = y_ref + poly_coeff(i+1) * x_lookahead^i;
    end

    y_LAD = single(y_ref);
    LAD = single(sqrt(x_lookahead^2 + y_LAD^2));
    
end
