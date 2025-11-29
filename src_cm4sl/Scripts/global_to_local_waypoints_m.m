function local_waypoints_ = global_to_local_waypoints_m(waypointList, current_lane_info, ego_status)

    num_waypoint = 30;
    vehicle_position = [ego_status.ego_X, ego_status.ego_Y];
    head = ego_status.ego_YAW;

    sub_waypointList = current_lane_info.lane_waypoints(1:current_lane_info.lane_waypoints_Length, :);

    if isempty(waypointList) || all(waypointList(:) == 0)
        waypoints = sub_waypointList;
    else
        waypoints = single(waypointList);
    end

    dx = waypoints(:,1) - vehicle_position(1);
    dy = waypoints(:,2) - vehicle_position(2);
    dist = dx.^2 + dy.^2;

    isNotForward = (dx*cos(head) + dy*sin(head)) < 0;
    dist(isNotForward) = inf;

    [~, idx_start] = min(dist);

    idx_end = idx_start + num_waypoint - 1;
    idx_end = min(idx_end, size(waypoints,1));
    selected = waypoints(idx_start:idx_end, :);

    L = size(selected,1);
    if L < num_waypoint
        selected = [selected; repmat(selected(end,:), num_waypoint-L, 1)];
    end

    R_inv = [ cos(head),  sin(head);
             -sin(head),  cos(head) ];

    delta = bsxfun(@minus, selected, vehicle_position);
    local_waypoints_ = (R_inv * delta')';

end
