function current_lane_info_out  = current_lane_identifier_m(MapData_Info, current_lane_info_in, ego_status)
        
    current_lane_info_out = current_lane_info_in;

    w_theta = 0.5;
    x_ego = ego_status.ego_X;
    y_ego = ego_status.ego_Y;
    yaw_ego = ego_status.ego_YAW;

    route1 = MapData_Info.Route1;
    route2 = MapData_Info.Route2;
    route3 = MapData_Info.Route3;
    route4 = MapData_Info.Route_Parking;

    routes = {route1, route2, route3, route4};
    scores = zeros(4,1);
    
    for k = 1:4
        route = routes{k};
        
        % projection distance + lane direction
        [d_proj, theta_lane] = proj_distance(route, x_ego, y_ego);
        
        % heading error
        diff = yaw_ego - theta_lane;
        heading_err = abs(atan2(sin(diff), cos(diff)));
        
        scores(k) = d_proj + w_theta * heading_err;
    end
    
    [~, lane_idx] = min(scores);
    
    waypoints = routes{lane_idx};
    N = size(waypoints, 1);

    padded = zeros(300, 2);
    padded(1:N, :) = waypoints;

    current_lane_info_out.lane_Idx = uint8(lane_idx);
    current_lane_info_out.lane_waypoints = single(padded);
    current_lane_info_out.lane_waypoints_Length = uint16(N);
    %disp(lane_idx)
end


function [d_min, theta_lane] = proj_distance(route, x, y)
    N = size(route,1);
    p = [x; y];
    
    d_min = single(1e9);
    theta_lane = single(0.0);
    
    for i = 1:N-1
        p1 = route(i,:)';
        p2 = route(i+1,:)';
        
        v = p2 - p1;
        u = p - p1;
        
        denom = v' * v;
        if denom < 1e-9
            continue;
        end
        
        % projection factor
        t = (u' * v) / denom;
        t = max(0, min(1, t));   % clamp to segment
        
        % projected point
        p_proj = p1 + t * v;
        
        % distance
        d = norm(p - p_proj);
        
        if d < d_min
            d_min = d;
            theta_lane = single(atan2(v(2), v(1)));  % segment 방향
        end
    end
end