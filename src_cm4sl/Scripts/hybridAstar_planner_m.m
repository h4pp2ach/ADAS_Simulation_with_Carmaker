function path = hybridAstar_planner_m(parking_mission_info, obstacles, param)
    
    persistent initialized
    persistent cached_path
    persistent cached_len

    MAX_PATH = 20000;
    if isempty(initialized)
        initialized = false;
        cached_path = zeros(MAX_PATH, 4, 'single');
        cached_len  = uint16(0);
    end

    if initialized
        path = cached_path(1:cached_len, :);
        return;
    end
    x_min = parking_mission_info.parkingMapBoundary_X_Min;
    x_max = parking_mission_info.parkingMapBoundary_X_Max;
    y_min = parking_mission_info.parkingMapBoundary_Y_Min;
    y_max = parking_mission_info.parkingMapBoundary_Y_Max;
    
    space = [x_min, x_max, y_min, y_max];

    start_X   = parking_mission_info.parkingStartPos_X;
    start_Y   = parking_mission_info.parkingStartPos_Y;
    start_YAW = 0;

    obstacles = obstacles(:,1:3);
    
    goal_X   = parking_mission_info.parkingDestinationPos_X;
    goal_Y   = parking_mission_info.parkingDestinationPos_Y;
    goal_YAW = parking_mission_info.parkingDestinationYaw;

    goal = [goal_X, goal_Y, goal_YAW];

    p = param;

    nx = single(floor((x_max - x_min) / p.xyResolution) + 1);
    ny = single(floor((y_max - y_min) / p.xyResolution) + 1);
    nyaw = single(floor(2*pi / p.yawResolution) + 1);
    
    costList    = inf(nx, ny, nyaw);
    nodeIdList  = zeros(nx, ny, nyaw);
    closedList  = false(nx, ny, nyaw);
    
    MAX_NODES   = 20000;
    MAX_OPENSET = 20000;
    MAX_CHILDREN = 6;

    nodeTemplate = struct('x',single(0), 'y',single(0), 'yaw',single(0),...
                          'g',single(0), 'h',single(0), 'f',single(0),...
                          'dir',single(1), 'parent',int32(0),...
                          'ix',single(0), 'iy',single(0), 'iyaw',single(0));

    nodes = repmat(nodeTemplate, MAX_NODES, 1);
    nodeCount = 1;

    [ix,iy,iyaw] = world2grid(start_X, start_Y, start_YAW, ...
                              x_min, y_min, p.xyResolution, p.yawResolution);

    nodes(1).x    = start_X;
    nodes(1).y    = start_Y;
    nodes(1).yaw  = wrapToPi_local(start_YAW);
    nodes(1).g    = single(0.0);
    nodes(1).h    = heuristic_cost(start_X, start_Y, start_YAW, goal, p);
    nodes(1).f    = nodes(1).g + nodes(1).h;
    nodes(1).dir  = single(1.0);
    nodes(1).parent = int32(0);
    nodes(1).ix   = ix;
    nodes(1).iy   = iy;
    nodes(1).iyaw = iyaw;

    costList(ix,iy,iyaw)   = 0;
    nodeIdList(ix,iy,iyaw) = 1;

    openSet     = zeros(MAX_OPENSET, 1, 'int32');
    openCount   = 1;
    openSet(1)  = 1;

    success = false;
    goalId  = int32(-1);

    iter = 0;
    while (openCount > 0) && (iter < p.maxIter)
        iter = iter + 1;

        bestIdx = 1;
        bestId  = openSet(1);
        bestF   = nodes(bestId).f;
        
        for i = 2:openCount
            nid = openSet(i);
            if nodes(nid).f < bestF
                bestF = nodes(nid).f;
                bestIdx = i;
                bestId  = nid;
            end
        end
        
        curId = bestId;
        openSet(bestIdx) = openSet(openCount);  % 마지막 원소와 교체
        openCount = openCount - 1;

        curNode = nodes(curId);
        if closedList(curNode.ix, curNode.iy, curNode.iyaw)
            continue;
        end
        closedList(curNode.ix, curNode.iy, curNode.iyaw) = true;

        if isGoalReached(curNode, goal, p)
            success = true;
            goalId  = int32(curId);
            break;
        end

        children = CreateNode(curNode, p);

        for k = 1:numel(children)
            child = children(k);

            if child.x < x_min || child.x > x_max || ...
               child.y < y_min || child.y > y_max
                continue;
            end

            if checkCollisionMotion(curNode, child, obstacles, space, p)
                continue;
            end

            [ix_c,iy_c,iyaw_c] = world2grid(child.x, child.y, child.yaw, ...
                                            x_min, y_min, ...
                                            p.xyResolution, p.yawResolution);

            if closedList(ix_c,iy_c,iyaw_c)
                continue;
            end

            g_new = curNode.g + child.cost;

            if g_new < costList(ix_c,iy_c,iyaw_c)
                childNode = nodeTemplate;
                childNode.x    = child.x;
                childNode.y    = child.y;
                childNode.yaw  = wrapToPi_local(child.yaw);
                childNode.g    = g_new;
                childNode.h    = heuristic_cost(childNode.x, childNode.y, childNode.yaw, goal, p);
                childNode.f    = childNode.g + childNode.h;
                childNode.dir  = child.dir;
                childNode.parent = curId;
                childNode.ix   = ix_c;
                childNode.iy   = iy_c;
                childNode.iyaw = iyaw_c;

                if nodeIdList(ix_c,iy_c,iyaw_c) == 0
                    nodeCount = nodeCount + 1;
                    if nodeCount > MAX_NODES
                        continue;
                    end

                    nodes(nodeCount) = childNode;
                    newId = nodeCount;
                else
                    newId = nodeIdList(ix_c,iy_c,iyaw_c);
                    nodes(newId) = childNode;
                end

                costList(ix_c,iy_c,iyaw_c)   = g_new;
                nodeIdList(ix_c,iy_c,iyaw_c) = newId;
                
                if openCount < MAX_OPENSET
                    openCount = openCount + 1;
                    openSet(openCount) = newId;
                else
                end

            end
        end
    end

    if ~success
        path = zeros(1,4,'single');
        if ~initialized
            visualize_hybridAstar_internal(path, obstacles, [start_X start_Y], [goal_X goal_Y], p);
            initialized = true;
        end
        disp('no path')
        return;
    end

    path = backtrackPath(nodes, goalId, MAX_NODES);
    disp('find path')
    
    valid = (path(:,1) ~= 0) | (path(:,2) ~= 0) | (path(:,3) ~= 0);
    path = path(valid, :);

    k = size(path,1);
    cached_path(1:k, :) = path;
    cached_len = uint16(k);
    initialized = true;

    visualize_hybridAstar_internal(path, obstacles, [start_X start_Y], [goal_X goal_Y], p);
end



function ang = wrapToPi_local(ang)
    ang = single(mod(ang + pi, 2*pi) - pi);
end

% 양자화 때리는거
function [ix, iy, iyaw] = world2grid(x, y, yaw, x_min, y_min, res_xy, res_yaw)
    ix = floor((x - x_min) / res_xy) + 1;
    iy = floor((y - y_min) / res_xy) + 1;
    yaw = wrapToPi_local(yaw);
    iyaw = floor((yaw + pi) / res_yaw) + 1;
end

% 도착했는지 검사
function flag = isGoalReached(node, goal, p)
    dx = goal(1) - node.x;
    dy = goal(2) - node.y;
    dist = sqrt(dx^2 + dy^2);

    dyaw = abs(wrapToPi_local(goal(3) - node.yaw));
    flag = (dist < p.goalPosTol) && (dyaw < p.goalYawTol);
end

% 자식낳기
function children = CreateNode(node, p)

    R = p.R_min;
    kappa_max = 1.0 / R;
    ds = p.stepSize;

    steeringSet = p.steerSet * kappa_max;
    dirSet = [1, -1];

    MAX_CHILDREN = 6;
    children = repmat(struct('x',single(0), 'y',single(0), 'yaw',single(0), ...
                             'dir',single(0), 'cost',single(0)), single(MAX_CHILDREN), single(1));
    childCount = 0;

    for i = 1:length(steeringSet)

        kappa = steeringSet(i);
        for d = 1:length(dirSet)
            dir = dirSet(d);
            s   = dir * ds;

            [xn, yn, yawn] = BicycleKinematic(node.x, node.y, node.yaw, kappa, s);

            % cost: 경로 길이 + 후진 패널티
            cost = abs(ds);
            if dir < 0
                cost = cost * p.reversePenalty;
            end

            childCount = childCount + 1;
            children(childCount).x    = xn;
            children(childCount).y    = yn;
            children(childCount).yaw  = yawn;
            children(childCount).dir  = single(dir);
            children(childCount).cost = cost;
        end
    end
end

% Bicycle 모델
function [x_new, y_new, yaw_new] = BicycleKinematic(x, y, yaw, kappa, s)
    if abs(kappa) < 1e-6

        x_new   = x + s * cos(yaw);
        y_new   = y + s * sin(yaw);
        yaw_new = yaw;
    else

        R = 1.0 / kappa;
        yaw_new = yaw + kappa * s;
        x_new = x + R * (sin(yaw_new) - sin(yaw));
        y_new = y - R * (cos(yaw_new) - cos(yaw));
    end
    yaw_new = wrapToPi_local(yaw_new);
end

% 중간 샘플 몇 개 찍어서 충돌 체크 할거임
function flag = checkCollisionMotion(n1, n2, obstacles, space, p)
    
    nSamples = 5;
    flag = false;

    kappa_est = curvatureFromNodes(n1, n2, p);

    for i = 0:nSamples
        t  = i / nSamples;
        s  = t * p.stepSize * sign(n2.dir); % 방향 고려
        [x_s, y_s, ~] = BicycleKinematic(n1.x, n1.y, n1.yaw, ...
                                           kappa_est, s);

        if checkCollisionPoint(x_s, y_s, obstacles, space, p)
            flag = true;
            return;
        end
    end
end

function flag = checkCollisionPoint(x, y, obstacles, space, p)
    % World boundary
    if x < space(1) || x > space(2) || ...
       y < space(3) || y > space(4)
        flag = true;
        return;
    end

    % 장애물: 뒷대가리 기준 + heading → 차량 길이 따라 원 2개 배치
    L = p.carLengthObs;
    
    d1 = L * 0.3;    % 뒤쪽 쪽
    d2 = L * 0.8;    % 앞쪽 쪽

    effR2 = (p.obsRadius + p.vehicleRadius)^2;

    for i = 1:size(obstacles,1)
        xr = obstacles(i,1);
        yr = obstacles(i,2);
        hdg = obstacles(i,3);

        % 두 개 서클의 중심
        x1 = xr + d1*cos(hdg);
        y1 = yr + d1*sin(hdg);
        x2 = xr + d2*cos(hdg);
        y2 = yr + d2*sin(hdg);

        dx1 = x - x1; dy1 = y - y1;
        dx2 = x - x2; dy2 = y - y2;

        if (dx1*dx1 + dy1*dy1 < effR2) || (dx2*dx2 + dy2*dy2 < effR2)
            flag = true;
            return;
        end
    end

    flag = false;
end

function kappa = curvatureFromNodes(n1, n2, p)
    % n1→n2 κ 추정 (대충 yaw 변화로 계산)
    ds = p.stepSize;
    dyaw = wrapToPi_local(n2.yaw - n1.yaw);
    if abs(ds) < 1e-6
        kappa = single(0);
    else
        kappa = single(dyaw / (ds * sign(n2.dir)));
    end
end

function h = heuristic_cost(x, y, yaw, goal, p)
    dx = goal(1) - x;
    dy = goal(2) - y;
    dist = sqrt(dx^2 + dy^2);

    dyaw = abs(wrapToPi_local(goal(3) - yaw));
    h = dist + p.yawWeight * dyaw;
end

function path = backtrackPath(nodes, goalId, MAX_PATH)
    path = zeros(MAX_PATH, 4, 'single');

    idx = goalId;
    k = 0;
    
    % 뒤에서부터 채우고 뒤집음
    while (idx ~= 0) && (k < MAX_PATH)
        n = nodes(idx);
        k = k + 1;
        path(k, :) = [n.x, n.y, n.yaw, n.dir];
        idx = n.parent;
    end

    if k > 1
        % start→goal 순서로 뒤집기
        path(1:k, :) = flipud(path(1:k, :));
    end
end

% 시각화 (한번만 함)
function visualize_hybridAstar_internal(path, obstacles, startXY, goalXY, p)

    figure; hold on; axis equal; grid on;
    xlabel('X'); ylabel('Y');
    title('Hybrid A* Path');

    for i = 1:size(obstacles,1)
        drawObstacleCarDual(obstacles(i,1), obstacles(i,2), obstacles(i,3), p);
    end

    % Start & Goal
    plot(startXY(1), startXY(2), 'go', 'MarkerFaceColor','g','MarkerSize',10);
    plot(goalXY(1),  goalXY(2),  'ro', 'MarkerFaceColor','r','MarkerSize',10);

    % Path
    xs = path(:,1); ys = path(:,2); dirs = path(:,4);
    valid = xs ~= 0;
    xs = xs(valid); ys = ys(valid); dirs = dirs(valid);

    for i = 1:length(xs)-1
        if dirs(i) > 0
            plot(xs(i:i+1), ys(i:i+1), 'b-', 'LineWidth', 2);
        else
            plot(xs(i:i+1), ys(i:i+1), 'r-', 'LineWidth', 2);
        end
    end

    hold off;
end


function h = drawCircle(x, y, r, color)
    th = linspace(0, 2*pi, 40);
    xc = x + r*cos(th);
    yc = y + r*sin(th);
    h = fill(xc, yc, color, 'FaceAlpha', 0.3, 'EdgeColor', color);
end

function drawObstacleCarDual(xr, yr, hdg_deg, p)
    L   = p.carLengthObs;
    hdg = hdg_deg;

    d1 = L * 0.3;
    d2 = L * 0.8;

    x1 = xr + d1*cos(hdg);
    y1 = yr + d1*sin(hdg);
    x2 = xr + d2*cos(hdg);
    y2 = yr + d2*sin(hdg);

    drawCircle(x1, y1, p.obsRadius, [0.7 0.2 0.2]);
    drawCircle(x2, y2, p.obsRadius, [0.7 0.2 0.2]);
end