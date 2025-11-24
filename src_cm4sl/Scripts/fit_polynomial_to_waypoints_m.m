function current_lane_info_out = fit_polynomial_to_waypoints_m(local_waypoints, current_lane_info_in)
    
    current_lane_info_out = current_lane_info_in;
    num_degree = 3;
    N = size(local_waypoints, 1);

    x = local_waypoints(:,1);
    b = local_waypoints(:,2);

    A = zeros(N, num_degree+1);
    for i = 0:num_degree
        A(:, i+1) = x.^i;
    end
    
    poly_coeff = pinv(A) * b;
    current_lane_info_out.lane_Coeff = poly_coeff;
    
    % disp('---polyfit Node---')
    % disp(current_lane_info_out.lane_Coeff)
    
end