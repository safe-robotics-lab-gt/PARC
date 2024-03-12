function lookup_points = create_turtle_lookup(xytkk_range, xy_sample_n)
% Expert Plan for the Dubins car Planning model
%
% Assumes X_goal centered at 0
% Assumes theta_range = [-pi, pi]
% Assumes x_0_m, y_0_m of lookup_points lie to the left of X_goal
% 
% Author: Long Kiu Chung
% Created: 2023/10/17
% Updated: 2023/10/17
%
% INPUTS:
% xytkk_range: 1*5 cell of 1*2 vectors; [lower_bound, upper_bound] of {1} 
%              x_lookup, {2} y_lookup, {3} theta, {4} k_1, and {5} k_2
% xy_sample_n: 1*2 cell of scalars; number of samples for {1} x_lookup and 
%              {2} y_lookup
%
% OUTPUTS:
% lookup_points: 1*total_sample_n cell of 1*5 vectors, where total_sample_n
%                = xy_sample_n{1}*xy_sample_n{2}; initial conditions 
%                [x_0_m, y_0_m, theta_0_m, k_1_0_m, k_2_0_m] of each lookup
%                points
    
    % Unpack cells
    x_sample_n = xy_sample_n{1};
    y_sample_n = xy_sample_n{2};
    x_range = xytkk_range{1};
    y_range = xytkk_range{2};
    k_2_range = xytkk_range{5};

    total_sample_n = x_sample_n.*y_sample_n;
    lookup_points = cell(1, total_sample_n);
    
    % This initial condition is a straight line trajectory to X_goal at
    % maximum speed
    k_1_0_m = 0;
    k_2_0_m = k_2_range(2);
    i = 0;
    for x_0_m = linspace(x_range(1), x_range(2), x_sample_n)
        for y_0_m = linspace(y_range(1), y_range(2), y_sample_n)
            theta_0_m = atan(y_0_m./x_0_m);
            if isnan(theta_0_m) % If x_0_m == 0 && y_0_m == 0
                theta_0_m = 0;
            end
            i = i + 1;
            lookup_points{i} = [x_0_m; y_0_m; theta_0_m; k_1_0_m; k_2_0_m];
        end
    end