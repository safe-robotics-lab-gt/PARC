function rrt_obstacles = generate_obstacles_from_zonotopes(zonotope_obstacles)
%% Obstacle translator from zonotpe obstacle to rrt_obstacles
% Translate the zonotope_obstacles into the rrt_obstacles
% Author: Wonsuhk JUng
% Created: Jan 20th 2024

N_obstacle = length(zonotope_obstacles);
rrt_obstacles = [];
for i = 1:N_obstacle
    % To generate 3D cuboid, we have to design 6 faces
    % We use obs.pad function to do this

    obs_i = zonotope_obstacles{i};

    % Parse zonotope obstacle information
    center_i   = obs_i.center;
    body_dim_i = obs_i.body_dimensions;

    cx = center_i(1);
    cy = center_i(2);
    cz = center_i(3);

    t = body_dim_i(1);
    w = body_dim_i(2);
    h = body_dim_i(3);

    % Translate it to the rrt-plane
    obs_rrt_plane_i = [cx cy-w/2 cz-h/2; ...
                       cx cy-w/2 cz+h/2;...
                       cx cy+w/2 cz+h/2;...
                       cx cy+w/2 cz-h/2];

    pad_i = [t/2; 0; 0];

    obstacle_padder = ObstacleMapRRT([]);
    
    obstacle_padder.pad(pad_i, obs_rrt_plane_i);

    obs_rrt_i = obstacle_padder.padded_obs;

    if i == 1
        rrt_obstacles = obs_rrt_i;
    else
        rrt_obstacles = cat(3, rrt_obstacles, obs_rrt_i);
    end
end
obs = rrt_obstacles;


obsMap = ObstacleMapRRT(obs);
figure
obsMap.plotGlobal;

save("obs_rrt_from_zonotope", 'obs', '-v7.3');
end

