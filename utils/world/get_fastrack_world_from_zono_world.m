function [goal, obs] = get_fastrack_world_from_zono_world(zonotope_world, varargin)
%% Obstacle translator from zonotpe obstacle to rrt_obstacles
% Translate the zonotope_obstacles into the rrt_obstacles
% Author: Wonsuhk Jung
% Created: Jan 20th 2024

kwargs = parse_function_args(varargin{:});

agent_body_radius = [0; 0; 0];
if isfield(kwargs, 'agent_body_radius')
    agent_body_radius = kwargs.agent_body_radius;
end

save_world = false;
if isfield(kwargs, 'save_path')
    save_path = kwargs.save_path;
    save_world = true;
end

ZW = zonotope_world;

goal = struct();
goal.center = ZW.goal;
goal.radius = ZW.goal_radius;
goal.type   = ZW.goal_type;


zonotope_obstacles = ZW.obstacles;
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
    obs_rrt_plane_i = [cx cy-w/2 cz-h/2;...
                       cx cy-w/2 cz+h/2;...
                       cx cy+w/2 cz+h/2;...
                       cx cy+w/2 cz-h/2];

    pad_i = [t/2; 0; 0];
    pad_i = pad_i + agent_body_radius;

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

if save_world
    save(save_path, 'goal', 'obs', '-v7.3');
end
end

