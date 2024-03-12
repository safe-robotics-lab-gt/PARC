function [goal, obstacles] = get_clbf_world_from_zono_world(zonotope_world)
%% Obstacle translator from zonotpe obstacle to rrt_obstacles
% Translate the zonotope world into the obstacle specifications for
% neural-clbf world. This can be generally used as parsing zonotope_world's
% goal and obstacles and exporting to the .mat file
% Author: Wonsuhk Jung
% Created: Jan 20th 2024

ZW = zonotope_world;

zono_obstacles = ZW.obstacles;
num_obstacle = length(zono_obstacles);

goal = struct();
goal.center = ZW.goal;
goal.radius = ZW.goal_radius;
goal.type   = ZW.goal_type;

obstacles = cell(1, num_obstacle);
for i = 1:num_obstacle
    obs_i = zono_obstacles{i};

    obs_clbf_i = struct();
    obs_clbf_i.center = obs_i.center;
    obs_clbf_i.size   = obs_i.body_dimensions;

    obstacles{i} = obs_clbf_i;
end

save("clbf_world", 'goal', 'obstacles', '-v7.3');
end
