function [poly_goal, poly_obstacles] = get_poly_world_from_zono_world(zonotope_world)
% Translate zonotope world into the polytope world
% You should install CORA 2018 to use this function.

ZW = zonotope_world;

% Parse goal
poly_goal = make_set_cube3D('center', ZW.goal, 'side', ZW.goal_radius*2);

% Parse obstacle and translate into the polytopes
zono_obstacles = ZW.obstacles;
num_obstacle = length(zono_obstacles);
poly_obstacles = cell(num_obstacle, 1);
for i = 1:length(zono_obstacles)
    zono_i      = zono_obstacles{i}.zono;
    cora_poly_i = mptPolytope(zono_i); % This is not actual mpt polytope
    v_data_i    = get(vertices(cora_poly_i), 'V')';
    mpt_poly_i  = Polyhedron(v_data_i);
    poly_obstacles{i, 1} = mpt_poly_i;
end
end

