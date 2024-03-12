%% Implement reach behavior of turtlebotsym
% Implement compute_avoid_set for reach-avoid navigation of turtlebot
%
% After implementing it, validates it with simulations and check if it is
% attributed to the tracking error being not robustly collected.
%
% Author: Wonsuhk Jung
% Created: Jan 17th 2024
% Updated: Jan 18th 2024

clear all; close all;

%% User-defined parameters
% Write own initializer
init_turtlebotsym;

visualize_simulation_env_flag = true;
visualize_reach_funnel_flag   = true;
visualize_avoid_funnel_flag   = true;

%% automated from here
% Lookup Points setting
lookup_points = create_turtle_lookup(xytkk_lookup_range, xy_lookup_sample_n);

% (1) Define trajectory producing model
TPM = TrajectoryPWADubins( ...
        'states_bound'       , states_bound, ...
        'params_bound'       , params_bound, ...
        'affinization_dt'    , affinization_dt, ...
        'affinization_N_grid', affinization_N_grid);

% (2) Define agent
controller   = turtlebot_iLQR_LLC;
agent        = turtlebot_agent('LLC', controller);
body_polygon = agent.footprint_polygon("num_side", body_polygon_num_side);

% (3) Define tracking error function to use
load(tracking_error_data_path);
TED_x  = datas.tracking_error_x;

TEF            = struct();
TEF.errors_max = TED_x.error_function();
TEF.stamps     = TED_x.stamps;

% (4) Define environment

% define target set representations and inform what dimensions of TPM are
% used for target set.
X_goal      = make_set_square2D('center', goal_center, 'side', goal_side);
goal_dim    = TPM.position_indices;

% define obstacle set representations
N_obstacle   = size(obstacle_config, 1);
obstacles = []; % polytopic obstacle representations in workspace

for i = 1:N_obstacle
    c_i       = obstacle_config(i, 1:2); % center information
    w_i       = obstacle_config(i, 3); % width information
    h_i       = obstacle_config(i, 4); % height information

    obs_i     = make_set_rectangle2D('center', c_i,  'width', w_i, 'height', h_i);
    obs_i     = obs_i + body_polygon;
    
    obstacles = [obstacles, obs_i];
end

% Visualize the simulation environment.
visualize_simulation_env('agent', agent, 'planner', TPM, ...
                         'goal', X_goal, 'obstacle', obstacles)

%% Compute reach sets
% This section compute backward reachable chain (reach-funnels) and
% backward reachable sets from
%
%   lookup point   : x0 \in R^(n_state+n_param)
%   t_max          : 
%   X_goal         : polytope \in R^(n_goal_dim)
%   tracking_error : error    \in R^(n_goal_dim)

% sanity check before running computing reach sets
assert(length(lookup_points{1})      == TPM.n_state+TPM.n_param);
assert(size(TEF.errors_max,1) == length(goal_dim));

% augment the target set to full state (=n_state+n_param)
non_goal_dim = setdiff(1:TPM.n_state+TPM.n_param, goal_dim);
X_goal_aug   = X_goal * TPM.get_bound_polytopes_by_dims(non_goal_dim);

% maximum step of backward reachable set
brs_step_max = ceil(t_plan_max/TPM.affinization_dt);

% Compute backward reachable chains from lookup point.
brs_chains   = TPM.compute_brs('X_goal'        , X_goal_aug, ...
                              'lookup_points'  , lookup_points, ...
                              't_max'          , brs_step_max, ...
                              'tracking_error' , TEF);

% Compute backward reachable sets from backward reachable chain
BRS_cells  = cellfun(@(x) x.nodes(end), brs_chains, 'UniformOutput', false);
BRS        = [BRS_cells{:}];

% Visualize the reach funnels if needed
if visualize_reach_funnel_flag
    figure; hc = hold_switch();
    vis_idx = 1;
    
    plot(BRS(vis_idx).projection(goal_dim), ...
         "color", 'green', "alpha", 1);
    plot(brs_chains{vis_idx}.projection(goal_dim).nodes, ...
         "color", 'gray', "alpha", 0.01);
    
    axis equal; grid on;
end

%% Compute avoid sets

% This function should be called when agent senses the obstacle
obstacle_funnels = TPM.get_obstacle_funnels( ...
                             'obstacles', obstacles, ...
                             't_max', brs_step_max, ...
                             'tracking_error', TEF, ...
                             'verbose', true);

avoid_sets = TPM.compute_avoid_set('reach_funnel', brs_chains, ...
                                   'avoid_funnel', obstacle_funnels,...
                                   'obstacle', obstacles);

%% Validate the avoid sets
close all;
grids = {linspace(-6, -2, 10), ...       % px-grid
         linspace(-2, 2, 5), ...        % py-grid
         linspace(-pi/2, pi/2, 10), ...  % th-grid
         linspace(0.5, 1, 5)};             % v-grid
states_grid = flatten_grid_to_array("grids", grids);

summary = simulate_grid('agent', agent, 'planner', TPM, 'goal', X_goal, ...
                        'obstacle', obstacles, 'states_grid', states_grid, ...
                        'reach_set', brs_chains, 'avoid_set', avoid_sets, ...
                        'visualize', true);

%% Inspect the summary
reach_goal_filter = cellfun(@(x)  x.reach_goal,summary);
collision_filter  = cellfun(@(x)  x.collision, summary);

fprintf("Reach goal rate: %f\n", sum(reach_goal_filter)/length(summary));
fprintf("Collision rate: %f\n", sum(collision_filter)/length(summary));

bug_cases = summary(:, ~reach_goal_filter);
nonrobust_error_filter = [];
for i = 1:length(bug_cases)
    bug_case = bug_cases{i};
    error_expected = match_trajectories(bug_case.stamps(end), ...
                        TEF.stamps, TEF.errors_max);
    error_real     = bug_case.position_real - bug_case.position_des;
    error_real     = error_real(:, end);

    nonrobust_error_filter = [nonrobust_error_filter, ...
                                any(abs(error_real) > error_expected)];
end

fprintf("Nonrobust error detected: %f\n", sum(nonrobust_error_filter)/length(bug_cases));










