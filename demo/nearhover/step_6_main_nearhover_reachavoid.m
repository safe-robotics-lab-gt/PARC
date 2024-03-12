%% Implement reach-avoid behavior of near-hover
%
% After implementing it, validates it with simulations and check if it is
% attributed to the tracking error being not robustly collected.
%
% Author: Wonsuhk Jung
% Created: Jan 19 2024
% Updated: Jan 23 2024

clear all; close all;

%% User parameters
% common near-hover initialization
init_nearhover;

% user initialization specific to this script
% Please over-ride params here instead of changing init_nearhover
visualize_reach_funnel_flag   = false;
visualize_simulation_env_flag = true;

%% automated from here
% (1) Define a trajectory producing model
TPM = TrajectoryPWASingleInt3D( ...
        'states_bound'       , states_bound, ...
        'params_bound'       , params_bound, ...
        'affinization_dt'    , affinization_dt, ...
        'affinization_N_grid', affinization_N_grid);

% (2) Define agent
controller    = ctrlLazyLQR("q_x", ctrl_q_x, "q_y", ctrl_q_y, "q_z", ctrl_q_z);
A             = NearHoverAgent();
A.LLC         = controller;
body_polytope = A.footprint_polytope();

% (3) Define Tracking Error Function (TEF)
load(tracking_error_data_path);
TED_x  = datas.tracking_error_x;
TED_z  = datas.tracking_error_z;

TEF_x  = TED_x.error_function();
TEF_z  = TED_z.error_function();
stamps = TED_x.stamps;

TEF            = struct();
TEF.errors_max = [TEF_x; TEF_x; TEF_z];
TEF.stamps     = stamps;

% (4) Define environment
% for quadrotor example, this share the same environment.
load(world_path)

% post-process the goal and obstacle
goal_dim = TPM.position_indices;
goal_center = mean((X_goal.V), 1);

obstacles = obstacles + body_polytope;

% Visualize the simulation environment
visualize_simulation_env('agent', A, 'planner', TPM, ...
                         'goal', X_goal, 'obstacle', obstacles);

% create lookup points to evaluate backward reachable set
lookup_points = TPM.advise_lookup_points("state_lookup", state_lookup_grids, ...
                                         "goal", goal_center);

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
         "color", 'green', "alpha", 0.5);
    plot(brs_chains{vis_idx}.projection(goal_dim).nodes(1:end-1), ...
         "color", 'red', "alpha", 0.2);
    
    axis equal; grid on;
    hold_switch(hc);
end

%% Compute avoid sets
obstacle_funnels = TPM.get_obstacle_funnels( ...
                             'obstacles', obstacles, ...
                             't_max', brs_step_max, ...
                             'verbose', true, ...
                             'tracking_error', TEF);

avoid_sets = TPM.compute_avoid_set('reach_funnel', brs_chains, ...
                                   'avoid_funnel', obstacle_funnels,...
                                   'obstacle', obstacles);

%% Validate the reach-avoid set
close all;
grids = {linspace(3, 6, 10),...  % p_x grid
         linspace(-1, 1, 10),... % p_y grid
         linspace(5, 5, 1), ...  % p_z grid
         0.4, 0.4, 0.4, 0, 0, 0};
states_grid = flatten_grid_to_array("grids", grids);

summary = simulate_grid('agent', A, 'planner', TPM, 'goal', X_goal, ...
                        'obstacle', obstacles, 'states_grid', states_grid, ...
                        'reach_set', brs_chains, 'avoid_set', avoid_sets, ...
                        'visualize', true, 'collision_check', false);

hold on; 
s = scatter3(states_grid(:, 1), states_grid(:, 2), states_grid(:, 3), 'r');

%% Inspect the summary
reach_goal_filter = cellfun(@(x)  x.reach_goal, summary);
fprintf("Reach goal rate: %f\n", sum(reach_goal_filter)/length(summary));

if isfield(summary, "collision")
    collision_filter  = cellfun(@(x)  x.collision,  summary);
    fprintf("Collision rate: %f\n", sum(collision_filter)/length(summary));
end