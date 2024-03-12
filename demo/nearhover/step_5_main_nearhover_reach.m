%% Implement motion planning of near-hover with reach guarantee
%
% After implementing it, validates it with simulations and check if it is
% attributed to the tracking error being not robustly collected.
%
% Author: Wonsuhk Jung
% Created: Jan 19 2023

clear all; close all;

%% User parameters

% common near-hover initialization
init_nearhover;

% user initialization specific to this script
% Please over-ride params here instead of changing init_nearhover
visualize_reach_funnel_flag = true;

% Initial state of the agent.
z_0 = [5; 0; 5];
v_0 = 0.1;
s_0 = [z_0; v_0*ones(3,1); zeros(3,1)];

%% automated from here
% create trajectory producing model
TPM = TrajectoryPWASingleInt3D( ...
        'states_bound'       , states_bound, ...
        'params_bound'       , params_bound, ...
        'affinization_dt'    , affinization_dt, ...
        'affinization_N_grid', affinization_N_grid);

% create environment
% load X_goal and obstacles from the data
load(world_path);

% post-process the goal and obstacle
goal_center = mean((X_goal.V), 1);
goal_dim    = TPM.position_indices;

% create agent
controller = ctrlLazyLQR("q_x", ctrl_q_x, "q_y", ctrl_q_y, "q_z", ctrl_q_z);
A          = NearHoverAgent();
A.LLC      = controller;

% create the initial condition
A.reset(s_0);

% Visualize the simulation environment
visualize_simulation_env('agent', A, 'planner', TPM, ...
                         'goal', X_goal);

% create lookup points to evaluate backward reachable set
lookup_points = TPM.advise_lookup_points("state_lookup", state_lookup_grids, ...
                                         "goal", goal_center);

% compute Tracking Error Function (TEF)
load(tracking_error_data_path);
TED_x  = datas.tracking_error_x;
TED_z  = datas.tracking_error_z;

TEF_x  = TED_x.error_function();
TEF_z  = TED_z.error_function();
stamps = TED_x.stamps;

TEF            = struct();
TEF.errors_max = [TEF_x; TEF_x; TEF_z];
TEF.stamps     = stamps;

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

%% Validate the reach-set here
close all;
grids = {linspace(3, 8, 6), ...  % px-grid
         linspace(-5, 5, 6), ... % py-grid
         linspace(3, 7, 6), ...  % pz-grid
         0.4, 0.4, 0.4, 0, 0, 0};
states_grid = flatten_grid_to_array("grids", grids);

summary = simulate_grid('agent', A, 'planner', TPM, 'goal', X_goal, ...
                        'states_grid', states_grid, 'reach_set', brs_chains, ...
                        'visualize', true);

hold on; 
s = scatter3(states_grid(:, 1), states_grid(:, 2), states_grid(:, 3), 'red', 'filled');
s.SizeData = 10;

reach_goal_filter = cellfun(@(x)  x.reach_goal,summary);
collision_filter  = cellfun(@(x)  x.collision_check, summary);

fprintf("Reach goal rate: %f\n", sum(reach_goal_filter)/length(summary));
fprintf("Collision rate: %f\n", sum(collision_filter)/length(summary));

% Every motion plan should reach the goal. If there is any case that does
% not, check if it is attributed to non-robust error function.
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