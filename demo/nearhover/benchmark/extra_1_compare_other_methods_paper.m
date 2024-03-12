%% This script is intended to compare three methodologies
% This script rolls out the PARC, FasTrack for the grid of initial states
% to compare. Neural CLBF methods are tested on the Python Environment,
% hence not showing up in this script.
%
% Author: Wonsuhk Jung
% Created: Feb 01 2024
% Modified: Feb 07 2024

clear all; close all;

%% User parameters
run_PARC     = false;
run_FasTrack = true;

% PARC configuraion
init_nearhover_paper;

% FasTrack configuration
fastrack_reach_set_data = "Q10D_Q3D_uplan_10_tenth.mat";

% Define the grid of initial states to roll-out.
if ~exist('states_grid', 'var')
    grids = {linspace( 3,   6,     10),... %px
             linspace(-3,   3,     10),... %py
             linspace( 3,   7,      1),... %pz
             0,...
             0,...
             0,...
             0, ...   %wx
             0, ...   %wy
             0};      %wz
    
    states_grid = flatten_grid_to_array("grids", grids);
end

save("nearhover_initial_states_data", "states_grid", "-v7.3");

if run_PARC
%% 2. PARC
% 1.1 PARC setup
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
[X_goal, obstacles] = get_poly_world_from_zono_world(W);
% Delete the walls because PARC is handling it through state bounds
obstacles = obstacles(7:end);

% post-process the goal and obstacle
goal_dim = TPM.position_indices;
goal_center = mean((X_goal.V), 1);

obstacles = [obstacles{:}];
obstacles = obstacles + body_polytope;

% Visualize the simulation environment
visualize_simulation_env('agent', A, 'planner', TPM, ...
                         'goal', X_goal, 'obstacle', obstacles);

% create lookup points to evaluate backward reachable set
lookup_points = TPM.advise_lookup_points("state_lookup", state_lookup_grids, ...
                                         "goal", goal_center);

% 1.2 PARC compute reach sets

% sanity check before running computing reach sets
assert(length(lookup_points{1})      == TPM.n_state+TPM.n_param);

% augment the target set to full state (=n_state+n_param)
non_goal_dim = setdiff(1:TPM.n_state+TPM.n_param, goal_dim);
X_goal_aug   = X_goal * TPM.get_bound_polytopes_by_dims(non_goal_dim);

% maximum step of backward reachable set
brs_step_max = ceil(t_plan_max/TPM.affinization_dt);

tic;
% Compute backward reachable chains from lookup point.
brs_chains   = TPM.compute_brs('X_goal'        , X_goal_aug, ...
                              'lookup_points'  , lookup_points, ...
                              't_max'          , brs_step_max, ...
                              'tracking_error' , TEF);

% Compute backward reachable sets from backward reachable chain
BRS_cells  = cellfun(@(x) x.nodes(end), brs_chains, 'UniformOutput', false);
BRS        = [BRS_cells{:}];

% 1.3 PARC compute avoid sets
obstacle_funnels = TPM.get_obstacle_funnels( ...
                             'obstacles', obstacles, ...
                             't_max', brs_step_max, ...
                             'verbose', true, ...
                             'tracking_error', TEF);

avoid_sets = TPM.compute_avoid_set('reach_funnel', brs_chains, ...
                                   'avoid_funnel', obstacle_funnels,...
                                   'obstacle', obstacles);
toc;

% 1.4 PARC generate plot to data
summary = simulate_grid('agent', A, 'planner', TPM, 'goal', X_goal, ...
                        'obstacle', obstacles, 'states_grid', states_grid, ...
                        'reach_set', brs_chains, 'avoid_set', avoid_sets, ...
                        'visualize', true);

hold on; 
s = scatter3(states_grid(:, 1), states_grid(:, 2), states_grid(:, 3), 'r');

save("nearhover_PARC_rollout_result.mat", ...
    "summary", "A", "TPM", "X_goal", "obstacles", "states_grid", "brs_chains", "avoid_sets", ...
    "-v7.3");
end % end of PARC computation



if run_FasTrack
%% 2. FasTrack
% 2.1 Fastrack Setting
% load the HJB value grid, Tracking Error Bound
load(fastrack_reach_set_data);
[goal_FT, obs_FT] = get_fastrack_world_from_zono_world(W, ...
                            'agent_body_radius', [0.27;0.27;0.025]);
states_FT_history = {};
controls_FT_history = {};
plan_FT_history = {};

pb = waitbar(0, "Simulating FasTrack");
for i = 1:size(states_grid,1)
% 2.2 Run Fastrack
waitbar(i/size(states_grid,1), pb);
w0 = states_grid(i, 1:3)';
try
[tracker_FT, plan_FT, controller_FT] = ...
               simulateFastrack(sD_X, derivX, sD_Z, derivZ, TEB, obs_FT, goal_FT, ...
                 'start', w0, ...
                 'control_gain', [ctrl_q_x; ctrl_q_y; ctrl_q_z], ...
                 'dt', 0.01, ...
                 'switch_control_margin', 0.5, ...
                 't_max', 10, ...
                 'visualize', false);
catch
    continue
end


% 2.3 Record Fastrack states
states_FT   = tracker_FT.xhist;
controls_FT = tracker_FT.uhist;

states_FT_history{end+1} = states_FT;
controls_FT_history{end+1} = controls_FT;
plan_FT_history{end+1} = plan_FT;
end
save("nearhover_FasTrack_rollout_result.mat", ...
    "states_FT_history", "controls_FT_history", "plan_FT_history",...
    "-v7.3");
end


