%% Implement reach behavior of turtlebotsym
% Implement compute_brs for reaching-goal behavior
%
% After implementing it, validates it with simulations and check if it is
% attributed to the tracking error being not robustly collected.
%
% Author: Wonsuhk Jung
% Created: Jan 16 2023

clear all; close all;
%%
init_turtlebotsym;

% Lookup Points setting
lookup_points = create_turtle_lookup(xytkk_lookup_range, xy_lookup_sample_n);

% Define trajectory producing model
TPM = TrajectoryPWADubins( ...
        'states_bound'       , states_bound, ...
        'params_bound'       , params_bound, ...
        'affinization_dt'    , affinization_dt, ...
        'affinization_N_grid', affinization_N_grid);

% Define agent
agent = turtlebot_agent('LLC', turtlebot_iLQR_LLC);

% Define tracking error function to use
load(tracking_error_data_path);
TED_x  = datas.tracking_error_x;

TEF            = struct();
TEF.errors_max = TED_x.error_function();
TEF.stamps     = TED_x.stamps;

% Define X_goal
theta_bound = Polyhedron('A', [1; -1], 'b', [th_bound(2); -th_bound(1)]);
K_bound     = make_set_rectangle2D('center', [mean(w_des_bound), mean(v_des_bound)], ...
                                   'height', v_des_bound(2)-v_des_bound(1), ...
                                   'width',  w_des_bound(2)-w_des_bound(1));
X_goal_2D   = make_set_square2D('center', [0, 0], 'side', 2);
X_goal_low  = X_goal_2D * theta_bound;
X_goal      = X_goal_2D * theta_bound * K_bound;

%% automated from here
t_max      = ceil(t_plan_max/TPM.affinization_dt);
brs_chains = TPM.compute_brs('X_goal', X_goal, ...
                             'lookup_points', lookup_points, ...
                             't_max', t_max, ...
                             'tracking_error', TEF);

BRS_cells  = cellfun(@(x) x.nodes(end), brs_chains, 'UniformOutput', false);
BRS        = [BRS_cells{:}];
BRS_low    = BRS.projection(TPM.state_indices);

% Computed backward reachable set of the X_goal with tracking error
BRS               = PolyUnion(BRS);
BRS_low_sample    = PolyUnion(BRS_low);

%% Validate the reach-set here
close all;
grids = {linspace(-6, -2, 10), ...       % px-grid
         linspace(-5, 5, 10), ...        % py-grid
         linspace(-pi/2, pi/2, 10), ...  % th-grid
         linspace(0, 1, 5)};             % v-grid
states_grid = flatten_grid_to_array("grids", grids);

summary = simulate_grid('agent', agent, 'planner', TPM, 'goal', X_goal_2D, ...
                        'states_grid', states_grid, 'reach_set', brs_chains, ...
                        'visualize', true);
