%% Implement reach-avoid behavior of near-hover 
%
% After implementing it, validates it with simulations and check if it is
% attributed to the tracking error being not robustly collected.
%
% Author: Wonsuhk Jung, Long Kiu Chung
% Created: Jan 19 2024
% Updated: Mar 14 2024

clear all; close all;

%% User parameters
% Whether to include tracking error or not
% Produces Fig. 5 in the paper if true, produces Fig. 4 in the paper if false
include_error = true;

% common near-hover initialization
init_quad2dbad;

% Initial state of the agent.
z0 = [-0.842; 0.741; 0; 1; 0; 0];
f_norm = 4.905; % N
F1_des = f_norm-0.2; % 1.5 rad/s 
F2_des = F1_des+0.01; % %0.091

%% automated from here
% (1) Define a trajectory producing model
TPM = TrajectoryPWAQuad2dBad( ...
        'states_bound'       , states_bound, ...
        'params_bound'       , params_bound, ...
        'affinization_dt'    , affinization_dt, ...
        'affinization_N_grid', affinization_N_grid);


% (2) Define agent
A = quad2dAgent();
A.reset(z0);
body_polytope = Polyhedron('A', [eye(2); -eye(2)], 'b', [0.1; 0.1; 0.1; 0.1]);

% (3) Define Tracking Error Function (TEF)
load(tracking_error_data_path);
TED = datas.tracking_error;
stamps = TED.stamps;

TEF            = struct();
TEF.errors_max = TED.error_function();
TEF.stamps     = stamps;

% (4) Define environment
% for quadrotor example, this share the same environment.
X_goal = make_set_rectangle2D("center", goal_center,...
                                "width", sqrt(2)*goal_radius,...
                                "height", sqrt(2)*goal_radius);
o_1_real = make_set_rectangle2D("center", obstacle_config(1,1:2),...
                                "width", obstacle_config(1,3),...
                                "height", obstacle_config(1,4));
o_2_real = make_set_rectangle2D("center", obstacle_config(2,1:2),...
                                "width", obstacle_config(2,3),...
                                "height", obstacle_config(2,4));
obstacles = {o_1_real,o_2_real};

% post-process the goal and obstacle
goal_dim = TPM.position_indices;
goal_center = mean((X_goal.V), 1);

obstacles = [obstacles{:}];

% create lookup points to evaluate backward reachable set

lookup_points{1} = [z0; F1_des; F2_des];

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
if include_error
    brs_chains   = TPM.compute_brs('X_goal'        , X_goal_aug, ...
                              'lookup_points'  , lookup_points, ...
                              't_max'          , brs_step_max, ...
                              'tracking_error' , TEF);
else
    brs_chains   = TPM.compute_brs('X_goal'        , X_goal_aug, ...
                                   'lookup_points'  , lookup_points, ...
                                   't_max'          , brs_step_max);
end

% Compute backward reachable sets from backward reachable chain
BRS_cells  = cellfun(@(x) x.nodes(end), brs_chains, 'UniformOutput', false);
BRS        = [BRS_cells{:}];

%% Compute avoid sets
brs_chain = brs_chains{1};
ap = brs_chain.activation_pattern;
nodes = brs_chain.nodes;

E_obs = TPM.get_error_polyhedrons('t_max', brs_step_max, ...
                    'time_axis', TEF.stamps, ...
                    'error_function', TEF.errors_max, ...
                    'mode', 'obstacle');

avoids = [];

aug_dim = setdiff(TPM.avoidset_indices, TPM.position_indices);
aug_polytope = TPM.get_bound_polytopes_by_dims(aug_dim);
z_dim   = setdiff(1:TPM.n_state+TPM.n_param, TPM.avoidset_indices);
N_z     = length(z_dim);
is_avoidset_full_dim = (N_z == 0);
if ~is_avoidset_full_dim
    z_polytope    = TPM.get_bound_polytopes_by_dims(z_dim);    
    Rn_polyhedron = Polyhedron("A", zeros(1, N_z), "b", 0);
end
tic

for i = 1:length(obstacles)
    disp(['Now working on ', num2str(i), ' of ', num2str(length(obstacles)), ' obstacles'])
    obs = obstacles(i);
    for j = 1:length(ap)
        disp(['Now working on ', num2str(j), ' of ', num2str(length(ap)), ' timesteps'])
        if include_error
            obs_ij = obs + E_obs(end + 1 - j).projection(1:2);
        else
            obs_ij = obs;
        end
        obs_x = obs_ij*aug_polytope;
        obs_xz = cartesian_product_with_axis(obs_x, Rn_polyhedron, TPM.avoidset_indices, z_dim);

        obs_brss = TPM.brs_from_ap(ap(j), obs_xz);
        obs_brs = obs_brss(end).projection(TPM.avoidset_indices);

        obs_funnel = PolyUnion('Set', [obs_brs, obs_x]).convexHull;
        obs_funnel = cartesian_product_with_axis(obs_funnel, z_polytope, ...
                                        TPM.avoidset_indices, z_dim);

        obs_funnel = minHRep(obs_funnel);

        avoid = intersect(obs_funnel, nodes(j + 1));
        if ~avoid.isEmptySet()
            for k = (j + 1):length(ap)
                avoid_brs = TPM.brs_from_ap(ap(k), avoid);
                avoid = avoid_brs(end);
            end
            if ~avoid.isEmptySet()
                avoids = [avoids, avoid];
            end
        end
    end
end
toc

%% Compute BRAS
BRS = intersect(BRS, Polyhedron('A', [0, 0, 0, 1, 0, 0, 0, 0; 0, 0, 0, -1, 0, 0, 0, 0], 'b', [1.5;1.5]));
safe = BRS.slice([3, 5:6], [0;0;0])\avoids.slice([3, 5:6], [0;0;0]);

%% Plotting
figure();
hold on; grid on; box on
for i = 1:numel(obstacles)
    true_obstacle = obstacles - body_polytope;
    true_obstacle.plot('color', 'r','alpha',0.7);
end
safe.projection(1:2).plot('color','b','alpha',0.4,'linestyle','none');
X_goal.plot('color', 'g','alpha', 1);
axis equal
xlim(x_bound);
ylim(z_bound);
xlabel('$p_x$ (m)', 'Interpreter', 'latex', 'FontSize', 15)
ylabel('$p_z$ (m)', 'Interpreter', 'latex', 'FontSize', 15)
hold off;

