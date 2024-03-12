%% Scripts for generating figure for the paper
%
% Generates the Figure 2, Figure 3 for the paper
% Goal-Reaching Trajectory Design Near Danger with Piecewise Affine Reach-avoid Computation
% (https://arxiv.org/abs/2402.15604)
clear all; close all;
%% Visualization & configuration setting
seed = 1;
palette = get_palette_colors();
init_turtlebotsym_figure;

rng(seed);

% Define trajectory producing model
TPM = TrajectoryPWADubins( ...
        'states_bound'       , states_bound, ...
        'params_bound'       , params_bound, ...
        'affinization_dt'    , affinization_dt, ...
        'affinization_N_grid', affinization_N_grid);

% Define agent
agent = turtlebot_agent('LLC', turtlebot_iLQR_LLC);

% Define tracking error function to use
error_function_path = "error_function_new_ilqr.mat";
error_data = load(error_function_path, "data").data;

% Define X_goal
theta_bound = Polyhedron('A', [1; -1], 'b', [th_bound(2); -th_bound(1)]);
K_bound     = make_set_rectangle2D('center', [mean(w_des_bound), mean(v_des_bound)], ...
                                   'height', v_des_bound(2)-v_des_bound(1), ...
                                   'width',  w_des_bound(2)-w_des_bound(1));
X_goal_2D   = make_set_square2D('center', [0, 0], 'side', 2);
X_goal_low  = X_goal_2D * theta_bound;
X_goal      = X_goal_2D * theta_bound * K_bound;

%% Lookup point formula
x = -3.5;
y = 0;
theta = pi/5;

% get expert plan
w = -2*theta/t_plan_max;
v = abs(x/sin(theta))*theta/t_plan_max;

p0 = [x; y; theta];
k0 = [w; v];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%           FIGURE 2         %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Generate data for Figure 2
close all;

% Run expert trajectory
[T_des, U_des, Z_des] = TPM.make_desired_trajectory(p0, t_plan_max, ...
                                         "w_des", k0(1), "v_des", k0(2));
Z_des_1 = Z_des(1:3, Z_des(3, :)>=0);
Z_des_2 = Z_des(1:3, Z_des(3, :)<=0);

% Compute reach set
t_max      = ceil(t_plan_max/TPM.affinization_dt);
brs_chains = TPM.compute_brs('X_goal', X_goal, ...
                             'lookup_points', {[p0; k0]}, ...
                             't_max', t_max);

% Mode division
ms        = brs_chains{1}.activation_pattern;
BRS       = brs_chains{1}.nodes(end);
brs_nodes = brs_chains{1}.projection(1:3).nodes;
bdry_idx  = find(ms==11,1);
brs_nodes_1 = brs_nodes(2:bdry_idx);
brs_nodes_2 = brs_nodes(bdry_idx+1:end);

% Rollout trajectory
BRS_low = BRS.projection(1:3);
x_low   = sample_from_set('set', BRS_low);
k_low   = BRS.slice(1:3, x_low);
k_sample = sample_from_set('set', k_low);

[T_des_sp, U_des_sp, Z_des_sp] = TPM.make_desired_trajectory(x_low, t_plan_max, ...
                                         "w_des", k_sample(1), ...
                                         "v_des", k_sample(2));
%% Plot Figure 2 of the PARC paper
font_size = 20;
color1    = palette.blue;
color2    = palette.magenta;
interval  = 3;

% Plot goal
plot(X_goal.projection(1:3), "alpha", 0.25, "color", 'green', "linewidth", 2); hold on;
axis equal;

% Plot brs_nodes (mode 1)
plot(brs_nodes_1(1:interval:end), "color", color2, "alpha", 0.15, "linestyle", 'none')
% Plot brs_nodes (mode 2)
plot(brs_nodes_2(1:interval:end), "color", color1, "alpha", 0.15, "linestyle", 'none')
% Plot brs
plot(brs_nodes_2(end), "color", color1, "alpha", 0.1, "linewidth", 2)

% Expertrajectory (mode 1)
h_expert_1 = plot_path(Z_des_1(1:3, :)); grid on; hold on;
h_expert_1.LineWidth = 5;
h_expert_1.Color     = color1;
h_expert_1.LineStyle = '-.';

% Expertrajectory (mode 2)
h_expert_2 = plot_path(Z_des_2(1:3, :)); grid on; hold on;
h_expert_2.LineWidth = 5;
h_expert_2.Color     = color2;
h_expert_2.LineStyle = '-.';

% sample trajectory
h_sample = plot_path(Z_des_sp(1:3, :));
h_sample.LineWidth = 5;
h_sample.Color     = palette.black;
h_sample.LineStyle = '-.';

xlim([-5 2]); ylim([-2.5 2.5]); zlim([-pi pi]);
xlabel("$x$", 'interpreter', 'latex', 'FontSize', font_size); 
ylabel("$y$", 'interpreter', 'latex', 'FontSize', font_size); 
zlabel("$\theta$", "interpreter", "latex",'FontSize', font_size);

% Initial state
scatter3(p0(1), p0(2), p0(3), 100, palette.blue, 'filled');
scatter3(x_low(1), x_low(2), x_low(3), 100, palette.black, 'filled');

view(45, 45);

figure;
K_dom = TPM.get_bound_polytopes_by_dims([4, 5]);
plot(K_dom, "color", 'black', 'alpha', 0.1); hold on;
plot(BRS.projection(4:5), "color", 'green', 'alpha', 0.5);
scatter(k_sample(1), k_sample(2), 500, palette.black, 'p', 'filled')

set(gca, 'FontSize', 20);
xlabel("$k_1$", 'Interpreter', 'latex', 'FontSize', 30);
ylabel("$k_2$", 'Interpreter', 'latex', 'FontSize', 30);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%           FIGURE 3         %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Generate data for Figure 3

%% Plot Figure 3











