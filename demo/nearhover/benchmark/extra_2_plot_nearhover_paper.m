%% This script is intended to visualize the simulation results of three methods
% We compare three methodologies here: FasTrack, Neural CLBF, PARC
%
% Creates Figure 6, 7, 8
% Author: Wonsuhk Jung
% Created: Feb 01 2024 (30 hours before submission...)
% Modified: Feb 07 2024
clear all; close all;

%% 1. Load Data
% PARC data
parc_rollout_data_path = "nearhover_PARC_rollout_result.mat";
% FasTrack data
fastrack_rollout_data_path = "nearhover_FasTrack_rollout_result.mat";
% CLBF data
clbf_rollout_data_path = "nearhover_CLBF_rollout_result.mat";

load(fastrack_rollout_data_path);
load(parc_rollout_data_path);
load(clbf_rollout_data_path);

%% 2. Visualization settings
palette = get_palette_colors();

%%
init_nearhover_paper;

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

% post-process the goal and obstacle
goal_dim = TPM.position_indices;
goal_center = mean((X_goal.V), 1);

obstacles = [obstacles{:}];
obstacles = obstacles + body_polytope;

%% Figure 1 - Plotting 8,100 simulations
close all;
reach_goal_filter = cellfun(@(x)  x.reach_goal,summary);
collision_filter  = cellfun(@(x)  x.collision_check, summary);

fprintf("Reach goal rate: %f\n", sum(reach_goal_filter)/length(summary));
fprintf("Collision rate: %f\n", sum(collision_filter)/length(summary));

% Visualize the simulation environment
obstacles_ = obstacles(7:8);
visualize_simulation_env('agent', A, 'planner', TPM, ...
                         'goal', X_goal, 'obstacle', obstacles_);hold on;
for i = 1:length(summary)
    h = plot_path(summary{i}.position_real); 
    h.Color = palette.blue;
end
% view(-135, 30)
view(270, 90)
xlabel("$p_x$", "Interpreter","latex", "FontSize", 30);
ylabel("$p_y$", "Interpreter","latex", "FontSize", 30);
zlabel("$p_z$", "Interpreter","latex", "FontSize", 30);

visualize_simulation_env('agent', A, 'planner', TPM, ...
                         'goal', X_goal, 'obstacle', obstacles_);hold on;
for i = 1:length(states_FT_history)
    h = plot_path(states_FT_history{i}([1, 5, 9], :));
    h.Color = palette.orange;
end
% view(-135, 30)
view(270, 90)
xlabel("$p_x$", "Interpreter","latex", "FontSize", 30);
ylabel("$p_y$", "Interpreter","latex", "FontSize", 30);
zlabel("$p_z$", "Interpreter","latex", "FontSize", 30);

visualize_simulation_env('agent', A, 'planner', TPM, ...
                         'goal', X_goal, 'obstacle', obstacles_);hold on;
for i = 1:size(agent_states, 1)
    agent_states_i = squeeze(agent_states(i, :, 1:3));
    if norm(agent_states_i(end, :) - goal_center) > 3
        continue
    end
    max_state = max(abs(agent_states_i(:, :)), [], 1);
    if max_state(1) > 10 || max_state(2) > 5 || max_state(3) >10
        continue
    end
    h = plot_path(agent_states_i');
    h.Color = palette.magenta;
end
% view(-135, 30)
view(270, 90)
xlabel("$p_x$", "Interpreter","latex", "FontSize", 30);
ylabel("$p_y$", "Interpreter","latex", "FontSize", 30);
zlabel("$p_z$", "Interpreter","latex", "FontSize", 30);

%% Analysis 1.1: Analyze Success Rate and Safety Rate
reach_mask = zeros(size(agent_states, 1), 1);
collide_mask = zeros(size(agent_states, 1), 1);
for i = 1:size(agent_states, 1)
    reach = false;
    collide = false;
    end_pos = squeeze(agent_states(i, end, 1:3));

    state_i = squeeze(agent_states(i, :, 1:3));

    if norm(end_pos - goal_center') < goal_radius
        reach = true;
    end
    
    if max(state_i(:, 1)) > 10 || min(state_i(:, 1)) <0 || ...
       max(state_i(:, 2)) > 10 || min(state_i(:, 2)) < -10 || ...
       max(state_i(:, 3)) > 10 || min(state_i(:, 3)) < 0
       collide = true;
    end
    
    dist1 = abs(state_i - obstacles(7).interiorPoint.x');
    dist2 = abs(state_i - obstacles(8).interiorPoint.x');

    collide_with_obs1 = any(all(dist1<[1, 8.3, 8.2]/2, 2));
    collide_with_obs2 = any(all(dist2<[1, 8.3, 8.2]/2, 2));

    collide = collide || (collide_with_obs1) || collide_with_obs2;
    reach_mask(i, 1) = reach;
    collide_mask(i, 1) = collide;
end
    

fprintf("Reach rate: %f\n", sum(reach_mask)/8100);
fprintf("Reach-avoid rate: %f\n", sum(~collide_mask&reach_mask)/8100);


%% Figure 2. Multi-trajectory comparison
% Visualize the simulation environment
close all;

palette = get_palette_colors;

% PARC
tracking_errors     = cellfun(@(x)  abs(x.position_des - x.position_real), summary, ...
                            'UniformOutput', false);
% tracking_errors_avg = cellfun(@(x) norm(mean(x, 2)), tracking_errors, 'UniformOutput',false);
tracking_errors_max = cellfun(@(x) max(x(2, :)), tracking_errors, 'UniformOutput',false);
[error, idx_PARC] = max([tracking_errors_max{:}]);
fprintf("error: %f \n", error);

idx_PARC = 201;

visualize_simulation_env('agent', A, 'planner', TPM, ...
                         'goal', X_goal, 'obstacle', obstacles); hold on;
make_plot_pretty;

pos_PARC              = summary{idx_PARC}.position_real;
plan_PARC             = summary{idx_PARC}.position_des;
h_plan_PARC           = plot_path(plan_PARC);hold on;
h_plan_PARC.Color     = palette.black;
h_plan_PARC.LineStyle  = '-.';
h_plan_PARC.LineWidth = 3;
h_pos_PARC            = plot_path(pos_PARC);hold on;
h_pos_PARC.Color      = palette.blue;
h_pos_PARC.LineWidth  = 2;


% FasTrack
state_of_interest = summary{idx_PARC}.init_state(1:3);
fastrack_pos_indices = [1,5,9];

init_states_FT = cellfun(@(x) x(fastrack_pos_indices, 1), states_FT_history, ...
                         'UniformOutput', false);
init_states_FT = cat(2, init_states_FT{:});
state_mask_FT  = all(init_states_FT == state_of_interest);
indices_FT     = find(state_mask_FT == 1);

idx_FT             = indices_FT(3);
pos_FT             = states_FT_history{idx_FT}(fastrack_pos_indices, :);
plan_FT            = plan_FT_history{idx_FT};
h_pos_FT           = plot_path(pos_FT);
h_pos_FT.Color     = palette.orange;
h_pos_FT.LineWidth = 2;



% CLBF
clbf_pos_indices = 1:3;
init_states_CLBF     = squeeze(agent_states(:, 1, clbf_pos_indices))';
state_mask_CLBF  = all(abs(init_states_CLBF - state_of_interest) <0.1);
indices_CLBF     = find(state_mask_CLBF == 1);

idx_CLBF         = indices_CLBF(1);
pos_CLBF             = squeeze(agent_states(idx_CLBF, :, clbf_pos_indices))';

h_pos_CLBF           = plot_path(pos_CLBF);
h_pos_CLBF.Color     = palette.magenta;
h_pos_CLBF.LineWidth = 2;

A.reset(summary{idx_PARC}.init_state);
plot(A);
legend([h_pos_FT, h_pos_CLBF, h_pos_PARC, h_plan_PARC], ...
       ["FaSTrack", "Neural CLBF", "PARC", "PARC-plan"], ...
       "Location", "northoutside", "Orientation", "horizontal", 'FontSize', 20);



xlim([3, 10]); zlim([1, 9])
xlabel("$p_x$", "Interpreter","latex", "FontSize", 30);
ylabel("$p_y$", "Interpreter","latex", "FontSize", 30);
zlabel("$p_z$", "Interpreter","latex", "FontSize", 30);

% title("Rollout trajectories of nearhover");

% Assuming you already have a figure created
fig = gcf; % Get handle to current figure

% Set the figure's size: [left bottom width height]
set(fig, 'Position', [100, 100, 800, 600]); % Resize figure to 800x600 pixels

view(-90, 45);


%% Figure 3. Tracking Error Functions
close all;
load("Quad10D_g101_dt050_t10_veryHigh_quadratic_info.mat");
TED_x.plot(); hold on;
h_error_PARC = plot(TEF.stamps, TEF.errors_max(1, :));
h_error_PARC.Color = palette.magenta;
h_error_PARC.LineWidth = 3;
h_error_PARC.LineStyle = '-.';
h_error_FT   = plot(TED_x.stamps, repmat(TEB, size(TED_x.stamps)));
h_error_FT.Color = palette.blue;
h_error_FT.LineWidth = 3;
h_error_FT.LineStyle = '-.';

legend([h_error_FT, h_error_PARC], ["FaSTrack TEB", "Ours"], 'FontSize', 15);

xlabel("$t (sec)$", 'Interpreter', 'latex', 'FontSize', 30);
ylabel("$e_x (m)$", 'Interpreter', 'latex', 'FontSize', 30);






