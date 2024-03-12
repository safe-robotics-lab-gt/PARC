%% Create the video for nearhover comparison
% Create the animation
clear all; close all;

%% User Flag
init_nearhover_paper;

vis_PARC     = true;
vis_CLBF     = true;
vis_FasTrack = true;
save_video   = true;
camera_view_style = 'behind';

palette = get_palette_colors();

%% 1. Load Data
% PARC data
parc_rollout_data_path = "nearhover_PARC_rollout_result.mat";
% FasTrack data
fastrack_rollout_data_path = "nearhover_FasTrack_rollout_result.mat";
% fastrack_rollout_data_path = "nearhover_FasTrack_rollout_result_large_gap.mat";
% CLBF data
clbf_rollout_data_path = "nearhover_CLBF_rollout_result.mat";

if vis_PARC, load(parc_rollout_data_path); end
if vis_FasTrack, load(fastrack_rollout_data_path); end
if vis_CLBF, load(clbf_rollout_data_path); end

%% PARC configuration
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

% (2-1) Agent visualization settings
A.camera_view_style = camera_view_style;
A.plot_body_edge_color = palette.purple;
A.camera_follow_distance = 1.5;
video_name = A.animation_video_filename;

% (3) Define environment
[X_goal, obstacles] = get_poly_world_from_zono_world(W);
obstacles = [obstacles{:}];
obstacles = obstacles([1, 3, 4, 7, 8]); % obstacles to visualize

% (4) State of interest
idx_of_interest   = 1254; % Appendix video
% idx_of_interest = 1282;
state_of_interest = summary{idx_of_interest}.init_state(1:3);

%% Piecewise-Affine Reach-avoid Computation
if vis_PARC
    close all;
    A.animation_video_filename = strcat(video_name, "_", A.camera_view_style, "_", "PARC");
    % Extract state of interest
    pos_indices_parc = A.position_indices;
    init_states_parc = cellfun(@(x) x.init_state(pos_indices_parc), summary, ...
                             'UniformOutput', false);
    init_states_parc = cat(2, init_states_parc{:});
    state_mask_parc  = all(init_states_parc == state_of_interest);
    indices_parc     = find(state_mask_parc == 1);

    idx_PARC         = indices_parc(1);
    
    % Inject the state histories
    A.reset();
    A.state    = summary{idx_PARC}.states;
    A.time     = summary{idx_PARC}.stamps;
    A.attitude = summary{idx_PARC}.attitudes;

    visualize_simulation_env('agent', A, 'planner', TPM, ...
                             'goal', X_goal, 'obstacle', obstacles);

    axis equal ; grid off ; axis off; hold on ;
    
    % plot(A)
    
    view(3) ;
    make_plot_pretty()
    
    v = A.animate('save_video', save_video);
    close(v)
end

if vis_FasTrack
    close all;
    % Pick the indices that match to state_of_interest
    A.animation_video_filename = strcat(video_name, "_", A.camera_view_style, "_", "FasTrack");
    fastrack_pos_indices = [1,5,9];
    
    init_states_FT = cellfun(@(x) x(fastrack_pos_indices, 1), states_FT_history, ...
                             'UniformOutput', false);
    init_states_FT = cat(2, init_states_FT{:});
    state_mask_FT  = all(init_states_FT == state_of_interest);
    indices_FT     = find(state_mask_FT == 1);

    idx_FT         = indices_FT(1);

    stamps = 0:0.01:10;
    % Convert Fastrack convention to Nearhover convention
    states_ft = states_FT_history{idx_FT};
    states_nh = [states_ft([1, 5, 9], :);
                 states_ft([2, 6, 10], :);
                 states_ft([3, 7], :);
                 states_ft([4, 8], :)];

    % Convert Nearhover convention to Agent convention
    [states, attitudes] = A.dynamics_state_to_agent_state(states_nh);

    A.reset();
    A.state = states;
    A.attitude = attitudes;
    A.time = stamps(1:end-1);

    % visualize environment
    visualize_simulation_env('agent', A, 'planner', TPM, ...
                             'goal', X_goal, 'obstacle', obstacles);
    axis equal ; grid off; axis off; hold on ;
    
    % visualize agent
    % plot(A)
    
    view(3) ;
    make_plot_pretty()
    material dull; lighting gouraud;
    v= A.animate('save_video', save_video);
    close(v);
end
if vis_CLBF
    close all;
    A.animation_video_filename = strcat(video_name, "_", A.camera_view_style, "_", "CLBF");
    
    % Pick the indices that match to state_of_interest
    clbf_pos_indices = 1:3;
    init_states_CLBF = squeeze(agent_states(:, 1, clbf_pos_indices))';
    state_mask_CLBF  = all(abs(init_states_CLBF - state_of_interest) < 0.1);
    indices_CLBF     = find(state_mask_CLBF == 1);
    
    idx_CLBF         = indices_CLBF(1);

    stamps = 0:0.01:10;
    stamps_clbf = 0.001:0.001:9.999;

    % Convert clbf convention to Nearhover convention
    states_clbf = squeeze(agent_states(idx_CLBF, :, :));
    states_nh   = match_trajectories(stamps, stamps_clbf, states_clbf');
    pos_nh      = states_nh(A.position_indices, :);
    
    % Convert Nearhover convention to Agent convention
    [states, attitudes] = A.dynamics_state_to_agent_state(states_nh);

    % Collision detecting
    dist1 = abs(pos_nh - obstacles(1).interiorPoint.x);
    dist2 = abs(pos_nh - obstacles(2).interiorPoint.x);

    collide_with_obs1 = all(dist1<[1; 8.3; 8.2]/2, 1);
    collide_with_obs2 = all(dist2<[1; 8.3; 8.2]/2, 1);

    collide = collide_with_obs1 | collide_with_obs2;
    t_collide = find(collide, 1, 'first');


    % TODO: Implement fall down after collision maybe?

    visualize_simulation_env('agent', A, 'planner', TPM, ...
                             'goal', X_goal, 'obstacle', obstacles);
    axis equal ; hold on ; grid off; axis off;
    
    view(3) ;
    make_plot_pretty()
    
    % Before collision
    A.reset();
    A.state = states(:, 1:end);
    A.attitude = attitudes(:, :, 1:end);
    A.time = stamps(:, 1:end);

    %plot(A); 
    v= A.animate('save_video', save_video); close(v);
end

%% Experiment code