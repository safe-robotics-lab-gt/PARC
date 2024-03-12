%% Build tracking error function of Nearhover-SingleIntegrator3D
% Build tracking error function by sampling. For computational ease, the
% tracking error is collected in decoupled fashion.
%
% error_function_x
% error_function_y (= error_function_x)
% error_function_z
%
% Author: Wonsuhk Jung
% Created: Jan 22th 2024
% TODO: implement robustify_tracking_error_function

clear all; close all;

%% User Parameter
init_nearhover;

% Initial state vector grid
% The state bounds of the grid is motivated by the FasTrack configuration

% x-state grid
x_grid   = 5; 
vx_grid  = linspace(-1, 1, 10);
thx_grid = linspace(-45*pi/180, 45*pi/180, 10);
wx_grid  = linspace(-0.5, 0.5, 10);

% x-param grid
v_delta_grid_x = linspace(-0.5, 0.5, 3);

% z-state grid
z_grid         = 3;
vz_grid        = linspace(-2, 2, 10);

% z-param grid
v_delta_grid_z = linspace(-0.5, 0.5, 10);

% Error function horizon
t_plan_max = 10;

%% automated from here
% (1) define trajectory producing model
TPM = TrajectoryPWASingleInt3D( ...
        'states_bound'       , states_bound, ...
        'params_bound'       , params_bound, ...
        'affinization_dt'    , affinization_dt, ...
        'affinization_N_grid', affinization_N_grid);

% (2) define agent
controller    = ctrlLazyLQR("q_x", ctrl_q_x, "q_y", ctrl_q_y, "q_z", ctrl_q_z);
A             = NearHoverAgent();
A.LLC         = controller;

%% collect tracking error x, y function
% (3) collect tracking error function
initial_state_grid_x = flatten_grid_to_array('grids', {x_grid, vx_grid, thx_grid, wx_grid});
n_initial_states     = length(initial_state_grid_x);

progress_bar = waitbar(0, 'Collecting tracking error in x-direction.');

te_x_datas = {};

for i = 1:n_initial_states
    waitbar(i/n_initial_states, progress_bar);

    % Parse initial states
    x_i  = initial_state_grid_x(i, 1);
    v_i  = initial_state_grid_x(i, 2);
    th_i = initial_state_grid_x(i, 3);
    w_i  = initial_state_grid_x(i, 4);

    % Augment the high-fidelity state with dummy value
    % this does not mess up because dynamics is decoupled
    s0_i                                = zeros(A.n_states, 1);
    s0_i(A.position_indices, :)         = x_i;
    s0_i(A.velocity_indices, :)         = v_i;
    s0_i(A.angular_velocity_indices, :) = w_i;

    R0_i                                = eul2rotm([th_i, th_i, 0], 'ZYX');

    % Define the params grid adaptively
    v_des_grid = v_i + v_delta_grid_x;
    v_des_grid = v_des_grid((v_des_grid >= TPM.params_bound{1}(1)) & ...
                            (v_des_grid <= TPM.params_bound{1}(2)));

    for j = 1:length(v_des_grid)
        % parse traj_param
        k_j = v_des_grid(j);

        % initialize and augment initial low-fidelity state
        z0_ij                       = zeros(TPM.n_state, 1);
        z0_ij(TPM.state_indices, :) = x_i;

        k_ij                        = zeros(TPM.n_param, 1);
        k_ij(1:TPM.n_param)         = k_j;

        % produce trajectory
        traj_varargin = key_value_to_varargin(TPM.param_names, k_ij);
        [T_des, U_des, Z_des] = TPM.make_desired_trajectory(z0_ij, ...
                                    t_plan_max, traj_varargin{:});

        % move agent
        A.reset(s0_i, R0_i);
        A.move(T_des(end), T_des, U_des, Z_des);

        % evaluate at the finer grid
        T_eval       = 0:A.integrator_time_discretization:max(T_des);
        pos_eval     = match_trajectories(T_eval, A.time, A.state(A.position_indices, :));
        pos_des_eval = match_trajectories(T_eval, T_des,  Z_des(TPM.position_indices, :));
        err_eval     = abs(pos_eval - pos_des_eval);

        % data logging: write custom data struct here
        data_ij = TrackingErrorData('errors', err_eval(1, :), ...
                                    'stamps', T_eval, ...
                                    'initial_state', [x_i, v_i, th_i, w_i], ...
                                    'traj_param',    k_j);

        te_x_datas{end+1} = data_ij;
    end
end
close(progress_bar);

% concatenate tracking error data
error_data_x = te_x_datas{1}.extend('datas', te_x_datas(2:end));

%% collect z-dir tracking error data
% (4) collect tracking error function
initial_state_grid_z = flatten_grid_to_array('grids',{z_grid, vz_grid});
n_initial_states     = length(initial_state_grid_z);

progress_bar = waitbar(0, 'Collecting tracking error in z-direction.');

te_z_datas = {};

for i = 1:n_initial_states
    waitbar(i/n_initial_states, progress_bar);

    % Parse initial states
    z_i  = initial_state_grid_z(i, 1);
    v_i  = initial_state_grid_z(i, 2);

    % Augment the high-fidelity state with dummy value
    % this does not mess up because dynamics is decoupled
    s0_i                                = zeros(A.n_states, 1);
    s0_i(A.position_indices, :)         = z_i;
    s0_i(A.velocity_indices, :)         = v_i;

    % Define the params grid adaptively
    v_des_grid = v_i + v_delta_grid_z;
    v_des_grid = v_des_grid((v_des_grid >= TPM.params_bound{3}(1)) & ...
                            (v_des_grid <= TPM.params_bound{3}(2)));

    for j = 1:length(v_des_grid)
        % parse traj_param
        k_j = v_des_grid(j);
        
        % initialize and augment initial low-fidelity state
        z0_ij                       = zeros(TPM.n_state, 1);
        z0_ij(TPM.state_indices, :) = z_i;

        k_ij                        = zeros(TPM.n_param, 1);
        k_ij(1:TPM.n_param)         = k_j;
        
        % produce trajectory
        traj_varargin = key_value_to_varargin(TPM.param_names, k_ij);
        [T_des, U_des, Z_des] = TPM.make_desired_trajectory(z0_ij, ...
                                    t_plan_max, traj_varargin{:});

        % move agent
        A.reset(s0_i);
        A.move(T_des(end), T_des, U_des, Z_des);

        % evaluate at the finer grid
        T_eval       = 0:A.integrator_time_discretization:max(T_des);
        pos_eval     = match_trajectories(T_eval, A.time, A.state(A.position_indices, :));
        pos_des_eval = match_trajectories(T_eval, T_des,  Z_des(TPM.position_indices, :));
        err_eval     = abs(pos_eval - pos_des_eval);

        % data logging: write custom data struct here
        data_ij = TrackingErrorData('errors', err_eval(3, :), ...
                                    'stamps', T_eval, ...
                                    'initial_state', [z_i, v_i], ...
                                    'traj_param',    k_j);
        
        te_z_datas{end+1} = data_ij;
    end
end
close(progress_bar);

% concatenate tracking error data
error_data_z = te_z_datas{1}.extend('datas', te_z_datas(2:end));


%% Visualize the tracking error function

% visualize tracking error data
error_data_x.plot();
error_data_z.plot();

% save the tracking error data
datas.tracking_error_x = error_data_x;
datas.tracking_error_z = error_data_z;
datas.states_grid_x    = initial_state_grid_x;
datas.states_grid_z    = initial_state_grid_z;
datas.params_grid_x    = v_delta_grid_x;
datas.params_grid_z    = v_delta_grid_z;
datas.traj_model       = TPM;
datas.agent            = A;
datas.description      = "State is (x, vx, thx, wx) and param is (vx_des)";

% save the data
directory = ['mat/Nearhover_TError_' num2str(now)];
mkdir(directory);
save([directory '/Nearhover_TError.mat'], 'datas','-v7.3');