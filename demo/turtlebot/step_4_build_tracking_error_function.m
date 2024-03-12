%% Build tracking error function of Nearhover-SingleIntegrator3D
% Build tracking error function by sampling. For computational ease, the
% tracking error is collected in decoupled fashion.
%
% error_function_x
% error_function_y (= error_function_x)
% error_function_z
%
% Author: Wonsuhk Jung
% Created: Mar 11th 2024
% TODO: implement robustify_tracking_error_function

clear all; close all;

%% User Parameter
init_turtlebotsym;

% Initial state vector grid
% The state bounds of the grid is motivated by the FasTrack configuration

% state grid
x_grid   = 0;
y_grid   = 0;
th_grid  = linspace(-pi, pi, 10);
v_grid   = linspace(0, 2.0, 10);

% param grid
v_delta_grid = linspace(-0.5, 0.5, 10);
w_des_grid = linspace(0, 1.5, 10);

% Error function horizon
t_plan_max = 3;

%% automated from here
% (1) define trajectory producing model
TPM = TrajectoryPWADubins( ...
        'states_bound'       , states_bound, ...
        'params_bound'       , params_bound, ...
        'affinization_dt'    , affinization_dt, ...
        'affinization_N_grid', affinization_N_grid);

% (2) define agent
controller    = turtlebot_iLQR_LLC;
A             = turtlebot_agent();
A.LLC         = controller;

%% collect tracking error x, y function
% (3) collect tracking error function
initial_state_grid   = flatten_grid_to_array('grids', {x_grid, y_grid, th_grid, v_grid});
n_initial_states     = length(initial_state_grid);

progress_bar = waitbar(0, 'Collecting tracking error');

te_datas = {};

for i = 1:n_initial_states
    waitbar(i/n_initial_states, progress_bar);

    % Parse initial states
    x_i  = initial_state_grid(i, 1);
    y_i  = initial_state_grid(i, 2);
    th_i = initial_state_grid(i, 3);
    v_i  = initial_state_grid(i, 4);

    % Augment the high-fidelity state with dummy value
    % this does not mess up because dynamics is decoupled
    s0_i                                = [x_i; y_i; th_i; v_i];

    % Define the params grid adaptively
    v_des_grid = v_i + v_delta_grid;
    v_des_grid = v_des_grid((v_des_grid >= TPM.params_bound{2}(1)) & ...
                            (v_des_grid <= TPM.params_bound{2}(2)));

    for j = 1:length(v_des_grid)
        for k = 1:length(w_des_grid)
            % parse traj_param
            k_iter = [w_des_grid(k); v_des_grid(j)];
    
            % initialize and augment initial low-fidelity state
            z0_ij                       = [x_i; y_i; th_i];
    
            % produce trajectory
            traj_varargin = key_value_to_varargin(TPM.param_names, k_iter);
            [T_des, U_des, Z_des] = TPM.make_desired_trajectory(z0_ij, ...
                                        t_plan_max, traj_varargin{:});

            if length(T_des) ~= size(Z_des, 2)
                continue
            end
    
            % move agent
            A.reset(s0_i);
            A.move(T_des(end), T_des, U_des, Z_des);
    
            % evaluate at the finer grid
            T_eval       = 0:A.integrator_time_discretization:max(T_des);
            pos_eval     = match_trajectories(T_eval, A.time, A.state(A.position_indices, :));
            pos_des_eval = match_trajectories(T_eval, T_des,  Z_des(TPM.position_indices, :));
            err_eval     = abs(pos_eval - pos_des_eval);
    
            % data logging: write custom data struct here
            data_iter = TrackingErrorData('errors', err_eval, ...
                                        'stamps', T_eval, ...
                                        'initial_state', [x_i, y_i, th_i, v_i], ...
                                        'traj_param',    k_iter);
    
            te_datas{end+1} = data_iter;
        end
    end
end
close(progress_bar);

% concatenate tracking error data
error_data_x = te_datas{1}.extend('datas', te_datas(2:end));


%% Visualize the tracking error function

% visualize tracking error data
error_data_x.plot();

% save the tracking error data
datas.tracking_error_x = error_data_x;
datas.states_grid_x    = initial_state_grid;
datas.params_grid_x    = v_delta_grid;
datas.traj_model       = TPM;
datas.agent            = A;
datas.description      = "State is (x, y, th, v) and param is (w_des, v_des)";

% save the data
directory = ['mat/Turtlebot_TError_' num2str(now)];
mkdir(directory);
save([directory '/Turtlebot_TError.mat'], 'datas','-v7.3');