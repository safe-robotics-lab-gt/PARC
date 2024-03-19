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
init_quad2dbad;
f_norm = 4.905; % N

% Initial state vector grid
% "Expert plan"
state_ini = [-0.75, 0.75, 0, 0, 0, 0]';

% x-state grid
f_rad = 0.05;
f_center = f_norm-0.2;
F1_grid  = linspace(f_center-f_rad, f_center+f_rad, 10);
F2_grid = F1_grid;
vx_grid = linspace(-1.5,1.5,10);

%% automated from here
% (1) define trajectory producing model
TPM = TrajectoryPWAQuad2dBad( ...
        'states_bound'       , states_bound, ...
        'params_bound'       , params_bound, ...
        'affinization_dt'    , affinization_dt, ...
        'affinization_N_grid', affinization_N_grid);

% (2) define agent
A = quad2dAgent();

%% collect tracking error x, y function
% (3) collect tracking error function
input_grid = flatten_grid_to_array('grids', {F1_grid, F2_grid});
n_input = length(input_grid);
progress_bar = waitbar(0, 'Collecting tracking error by varying F1 X F2.');
n_iter = n_input*length(vx_grid);
quad2d_error_datas = {};
tic
for iv = 1:length(vx_grid)
for i = 1:n_input
    prog = ((iv-1)*n_input+i);
    waitbar(prog/n_iter, progress_bar,sprintf('i=%d',prog));

    % Parse initial states
    F1  = input_grid(i, 1);
    F2  = input_grid(i, 2);
    state_ini(4) = vx_grid(iv);
    [T_des, U_des, Z_des] = TPM.make_desired_trajectory(state_ini,t_plan_max,...
                                'F1', F1, 'F2', F2);
    % move agent
    A.reset(state_ini);
    A.move(T_des(end), T_des, U_des, Z_des);

    % evaluate at the finer grid
    T_eval       = 0:A.integrator_time_discretization:max(T_des);
    pos_eval     = match_trajectories(T_eval, A.time, A.state(A.position_indices, :));
    pos_des_eval = match_trajectories(T_eval, T_des,  Z_des(TPM.position_indices, :));
    err_eval     = abs(pos_eval - pos_des_eval);

    % data logging: write custom data struct here
    data_ij = TrackingErrorData('errors', err_eval, ...
                                'stamps', T_eval, ...
                                'traj_param', [F1;F2], ...
                                'vx_0', vx_grid(iv));

    quad2d_error_datas{end+1} = data_ij;
end
end
close(progress_bar);
%% 

% concatenate tracking error data
error_data = quad2d_error_datas{1}.extend('datas', quad2d_error_datas(2:end));


%% Visualize the tracking error function

% visualize tracking error data
error_data.plot();

% save the tracking error data
datas.tracking_error = error_data;
datas.inital_state = state_ini;
datas.traj_model       = TPM;
datas.agent            = A;
datas.description      = "State is (x,z,th, vx,vz,thd) and param is (F1,F2)";

% save the data
save('demo/quad2d/quad2dbad/data/quad2dbad_TError.mat', 'datas','-v7.3');
toc