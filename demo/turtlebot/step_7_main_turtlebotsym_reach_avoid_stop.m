%% Perform PARC with abstractized class
%
% Perform reach-avoid-stop behavior of the turtlebot agent
% Not implemented
%
% Author: Wonsuhk Jung
% Created: Jan 16 2023
% Updated: nah

clear all; close all; clc;

%% User Argument

% Define Initializer
% Your initializer should define following things
% 1) state_range
% 2) param_range
% 3) affinization grid: [nx, ny, nz, nk1, nk2]
% 4) affinization dt
% 5) max_brs_step
init_turtlebot_abstraction;

%% Automated from here

% Step 1. Define your trajectory producing model
% 1-1. Implement make_desired_trajectory function
TPM = TrajectoryPWATurtleSym( ...
        'states_bound'       , states_bound, ...
        'params_bound'       , params_bound, ...
        'affinization_dt'    , affinization_dt, ...
        'affinization_N_grid', affinization_N_grid);

% Step 2. Define your agent and controller
LLC   = turtlebot_iLQR_LLC;
agent = turtlebot_agent('LLC', LLC);

[T_des, U_des, Z_des] = pwa.make_desired_trajectory( ...
        z0, t_i, 'w_des', 0, 'v_des', 0.5);

% Step 3. Define your error-reachable-set



% Step 5. Compute reach-set

% Step 6. Compute avoid-set

% Step 7. Plot & Animation!

