%% Initialize the system with same_setting
% Initializer script to reproduce the figures in the PARC paper
% Figure 2, 3
%% Step 1. Agent configuration
body_polygon_num_side = 10;

%% Step 2. Trajectory-producing model configuration
% Define the region of the PWA System
% You have to define
%   states_bound
%   params_bound
%   affinization_dt
%   affinization_N_grid
%   t_plan

% Define the bound of affinization
x_bound = [-10, 1];
y_bound = [-10, 10];
th_bound = [-pi, pi];
w_des_bound = [-2, 2]; % w_des
v_des_bound = [0, 1.5]; % v_des

states_bound = {x_bound, y_bound, th_bound};
params_bound = {w_des_bound, v_des_bound};

% Define the discretization level
affinization_dt = 0.1;

% Define the affinization level
affinization_N_grid = [1; 1; 4; 1; 4];

% Maximum trajectory length (sec)
t_plan_max = 4;

%% Step 3. Environment configuration
% The user should not be aware of backend-algorithm. Hence, all
% configurations are defined in the workspace.

% goal configuration (square)
goal_center = [0, 0];
goal_side   = 2;

% obstacle configuration (rectangle)
% center_x, center_y, width, height
obstacle_config  = [-1.375, -1.125, 0.6, 1.25;
                    -1.375,  1.375, 0.6, 0.75;
%                     -3.3,  4.3, 0.65, 0.6;
%                     -5  , -4.3, 1.75, 1.5;
%                     -4.3, -7.3, 1   , 0.8;
                    ];

%% Step 4. Tracking error function configuration
% This is the result of the interaction between agent and the trajectory
% producing model. Error data should have this data. TPM, agent, controller
% should be injective to tracking_error_function.
% ex) error_function_turtlebot_PWATurtleSym_turtlebotilQR.mat

tracking_error_function_path = "error_function_new_ilqr.mat";

%% Step 5. Lookup-point configuration
% This is the grid of initial points for BRSChain generation

x_lookup_range = [-10, 1];
y_lookup_range = [-10, 10];

x_lookup_sample_n = 10; % To sample at an interval of 0.5
y_lookup_sample_n = 10;

xytkk_lookup_range = {x_lookup_range, y_lookup_range, th_bound, w_des_bound, v_des_bound};
xy_lookup_sample_n = {x_lookup_sample_n, y_lookup_sample_n};