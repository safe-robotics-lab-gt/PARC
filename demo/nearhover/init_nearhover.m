%% Initialize the system with same_setting
%% Step 1. Agent configuration
% body_polygon_num_side = 10;
ctrl_q_x    = 13; % controller gain for lazyLQR
ctrl_q_y    = 13; % controller gain for lazyLQR
ctrl_q_z    = 0.01; % controller gain for lazyLQR

%% Step 2. Trajectory-producing model configuration
% Define the region of the PWA System
% You have to define
%   states_bound
%   params_bound
%   affinization_dt
%   affinization_N_grid
%   t_plan

% Define the bound of affinization
x_bound = [0, 10];
y_bound = [-10, 10];
z_bound = [0, 10];

vx_des_bound = [-0.5, 0.5]; % vx_des
vy_des_bound = [-0.5, 0.5]; % vy_des
vz_des_bound = [-0.5, 0.5]; % vz_des

states_bound = {x_bound, y_bound, z_bound};
params_bound = {vx_des_bound, vy_des_bound, vz_des_bound};

% Define the discretization level
affinization_dt = 0.1;

% Define the affinization level
affinization_N_grid = [1; 1; 1; 1; 1; 1];

% Maximum trajectory length (sec)
t_plan_max = 10;

%% Step 3. Environment configuration
% create polytope world: polytopic representation of obstacle and goal set.
world_path = "world_nearhover.mat";

%% Step 4. Tracking error function configuration
% This is the result of the interaction between agent and the trajectory
% producing model. Error data should have this data. TPM, agent, controller
% should be injective to tracking_error_function.
% ex) error_function_turtlebot_PWATurtleSym_turtlebotilQR.mat

tracking_error_data_path = "Nearhover_TError.mat";

%% Step 5. Lookup-point configuration
% % This is the grid of initial points for BRSChain generation
x_lookup_grid       = linspace(1, 10, 5); 
y_lookup_grid       = linspace(1, 10, 5);
z_lookup_grid       = linspace(1, 10, 5);

state_lookup_grids  = {x_lookup_grid, y_lookup_grid, z_lookup_grid};