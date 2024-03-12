%% Initialize the system with same_setting
% Configurations to generate figures for the paper.
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
% The user should not be aware of backend-algorithm. Hence, all
% configurations are defined in the workspace.
% Quadrotor example uses the same environment.

% The world configuration used for the PARC paper
% world
bounds = [x_bound, y_bound, z_bound];
verbose_level = 10;
goal_radius = 1;

% grid obstacles setting
width_num = 2;          % number of grid in width direction (y-dir)
height_num = 1;         % number of grid in height direction (z-dir) 
width_clearance  = 1.8;    % clearance of grid in width direction (y-dir)
height_clearance = 1.8;   % clearance of grid in height direction (z-dir)
% width_clearance  = 3.0;    % clearance of grid in width direction (y-dir)
% height_clearance = 1.0;   % clearance of grid in height direction (z-dir)
obs_length = 1;
grid_xpos = 7;

% create zonotope_world
goal = [bounds(2) - goal_radius; 0; (bounds(5) + bounds(6))/2];
W = zonotope_box_world('verbose',verbose_level,'N_tall',0,...
    'N_wide',0,'N_boxy',0,'N_long',0,...
    'goal',goal,...
    'goal_radius',goal_radius,...
    'goal_type', 'box',...
    'bounds',bounds,...
    'use_wall_obstacles_flag',true);

W = add_grid_obstacles(W, [width_num, height_num], ... 
        [width_clearance, height_clearance], ... 
        grid_xpos, "length", obs_length);

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