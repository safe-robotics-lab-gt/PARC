%% Initialize the system with same_setting
%% Step 1. Agent configuration
drone.r = 0.25; % m
drone.m = 1.0; % kg
drone.I = 0.01; % kg*m^2
drone.g = 9.81; 

%% Step 2. Trajectory-producing model configuration
% Define the region of the PWA System
% You have to define
%   states_bound
%   params_bound
%   affinization_dt
%   affinization_N_grid
%   t_plan

% Define the bound of affinization
x_bound = [-2, 2];
z_bound = [-0.3,2];
theta_bound = [-pi, pi];

vx_bound = [-2, 2]; % vx_des
vz_bound = [-2, 2]; % vz_des
tdot_bound = [-2*pi, 2*pi]; % theta_bound

f_norm = drone.m * drone.g /2; % N
F1_bound = [f_norm-0.5, f_norm+0.5];
F2_bound = F1_bound;

states_bound = {x_bound, z_bound, theta_bound,vx_bound, vz_bound, tdot_bound};
params_bound = {F1_bound, F2_bound};

% Define the discretization level
affinization_dt = 0.1;

% Define the affinization level
affinization_N_grid = [1; 1; 5; 1; 1; 1; 4; 4];

% Maximum trajectory length (sec)
t_plan_max = 2;

%% Step 3. Environment configuration
% The user should not be aware of backend-algorithm. Hence, all
% configurations are defined in the workspace.
% Quadrotor example uses the same environment.

% world
bounds = [x_bound, z_bound, theta_bound];
verbose_level = 10;

% grid obstacles setting
% width_num = 2;          % number of grid in width direction (y-dir)
% height_num = 1;         % number of grid in height direction (z-dir) 
% width_clearance  = 1.8;    % clearance of grid in width direction (y-dir)
% height_clearance = 1.8;   % clearance of grid in height direction (z-dir)
% obs_length = 1;
% grid_xpos = 7;

% create zonotope_world
goal_center = [0, 0];
goal_radius   = 0.3;
% center_x, center_y, width, height
obstacle_config  = [-0.75, 0.1, 0.5, 0.8; 
                    0.5,  1.1, 1, 0.6];
% 
%% Step 4. Tracking error function configuration
% This is the result of the interaction between agent and the trajectory
% producing model. Error data should have this data. TPM, agent, controller
% should be injective to tracking_error_function.
% ex) error_function_turtlebot_PWATurtleSym_turtlebotilQR.mat

% tracking_error_data_path = "quad2d_TError_good_360.mat";
tracking_error_data_path = "quad2dbad_TError.mat";


%% Step 5. Lookup-point configuration
% % This is the grid of initial points for BRSChain generation
% for this example only one lookup point is used which is z0

% state_lookup_grids  = {x_lookup_grid, y_lookup_grid, z_lookup_grid};