% track given trajectory [x,y,V] using iLQR
clear all;
close all;
clc;
initialization;
%genCostConstraintFunc;

%% Test drift starting traj
dT = 0.05;
V_des = 15; % m/s 15
beta_des = deg2rad(30); % rad
S = genStartPath(V_des, beta_des, dT);
t_p = S.t_s;

% Set the mpc horizon
param.t_horizon = 1; % seconds
% horizon = round(t_horizon / dT);
% Set max numebr of iterations
param.n_iterations = 100;

% Set desired state
n_states = 4;
n_inputs = 2;
init_states = [S.x(1);S.y(1);S.yaw(1);S.V(1)]; % Define the initial state to be the origin with no velocity and right heading
target_states = [S.x,S.y,S.yaw, S.V];%V_des*ones(length(t_s),1)]; % Get to 3 meters to the right facing up and stop [N,4] tall matrix
% Set initial guess for input
initial_guess = [S.V_dot,S.delta];

%% set up MPC 


% Define weighting matrices
param.Q_k = 70*eye(n_states); % zero weight to penalties along a strajectory since we are finding a trajectory
param.R_k = 30*eye(n_inputs);
% R_k(2,2) = 50;

param.Q_T = 100*eye(n_states);
% Q_T(1,1) = 10;
% Q_T(2,2) = 10;
% Q_T(3,3) = 10;

% There are no physical parameters to adjust
param.parameters = [];

bounds = [Fxr_max, delta_max, Fxf_max];

A_predrift = bicycle_agent_predrifting(veh, tire_f, tire_r, bounds,dT);
A_predrift.simulate(t_p, init_states,initial_guess, target_states, param);

