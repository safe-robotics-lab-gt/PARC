% track given trajectory [x,y,V] using iLQR
clear all;
close all;
clc;
initialization;
%genCostConstraintFunc;
tic
%% Test drift starting traj
dT = 0.1;
V_des = 9; % m/s 10
beta_switch = deg2rad(40);%deg2rad(35);
% (9,40) % (11,30) %(11,0.64)

% beta_vec = linspace(deg2rad(25), deg2rad(35), 10); % drifting side slip angle
beta_traj = deg2rad(15); % rad 40deg
states_init = [0,0,0];
S = genStartPath(states_init,V_des, beta_traj, dT);
t_p = S.t_s;

% Set the mpc horizon
param.t_horizon = 0.5; % seconds
% horizon = round(t_horizon / dT);
% Set max numebr of iterations
param.n_iterations = 50;

% Set desired state
n_states = 4;
n_inputs = 2;
init_states = [S.x(1);S.y(1);S.yaw(1);S.V(1)]; % Define the initial state to be the origin with no velocity and right heading
target_states = [S.x,S.y,S.yaw, S.V];%V_des*ones(length(t_s),1)]; % Get to 3 meters to the right facing up and stop [N,4] tall matrix
% Set initial guess for input
initial_guess = [S.V_dot,S.delta];

%% set up MPC
% Define weighting matrices
param.Q_k = 70*eye(n_states);
param.R_k = 30*eye(n_inputs);
param.Q_T = 100*eye(n_states);

% There are no physical parameters to adjust
param.parameters = [];

bounds = [Fxr_max, delta_max, Fxf_max];

A_predrift = bicycle_agent_predrifting(veh, tire_f, tire_r, bounds,dT);
A_predrift.visual = false;
A_predrift.simulate(t_p, init_states,initial_guess, target_states, param);

[point_found, t_span, states, states_dot, inputs, fiala_tire_vars] = ...
    find_switching_point(A_predrift,beta_switch);
if point_found
    disp("beta matched the desired drifting point, execute drifting controller")
    t_final = 4;
    %Control parameters
    V_des = states(end,5);
    beta_des = states(end,6);

    equil = getDriftState_w_V_beta(V_des, beta_des);
    Fxr_ff = equil.Fxr;
    delta_ff = equil.delta;
    r_des = equil.r;

    Ux_des = V_des*cos(beta_des); %8
    Uy_des = V_des*sin(beta_des); % -4.34

    % initial path condition same as the reference
    % r_radps(1)  = r_des; % 1.19
    r_radps(1)  = states(end,4); % 1.19
    uy_mps(1)   = Uy_des; % -4.34
    ux_mps(1)   = Ux_des; % 8
    beta_rad(1) = beta_des;
    V_mps(1) = V_des;
    x_m(1) = states(end,1);
    y_m(1) = states(end,2);
    yaw_rad(1) = states(end,3);

    init_drift_states = [x_m(1);y_m(1);yaw_rad(1);
        r_radps(1);V_mps(1);beta_rad(1);
        ux_mps(1);uy_mps(1)];
    input_guess = [Fxr_ff;delta_ff];
    des_rVb_states = [r_des;V_des;beta_des];
    prev_inputs = inputs(end,:);

    A_drift = bicycle_agent_drifting(veh, tire_f, tire_r, bounds,dT);
    A_drift.visual = false;
    A_drift.simulate(t_final,init_drift_states, input_guess,des_rVb_states,prev_inputs);
    A_drift.t_s = [t_span;A_drift.t_s+t_span(end)];
    A_drift.states = [states;A_drift.states];
    A_drift.states_dot = [states_dot;A_drift.states_dot];
    A_drift.inputs = [inputs;A_drift.inputs];
    A_drift.fiala_tire_vars = [fiala_tire_vars;A_drift.fiala_tire_vars];
    toc
    ax_space = A_drift.visualize();
else
    disp("beta didn't matched the desired drifting point")
    A_predrift.visualize();
end
