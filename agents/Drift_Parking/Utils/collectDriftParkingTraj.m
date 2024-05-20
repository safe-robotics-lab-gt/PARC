%% Iteratively run main_drift_parking script and store trajectory
% Varying V_des and beta and collect the corresponding drift trajectories
% with inputs
% Author: Chuizheng Kong
% created: 01/20/2024

clear all; close all; clc;
save_dir = fullfile(what(fullfile('PARC')).path,'..','data');
Filename = sprintf('driftParkingTraj_%s.mat', datestr(now,'mm-dd-yyyy HH-MM'));
save_path = fullfile(save_dir,Filename);
tic
initialization;
dT = 0.1;
iter = 0;
data_cell = {};
%% set up MPC
n_states = 4;
n_inputs = 2;
% Set the mpc horizon
param.t_horizon = 0.5; % seconds
% Set max numebr of iterations
param.n_iterations = 50;
% Define weighting matrices
param.Q_k = 70*eye(n_states);
param.R_k = 30*eye(n_inputs);
param.Q_T = 100*eye(n_states);

% There are no physical parameters to adjust
param.parameters = [];

bounds = [Fxr_max, delta_max, Fxf_max];
%% Iteration grid
states_init = [0,0,0];
V_vec = linspace(9, 11, 100); % aiming drifting velocity
beta_vec = linspace(deg2rad(35), deg2rad(45), 100); % drifting side slip angle
N = length(V_vec) * length(beta_vec);

%% define agents
A_predrift = bicycle_agent_predrifting(veh, tire_f, tire_r, bounds,dT);
A_predrift.visual = false;

A_drift = bicycle_agent_drifting(veh, tire_f, tire_r, bounds,dT);
A_drift.visual = false;
%% Iteration
for V_drift = V_vec
    for beta_switch = beta_vec
        iter = iter + 1;
        fprintf("[%d/%d] V_des: %.3f, beta_switch: %.3f \n", ...
            iter, N, V_drift, beta_switch)

        beta_traj = deg2rad(15); % rad 40deg
        S = genStartPath(states_init,V_drift, beta_traj, dT);
        
        % S = genStartPath(V_drift, beta_drift, dT);
        t_p = S.t_s;
        
        init_states = [S.x(1);S.y(1);S.yaw(1);S.V(1)]; % Define the initial state
        target_states = [S.x,S.y,S.yaw, S.V];% [N,4] tall matrix
        % Set initial guess for input
        initial_guess = [S.V_dot,S.delta];
        
        %simulate predrifting trajectory
        A_predrift.simulate(t_p, init_states,initial_guess, target_states, param);
        
        % detect if desired drifting slide slip angle beta was achieved
        [point_found, t_span, states, states_dot, inputs, fiala_tire_vars] = ...
            find_switching_point(A_predrift,beta_switch);
        if point_found
            t_final = 4;
            %Control parameters using equilibrium method
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
            
            init_states = [x_m(1);y_m(1);yaw_rad(1);
                r_radps(1);V_mps(1);beta_rad(1);
                ux_mps(1);uy_mps(1)];
            input_guess = [Fxr_ff;delta_ff];
            des_rVb_states = [r_des;V_des;beta_des];
            prev_inputs = inputs(end,:);
            
            % simulate drifting and parking trajectory
            A_drift.simulate(t_final,init_states, input_guess,des_rVb_states,prev_inputs);
            
            % data logging
            data_i = struct();
            data_i.TrajParam = [V_drift, beta_switch];
            % data_i.successful_drift = true;
            % data_i.t_s = [t_span;A_drift.t_s+t_span(end)];
            data_i.states = [states(:,1:3);A_drift.states(:,1:3)];
            % data_i.states_dot = [states_dot;A_drift.states_dot];
            % data_i.inputs = [inputs;A_drift.inputs];
            % data_i.fiala_tire_vars = [fiala_tire_vars;A_drift.fiala_tire_vars];
        else
            disp("beta did not matched the desired drifting point")
            data_i = struct();
            data_i.TrajParam = [V_drift, beta_switch];
            % data_i.successful_drift = false;
            % data_i.t_s = A_predrift.t_s;
            data_i.states = A_predrift.states(:,1:3);
            % data_i.states_dot = A_predrift.states_dot;
            % data_i.inputs = A_predrift.inputs;
            % data_i.fiala_tire_vars = A_drift.fiala_tire_vars;
        end
        % Data Logging
        data_cell{end+1} = data_i;
    end %beta_des
end %V_des
toc

driftTrajData = data_cell;
save(save_path, 'driftTrajData');


%% Simple Analysis: Error function
% % x_m               = A.states(:,1);
% % y_m               = A.states(:,2);
% % yaw_rad           = A.states(:,3);
% % r_radps           = A.states(:,4);
% % V_mps             = A.states(:,5);
% % beta_rad          = A.states(:,6);
% % ux_mps            = A.states(:,7);
% % uy_mps            = A.states(:,8);
% % 
% % Fxr_N             = A.inputs(:,1);
% % delta_rad         = A.inputs(:,2);
% % Fxf_N             = A.inputs(:,3);
figure;
cmap = hot(length(driftTrajData));
for i = 1:length(driftTrajData)
    data_i = driftTrajData{i};
    
    subplot(3, 1, 1);
    hold on;
    plot(data_i.states(:,1),'Color',cmap(i,:));
    ylabel('x (m)')
    colormap(cmap);
    colorbar;
    
    subplot(3, 1, 2);
    hold on;
    plot(data_i.states(:,2),'Color',cmap(i,:));
    ylabel('y (m)')
    colormap(cmap);
    colorbar;

    subplot(3,1,3);
    hold on;
    plot(data_i.states(:,3)*180/pi,'Color',cmap(i,:));
    ylabel('yaw (deg)')
    colormap(cmap);
    colorbar;
    if data_i.states(end,3)*180/pi < -200
    disp(data_i.TrajParam)
    i_test = i;
    end
end
hold off

%% to visualize specific trajectory
% data_i = driftTrajData{i_test};
% visualize_traj(A_drift, data_i);
function visualize_traj(A,data_i)
A.t_s = data_i.t_s;
A.states = data_i.states;
A.states_dot = data_i.states_dot;
A.inputs = data_i.inputs;
A.fiala_tire_vars = data_i.fiala_tire_vars;
A.visualize();
end