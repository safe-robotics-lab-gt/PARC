%% Implement tracking controller and test the tracking performance.
% The main thing you should create here is LLC class
%   LLC.get_control_input()
%
% Test-out the performance of the tracking controller
%
% Author: Wonsuhk Jung
% Created: Jan 16 2023

clear all; close all;

%% User parameters

% trajectory initialization
z_0        = [-3; 0; 0.1];
t_plan     = 3;
w_des      = 0.4;
v_des      = 1.5;

% agent initialization
v_0        = 1;
s_0        = [z_0; v_0];

% controller
controller = turtlebot_iLQR_LLC;

%% automated from here
init_turtlebotsym;

% create trajectory producing model
TPM = TrajectoryPWADubins( ...
        'states_bound'       , states_bound, ...
        'params_bound'       , params_bound, ...
        'affinization_dt'    , affinization_dt, ...
        'affinization_N_grid', affinization_N_grid);

% create turtlebot
A = turtlebot_agent ;
A.LLC = controller;

% create the initial condition
A.reset(s_0);

%% trajectory setup and tracking
% make the desired trajectory
[T_des, U_des, Z_des] = TPM.make_desired_trajectory(z_0, t_plan, ...
                        'w_des', w_des, 'v_des', v_des);

% track the desired trajectory
A.move(T_des(end),T_des,U_des,Z_des) ;

%% tracking error computation
% get the realized position trajectory
T = A.time ;
Z = A.state(A.position_indices,:) ;

% interpolate the realized trajectory to match the braking traj timing
pos = match_trajectories(T_des,T,Z) ;

% get the desired trajectory
pos_des = Z_des(1:2,:) ;

% compute the tracking error
pos_err = abs(pos - pos_des) ;
x_err = pos_err(1,:) ;
y_err = pos_err(2,:) ;

%% animate robot
figure(1) ; clf ; axis equal ; hold on ; set(gca,'FontSize',15)

plot(Z_des(1,:),Z_des(2,:),'b--','LineWidth',1.5)
plot(A);
make_plot_pretty
legend('off'); grid on;
A.animate();

%% plot tracking error
figure(2) ; clf ; hold on ;
plot(T_des,x_err,'--','Color',[0.5 0.2 0.2],'LineWidth',1.5)
plot(T_des,y_err,'--','Color',[0.2 0.5 0.2],'LineWidth',1.5)
title('tracking error in x and y')
xlabel('time [s]')
ylabel('error [m]')
legend('x error','y error')
set(gca,'FontSize',15)

%% plot robot
figure(3) ; clf ; hold on ; axis equal ; grid on ;
plot_path(Z_des,'b--','LineWidth',1.5)
plot_path(Z,'b','LineWidth',1.5)
plot(A)
legend('desired traj','realized traj')
title('robot showing tracking error')
xlabel('x [m]')
ylabel('y [m]')
set(gca,'FontSize',15)





