%% description
% This script tests out the 10-D near hover quadrotor dynamics from the
% FasTrack paper
%
% Authors: Wonsuhk Jung
% Created: 15 Jan 2024
clear ; clc ;

%% user parameters
% pick which ode solver to use
t_move     = 5;
dt_int     = 0.1;

%% automated from here
% Define trajectory
% this is hand-crafted open-loop trajectory
T_ref = 0:dt_int:t_move;
U_ref = [0.02.*(2.5 - T_ref).^3 ; % accelerate and then brake
         0.4*cos(pi*T_ref) ; % wiggle left and right
         sin(pi*T_ref)] ; % wiggle upwards

% create 10D near-hover quadrotor agent
A = NearHoverAgent();
A.LLC = low_level_controller();

% reset agent
% yaw and yaw rate is automatically set to zero
A.reset();
A.move(t_move, T_ref, U_ref);

%% plotting
figure(1) ; clf ; axis equal ; hold on ; grid on ;

plot(A)

view(3) ;
make_plot_pretty()

A.animate()