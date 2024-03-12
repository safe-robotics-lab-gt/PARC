%% Implement turtlebot agent class and test the dynamics of the turtlebot.
% The main thing you should create here is agent class
%   agent.dynamics
%   agent.plot
%   agent.plot_at_time
%
% Give the fixated control inputs and see if the agent moves as expected.
%
% Author: Wonsuhk Jung
% Created: Jan 16 2024
% Updated: Mar 11 2024

%% user parameters
t_move     = 5;
dt_int     = 0.1;

%% automated from here
% Define trajectory
% this is hand-crafted open-loop trajectory
T_ref = 0:dt_int:t_move;
U_ref = [1*ones(size(T_ref)) ;   % angular velocity 1(rad/s) clockwise
         0.5*ones(size(T_ref))]; % linear velocity = 0.5(m/s)

% create turtlebot agent
A = turtlebot_agent;
% set controller: outputs an interpolation of the reference input. 
A.LLC = low_level_controller;

% reset agent
A.reset();
A.move(t_move, T_ref, U_ref, []);

%% plotting
figure(1) ; clf ; axis equal ; hold on ; grid on ;

plot(A)
make_plot_pretty()

A.animate()