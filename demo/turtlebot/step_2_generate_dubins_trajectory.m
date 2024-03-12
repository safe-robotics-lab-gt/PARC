%% Implement TrajectoryPWATurtleSym and test the parametrized traject
% The main thing you should create here is TrajectoryPWA class
%   TPWA.define_system
%   TPWA.define_braking_system (optional)
%   TPWA.make_desired_trajectory
%
% Test-out the trajectory producing model by visualizing trajectory
%
% Author: Wonsuhk Jung
% Created: Jan 16 2023
clear all; close all;

%% User parameters
init_state = [-3; 0; 0.01];
t_plan     = 3;
w_des      = 1;
v_des      = 1.5;


%% automated from here
init_turtlebotsym;

TPM = TrajectoryPWADubins( ...
        'states_bound'       , states_bound, ...
        'params_bound'       , params_bound, ...
        'affinization_dt'    , affinization_dt, ...
        'affinization_N_grid', affinization_N_grid);

[T_des, U_des, Z_des] = TPM.make_desired_trajectory(init_state, t_plan, ...
                        'w_des', w_des, 'v_des', v_des);

x = Z_des(1, :);
y = Z_des(2, :);

% Plot trajectory
figure(1) ;
plot(x,y,'b--','LineWidth',1.5); grid on;
xlabel("$p_x$", "Interpreter", "latex");
ylabel("$p_y$", "interpreter", "latex");
axis equal
legend('desired traj.')
set(gca,'FontSize',15)

% Plot each state-time plot
figure(2) ;
t = tiledlayout(2, 1);
nexttile;
plot(T_des(1:size(Z_des, 2)), Z_des(1, :), 'LineWidth', 1.5);
xlabel("$t$", "Interpreter", "latex");
ylabel("$p_x$", "interpreter", "latex");
grid on;

nexttile;
plot(T_des(1:size(Z_des, 2)), Z_des(2, :), 'LineWidth', 1.5);
xlabel("$t$", "Interpreter", "latex");
ylabel("$p_y$", "interpreter", "latex");
grid on;