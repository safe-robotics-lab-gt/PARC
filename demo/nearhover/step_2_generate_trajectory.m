%% Implement SingleIntegratorSym and test the parametrized trajectory
% The main thing you should create here is TrajectoryPWA class
%   TPWA.define_system
%   TPWA.make_desired_trajectory
%
% Test-out the trajectory producing model by visualizing trajectory
%
% Author: Wonsuhk Jung
% Created: Jan 18 2023

clear all; close all;

%% User parameters
init_nearhover;

z0 = [5; 5; 5];
t_plan  = 2;
vx_des  = 0.1;
vy_des  = 0.2;
vz_des  = 0.3;


%% automated from here
TPM = TrajectoryPWASingleInt3D( ...
        'states_bound'       , states_bound, ...
        'params_bound'       , params_bound, ...
        'affinization_dt'    , affinization_dt, ...
        'affinization_N_grid', affinization_N_grid);

[T_des, U_des, Z_des] = TPM.make_desired_trajectory(z0, t_plan, ...
                        'vx_des', vx_des, 'vy_des', vy_des, 'vz_des', vz_des);

x = Z_des(1, :);
y = Z_des(2, :);
z = Z_des(3, :);

%% Plot trajectory
figure(1) ; clf ; hold on ; grid on ; axis equal ; 
make_plot_pretty;
plot3(Z_des(1,:),Z_des(2,:),Z_des(3,:),'b--');
view(3) ;
xlabel("x"); ylabel("y"); zlabel("z");

