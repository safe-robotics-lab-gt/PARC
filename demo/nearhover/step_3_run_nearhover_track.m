%% Implement tracking controller and test the tracking performance.
% The main thing you should create here is LLC class
%   LLC.get_control_input()
%
% Test-out the performance of the tracking controller
%
% Author: Wonsuhk Jung
% Created: Jan 18 2023

clear all; close all;

%% User parameters
init_nearhover;

% trajectory initialization
z_0     = [6.6;-3.0;5];
t_plan  = 10;


v = [0.5,    0.146,  0.15;
     0.442,  0.161,  0.3;
     0.5,   -0.146,  0.151;
     0.477, -0.169,  0;
    -0.5 ,  -0.5  , -0.5;
     0.289,  0.4004  ,  0];

idx = 6;
vx_des = v(idx, 1);
vy_des = v(idx, 2);
vz_des = v(idx, 3);

% agent initialization
v_0        = 0.05;
w_0        = 0;
s_0        = [z_0; 0.4; 0.4; 0.4; w_0*ones(3,1)];

% controller
controller = ctrlLazyLQR("q_x", 13, "q_y", 13, "q_z", 0.01);

animate_robot = true;

%% automated from here
% create trajectory producing model
TPM = TrajectoryPWASingleInt3D( ...
        'states_bound'       , states_bound, ...
        'params_bound'       , params_bound, ...
        'affinization_dt'    , affinization_dt, ...
        'affinization_N_grid', affinization_N_grid);

% create agent
A = NearHoverAgent ;
A.LLC = controller;

% create the initial condition
A.reset(s_0);

%% trajectory setup and tracking
% make the desired trajectory
[T_des, U_des, Z_des] = TPM.make_desired_trajectory(z_0, t_plan, ...
                        'vx_des', vx_des, 'vy_des', vy_des, 'vz_des', vz_des);

% track the desired trajectory
A.move(T_des(end),T_des,U_des,Z_des) ;

%% tracking error computation
% get the realized position trajectory
T = A.time ;

% interpolate the realized trajectory to match the braking traj timing
pos_real = A.state(A.position_indices,:) ;

% get the desired trajectory
pos_des = match_trajectories(T, T_des, Z_des(TPM.position_indices,:));

% compute the tracking error
pos_err = abs(pos_real - pos_des) ;
x_err = pos_err(1,:) ;
y_err = pos_err(2,:) ;
z_err = pos_err(3,:) ;

%% animate robot
if animate_robot
figure(1) ; clf ; axis equal ; hold on ; set(gca,'FontSize',15)

plot3(Z_des(1,:),Z_des(2,:),Z_des(3,:),'b--','LineWidth',1.5)
plot(A);
make_plot_pretty
legend('off'); grid on;
view(3);
A.animate();
end


%% plot tracking error
figure(2) ; clf ; hold on ;
plot(T,x_err,'--','Color',[0.5 0.2 0.2],'LineWidth',1.5)
plot(T,y_err,'--','Color',[0.2 0.5 0.2],'LineWidth',1.5)
plot(T,z_err,'--','Color',[0.2 0.2 0.5],'LineWidth',1.5)
title('tracking error in x, y, and z')
xlabel('time [s]')
ylabel('error [m]')
legend('x error','y error','z error')
set(gca,'FontSize',15)

%% plot robot
figure(3) ; clf ; hold on ; axis equal ; grid on ;
plot_path(pos_des,'b--','LineWidth',1.5)
plot_path(pos_real,'b','LineWidth',1.5)
plot(A)
legend('desired traj','realized traj')
title('robot showing tracking error')
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
set(gca,'FontSize',15)


