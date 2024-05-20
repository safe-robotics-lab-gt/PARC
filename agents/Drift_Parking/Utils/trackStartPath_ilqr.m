% track given trajectory [x,y,V] using iLQR
clear all;
close all;
clc;
initialization;
%genCostConstraintFunc;

% % simulation time
% t_final = 3;
% dT      = 0.01;
% t_s     = 0:dT:t_final;
% S = genStartPath(t_s, -15, 7);

% new traj
dT = 0.01;
V_des = 7; % m/s 
beta_des = 30; % degree
S = genStartPath(V_des, beta_des, dT);
t_s = S.t_s;

% Set the mpc horizon
t_horizon = 0.5; % seconds
horizon = round(t_horizon / dT);
% Set max numebr of iterations
n_iterations = 100;

% Set desired state
n_states = 4;
n_inputs = 2;
init_state = [S.x(1);S.y(1);S.yaw(1);S.V(1)]; % Define the initial state to be the origin with no velocity and right heading
target_states = [S.x,S.y,S.yaw, S.V];%V_des*ones(length(t_s),1)]; % Get to 3 meters to the right facing up and stop [N,4] tall matrix
% Set initial guess for input
initial_guess = [S.V_dot,S.delta];

t_final = t_s(end);

% allocate space for simulation data
N = length(t_s);
r_radps     = zeros(N,1);
uy_mps      = zeros(N,1);
ux_mps      = zeros(N,1);
beta_rad    = zeros(N,1);
V_mps       = zeros(N,1);
delta_rad   = zeros(N,1);
alphaF_rad  = zeros(N,1);
alphaR_rad  = zeros(N,1);
alphaF_slip_rad = zeros(N,1);
alphaR_slip_rad = zeros(N,1);
Fxf_N       = zeros(N,1);
Fxr_N       = zeros(N,1);
Fyf_N       = zeros(N,1);
Fyr_N       = zeros(N,1);
x_m         = zeros(N,1);
y_m         = zeros(N,1);
yaw_rad     = zeros(N,1);

% % Set desired state
% n_states = 4;
% n_inputs = 2;
% init_state = [S.x(1);S.y(1);S.phi(1); 1]; % Define the initial state to be the origin with no velocity and right heading
% target_states = [S.x;S.y;S.phi; S.V]'; % Get to 3 meters to the right facing up and stop
% 
% % Set initial guess for input
% initial_guess = [zeros(size(t_s,2),1),zeros(size(t_s,2),1)];



% Define weighting matrices
Q_k = 0.5*eye(n_states); % zero weight to penalties along a strajectory since we are finding a trajectory
% Q_k(3,3) = 10;
R_k = 0.1*eye(n_inputs);
% R_k(2,2) = 50;


Q_T = 100*eye(n_states);
% Q_T(3,3) = 1000;
% Q_T(1,1) = 10;
% Q_T(2,2) = 10;
% Q_T(3,3) = 10;

% There are no physical parameters to adjust
parameters = [];

% Specify max number of iterations
n_iterations = 1000;

% Construct ilqr object
ilqr_ = ilqr(init_state,target_states,initial_guess,dT,0,t_final,@bike_f_disc,@bike_A_disc,@bike_B_disc,Q_k,R_k,Q_T,parameters,n_iterations);
% Solve
[states,inputs,k_feedforward,K_feedback,final_cost] = ilqr_.solve();
%%
x_m_plan = S.x';
y_m_plan = S.y';
yaw_rad_plan = S.yaw';
V_mps_plan = S.V';

x_m_pred = states(:,1);
y_m_pred = states(:,2);
yaw_rad_pred = states(:,3);
V_mps_pred = states(:,4);


V_dot_mpss_pred = inputs(:,1);
delta_rad = inputs(:,2);

% control parameters
K_vd = 2;

% initial path condition
x_m(1) = init_state(1);
y_m(1) = init_state(2);
yaw_rad(1) = init_state(3);
V_mps(1) = init_state(4);
beta_rad(1) = 0;
r_radps(1)  = 0; % 1.19
uy_mps(1)   = V_mps(1)*sin(beta_rad(1)); % -4.34
ux_mps(1)   = V_mps(1)*cos(beta_rad(1)); % 8


V_fTire_mps = zeros(N,1);
r_dot_radpss = zeros(N,1);
beta_dot_radps = zeros(N,1);
V_dot_mpss  = zeros(N,1);

V_prev = S.V(1);


for idx = 1:N
    r = r_radps(idx);
    uy = uy_mps(idx);
    ux = ux_mps(idx);
    beta = beta_rad(idx);
    V = V_mps(idx);
    x = x_m(idx);
    y = y_m(idx);
    yaw = yaw_rad(idx);

     %%%% iLQR controller CODE HERE %%%%
    V_dot_curr = (V-V_prev)/dT;
    Fxr = K_vd*(V_dot_mpss_pred(idx) - V_dot_curr)*veh.m;
    delta = delta_rad(idx);
    Fxf = 0;
    %%%% END STUDENT CODE %%%%

    delta = bound_values(delta, -delta_max, delta_max);
    Fxr = bound_values(Fxr, 0, Fxr_max);
    Fxf = bound_values(Fxf, -Fxf_max/1.5, 100);

    [ ux_dot, uy_dot, r_dot, V_dot, beta_dot, alpha_f, alpha_r, x_dot, y_dot, yaw_dot] = ...
    nonlinear_bicycle_model(ux, uy, r, V, beta, Fxr, Fxf, delta, veh, tire_f, tire_r, x, y, yaw);
    
    [alpha_slip_f, alpha_slip_r] = slip_angle_bound(tire_f, Fxf,tire_r, Fxr);
    
    if idx < N
        r_radps(idx+1) = integrate_euler(r, r_dot, dT);
        uy_mps(idx+1) = integrate_euler(uy, uy_dot, dT);
        ux_mps(idx+1) = integrate_euler(ux, ux_dot, dT);
        beta_rad(idx+1) = atan2(uy_mps(idx+1), ux_mps(idx+1));
        V_mps(idx+1) = integrate_euler(V, V_dot, dT);
        x_m(idx+1) = integrate_euler(x, x_dot, dT);
        y_m(idx+1) = integrate_euler(y, y_dot, dT);
        yaw_rad(idx+1) = integrate_euler(yaw, yaw_dot, dT);
    end
    delta_rad(idx) = delta;
    Fxr_N(idx) = Fxr;
    Fxf_N(idx) = Fxf;
    alphaF_rad(idx) = alpha_f;
    alphaR_rad(idx) = alpha_r;
    alphaF_slip_rad(idx) = alpha_slip_f;
    alphaR_slip_rad(idx) = alpha_slip_r;
    r_dot_radpss(idx) = r_dot;
    V_dot_mpss(idx) = V_dot;
    beta_dot_radps(idx) = beta_dot;
    V_prev = V;
end

slip_f_bool = abs(alphaF_rad) > alphaF_slip_rad;
slip_r_bool = abs(alphaR_rad) > alphaR_slip_rad;

f = animateDrift(ux_mps, uy_mps, r_radps, x_m, y_m, yaw_rad, slip_f_bool, slip_r_bool, delta_rad, veh, dT);
pld=plot(x_m_plan,y_m_plan,'b--','DisplayName','planned');
epd=plot(x_m_pred,y_m_pred,'k--','DisplayName','iLQR expected');
hold off
legend([pld,epd])

figure(1); 
grid on; box on;
plot(t_s, ux_mps, 'r', 'LineWidth', 2);
hold on;
plot(t_s, V_mps_plan, 'b--', 'LineWidth', 3);
plot(t_s, V_mps_pred(2:end), 'k--', 'LineWidth', 3);
title('Vehicle velocity over Time')
xlabel('Time, s');
legend('Ux, m/s', 'v_{planed}, m/s', 'v_{pred}, m/s', 'Location', 'southeast');
hold off;

figure(2)
subplot(3,1,1)
plot(t_s, rad2deg(delta_rad))
grid on
ylabel('delta [deg]')
title('Actuator Commands over Time')

subplot(3,1,2)
plot(t_s, V_dot_mpss)
hold on 
plot(t_s, V_dot_mpss_pred,'b--')
hold off
grid on
ylabel('V_dot [m/s^2]')
xlabel('Time [s]')

subplot(3,1,3)
plot(t_s, Fxr_N)
grid on
ylabel('Fxr [N]')
xlabel('Time [s]')

figure(3)
subplot(2,1,1)
title('Vehicle States over Time')
plot(t_s, abs(alphaF_rad),'LineWidth',1.5)
hold on 
plot(t_s, alphaF_slip_rad,'LineWidth',1.5)
hold off
legend('\alpha_f (rad)', '\alpha_f_{slip}')

subplot(2,1,2)
plot(t_s, abs(alphaR_rad),'LineWidth',1.5)
hold on
plot(t_s, alphaR_slip_rad,'LineWidth',1.5)
hold off
legend('\alpha_r (rad)', '\alpha_r_{slip}')
