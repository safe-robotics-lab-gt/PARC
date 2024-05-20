% ME227 Vehicle Dynamics & Control
% Homework 5: Getting Sideways
% % code template
% Spring 2020
% Prof. Chris Gerdes & CAs Peter Schleede, John Talbot, previous CAs

clear all;
close all;
clc;

%genCostConstraintFunc;
initialization;

% simulation time
t_final = 10;
dT      = 0.01;
t_s     = 0:dT:t_final;


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



%Control parameters
V_des = 13;
r_des = -1.3; %1.19 %-0.6
beta_des = -sign(r_des)*deg2rad(35);%40

Ux_des = V_des*cos(beta_des); %8
Uy_des = V_des*sin(beta_des); % -4.34

Fx_ff = 6684;
Kx = 2000;
Kx_break = 2000;
delta_ff = deg2rad(sign(beta_des)*0); % 10
Kr = 0.4;
Ky = -0.5;
Kbeta = 1;

% guess initial input condition
delta = 64.11*pi/180;              % steering % pi/6 %0.4253
Fxf = -1000;     % breaking % -0.9*Fxf_max
Fxr = 4175.55;      % gas

% initial path condition same as the reference
% r_radps(1)  = r_des; % 1.19
% uy_mps(1)   = Uy_des; % -4.34
% ux_mps(1)   = Ux_des; % 8
% beta_rad(1) = beta_des;
% V_mps(1) = V_des;
% x_m(1) = 0;
% y_m(1) = 0;
% yaw_rad(1) = 0;

% r_radps(1)  = -1.35; % 1.19
% uy_mps(1)   = 6.5371; % -4.34
% ux_mps(1)   = 12.3128; % 8
% beta_rad(1) = 0.48;
% V_mps(1) = 13.92;
% x_m(1) = 0;
% y_m(1) = 0;
% yaw_rad(1) = 0;

% initial path condition same as the reference

r_radps(1)  = 0; % 1.19
uy_mps(1)   = 0; % -4.34
ux_mps(1)   = 0; % 8
beta_rad(1) = atan2(uy_mps(1),ux_mps(1));
V_mps(1) = norm([ux_mps(1),uy_mps(1)]);
x_m(1) = 0;
y_m(1) = 0;
yaw_rad(1) = 0;

V_fTire_mps = zeros(N,1);
r_dot_radpss = zeros(N,1);
V_dot_mpss = zeros(N,1);
beta_dot_radps = zeros(N,1);



for idx = 1:N
    r = r_radps(idx);
    uy = uy_mps(idx);
    ux = ux_mps(idx);
    beta = beta_rad(idx);
    V = V_mps(idx);
    x = x_m(idx);
    y = y_m(idx);
    yaw = yaw_rad(idx);
    
    %%%% STUDENT CODE HERE %%%%
    % Fxr = Kx*(Ux_des-ux)+Fx_ff;
    % delta = Kr*(r_des-r);
    % Fxf = 0;
    Fxr = 5000; %5
    delta = -delta_max/5;
    Fxf = 0;
    %%%% END STUDENT CODE %%%%
    V_fTire = 0;
    % if idx > 1% && idx < N/2
    %     % V_des = 2;
    %     % beta_des=-sign(r_des)*deg2rad(90-1);
    %     % Ux_des = V_des*cos(beta_des); %8
    %     % Uy_des = V_des*sin(beta_des); % -4.34
    %     % delta = Kr*(r_des-r)+Ky*(Uy_des-uy)+delta_ff;
    %     %beta_des = -sign(r)*deg2rad(90);
    %     %delta = -40;%Kbeta*(beta_des-beta);
    %     yaw = wrapTo2Pi(yaw);
    %     V_global = [ux*cos(yaw)-uy*sin(yaw);ux*sin(yaw)+uy*cos(yaw)];
    %     V_front = V_global+r*veh.a*[-sin(yaw);cos(yaw)]; % veh front vel in global frame
    %     e_front = [cos(yaw+delta);sin(yaw+delta)];
    %     V_fTire = V_front' * e_front;
    %     Fxf = -sign(ux)*Kx_break*(abs(V_fTire)); % along the front wheel dir
    %     if uy <= 0.1
    %         Fxr = 0;
    %     end
    % end
    V_fTire_mps(idx) = V_fTire;
    
    delta = bound_values(delta, -delta_max, delta_max);
    Fxr = bound_values(Fxr, 0, Fxr_max);
    Fxf = bound_values(Fxf, -Fxf_max/1.5, 100);%1.5


    
    [ ux_dot, uy_dot, r_dot, V_dot, beta_dot, alpha_f, alpha_r, x_dot, y_dot, yaw_dot] = ...
    nonlinear_bicycle_model(ux, uy, r, V, beta, Fxr, Fxf, delta, veh, tire_f, tire_r, x, y, yaw);
    
    [alpha_slip_f, alpha_slip_r] = slip_angle_bound(tire_f, Fxf,tire_r, Fxr);
    
    if idx < N
        r_radps(idx+1) = integrate_euler(r, r_dot, dT);
        uy_mps(idx+1) = integrate_euler(uy, uy_dot, dT);
        ux_mps(idx+1) = integrate_euler(ux, ux_dot, dT);
        beta_rad(idx+1) = integrate_euler(beta, beta_dot, dT);
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
end

slip_f_bool = abs(alphaF_rad) > alphaF_slip_rad;
slip_r_bool = abs(alphaR_rad) > alphaR_slip_rad;

animateDrift(ux_mps, uy_mps, r_radps, x_m, y_m, yaw_rad, slip_f_bool, slip_r_bool, delta_rad, veh, dT)

figure; hold on;
plot(t_s, ux_mps, 'r', 'LineWidth', 2);
plot(t_s, uy_mps, 'b', 'LineWidth', 2);
plot(t_s, r_radps, 'Color', [0 0.6 0], 'LineWidth', 2);
ylim([-8 15]);
grid on; box on;
title('Vehicle States over Time')
xlabel('Time, s');
ylabel('Vehicle states');
legend('Ux, m/s', 'Uy, m/s', 'r, rad/s', 'Location', 'SouthEast');
hold off;

figure(2)
subplot(3,1,1)
plot(t_s, rad2deg(delta_rad))
grid on
ylabel('delta [deg]')
title('Actuator Commands over Time')

subplot(3,1,2)
plot(t_s, Fxr_N)
grid on
ylabel('Fxr [N]')
xlabel('Time [s]')

subplot(3,1,3)
plot(t_s, Fxf_N)
grid on
ylabel('Fxf [N]')
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

figure(4)
subplot(3,1,1)
plot(t_s, rad2deg(beta_rad), 'r', 'LineWidth', 2);
grid on
ylabel('beta, deg');
ylim([-100,100])
subplot(3,1,2)
plot(t_s, V_mps, 'b', 'LineWidth', 2);
grid on
ylabel('V, m/s');
ylim([0,17])
subplot(3,1,3)
plot(t_s, r_radps, 'Color', [0 0.6 0], 'LineWidth', 2);
grid on
xlabel('Time, s');
ylabel('r, rad/s');

figure; hold on;
plot(t_s, r_dot_radpss, 'r', 'LineWidth', 2);
plot(t_s, V_dot_mpss, 'b', 'LineWidth', 2);
plot(t_s, beta_dot_radps, 'Color', [0 0.6 0], 'LineWidth', 2);
grid on; box on;
title('Vehicle States over Time')
xlabel('Time, s');
ylabel('Vehicle states');
legend('r_dot, m/s^2', 'V_dot, m/s^2', 'beta_dot, rad/s', 'Location', 'best');
hold off;

