function [x, dx] = define_system(obj)
% Define the dynamics with symbolic function
% User has to define the dynamics of trajectory producing model here.
% Author: Wonsuhk Jung
% Created: Jan 18th 2024

drone.r = 0.25; % m
drone.m = 1.0; % kg
drone.I = 0.01; % kg*m^2
drone.g = 9.81; 

syms p_x p_z th v_x v_z th_d F1 F2

% Define the state
x = [p_x; p_z; th; v_x; v_z; th_d; F1; F2];

% Define the dynamics before affinization
dx = [v_x;
      v_z;
      th_d;
      1/drone.m*sin(th)*(F1+F2);
      1/drone.m*cos(th)*(F1+F2)-drone.g;
      drone.r/drone.I*(F1-F2);
      0;
      0];

end