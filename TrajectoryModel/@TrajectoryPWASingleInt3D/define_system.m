function [x, dx] = define_system(obj)
% Define the dynamics with symbolic function
% User has to define the dynamics of trajectory producing model here.

syms p_x p_y p_z vx_des vy_des vz_des

% Define the state
x = [p_x; p_y; p_z; vx_des; vy_des; vz_des];

% Define the dynamics before affinization
dx = [vx_des; vy_des; vz_des; 0; 0; 0];

end