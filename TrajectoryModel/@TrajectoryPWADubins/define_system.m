function [x, dx] = define_system(obj)
% Define the dynamics with symbolic function
% User has to define the dynamics of trajectory producing model here.
% Author: Wonsuhk Jung
% Created: Jan 18th 2024

syms p_x p_y th k_w k_v

% Define the state
x = [p_x; p_y; th; k_w; k_v];

% Define the dynamics before affinization
dx = [k_v * cos(th); k_v * sin(th); k_w; 0; 0];

end