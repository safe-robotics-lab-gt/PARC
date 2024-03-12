function history = get_trajectory(ap, pwa, x_0)
% Get the trajectory of x_0 that travels through piecewise-affine system
% pwa with the activation pattern ap
% 
% Author: Long Kiu Chung
% Created: 2023/10/27
% Updated: 2023/10/27
%
% INPUTS:
% ap: 1*t_m vector; activation pattern of lookup_point, i.e. lookup_point
%     reaches X_goal in t_m timestep for pwa
%     Assumes ap is non-empty
% pwa: n-dimensional PWASystem from MPT3; PWA model of the system
% x_0: n*1 vector; initial condition of the system
%
% OUTPUTS:
% history: n*(t_m + 1) matrix; each column represents the states at the
%          timestep

    t_m = length(ap);
    n = pwa.nx;
    history = zeros(n, t_m + 1);
    
    history(:, 1) = x_0;
    x = x_0;
    for i = 1:t_m
        idx = ap(i);
        x = pwa.A{idx}*x + pwa.f{idx};
        history(:, i + 1) = x;
    end