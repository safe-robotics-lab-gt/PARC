function history = simulate(obj, x_0)
% Get the trajectory of x_0 that travels through the LTI system obj without
% computing the mode sequence
% 
% Author: Long Kiu Chung
% Created: 2023/10/30
% Updated: 2024/03/17
%
% INPUTS:
% obj: @PolynomialLTIQuad object; n-dimensional LTI model of the polynomial
%      quadrotor
% x_0: n*1 vector; initial condition of the system
% t_n: integer; number of timesteps to simulate
%
% OUTPUTS:
% history: n*(t_n + 1) matrix; each column represents the states at the
%          timestep

ltis = obj.systems;
n = ltis(1).nx;
t_n = obj.step_number;
history = zeros(n, t_n + 1);
history(:, 1) = x_0;

x_k = x_0;
for t = 1:t_n
    lti = ltis(t);
    x_k = lti.A*x_k;
    history(:, t + 1) = x_k;
end
end

