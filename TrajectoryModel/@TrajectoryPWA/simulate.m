function history = simulate(obj, x_0, t_f)
% Get the trajectory of x_0 that travels through piecewise-affine system
% pwa without first computing activation pattern ap
% 
% Author: Long Kiu Chung
% Created: 2023/10/30
% Updated: 2023/10/30
%
% INPUTS:
% pwa: n-dimensional PWASystem from MPT3; PWA model of the system
% x_0: n*1 vector; initial condition of the system
% t_f: integer; number of timesteps to simulate
%
% OUTPUTS:
% history: n*(t_f + 1) matrix; each column represents the states at the
%          timestep

pwa = obj.system;

n = pwa.nx;
history = zeros(n, t_f + 1);
history(:, 1) = x_0;

x = x_0;
for i = 1:t_f
    for j = 1:pwa.ndyn
        if pwa.domain(j).contains(x)
            x = pwa.A{j}*x + pwa.f{j};
            history(:, i + 1) = x;
            break
        elseif j == pwa.ndyn % next_step out of bounds
            error_msg = sprintf("%.2f ", x);
            error_msg = strcat("Trajectory Out of Bound:", error_msg);
            disp(error_msg);
            
            history = history(:, 1:i);
            return
        end
    end
end