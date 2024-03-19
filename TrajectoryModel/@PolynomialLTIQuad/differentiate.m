function [v_x, a_x, j_x, s_x] = differentiate(obj, k_v, k_a, k_pk)
% Approximate velocity, acceleration, jerk, and snap by differentiating
% through the Mueller polynomial model
% See Mueller et al. "A computationally efficient motion primitive for
% quadrocopter trajectory generation" for details of the polynomial
% planning model
% Author: Long Kiu Chung
% Created: 2024/02/05
% Updated: 2024/03/18

t_n = obj.step_number;
t_pk = obj.peak_time;
dt = obj.timestep;
t_f = obj.final_time;

v_x = zeros(1, t_n + 1);
a_x = zeros(1, t_n + 1);
j_x = zeros(1, t_n + 1);
s_x = zeros(1, t_n + 1);

c_1 = (12./(t_pk.^3)).*k_v + (6./(t_pk.^2)).*k_a - (12./(t_pk.^3)).*k_pk;
c_2 = (-6./(t_pk.^2)).*k_v - (4./(t_pk)).*k_a + (6./(t_pk.^2)).*k_pk;
c_3 = (12./((t_f - t_pk).^3)).*k_pk;
c_4 = (-6./((t_f - t_pk).^2)).*k_pk;

for i = 1:(t_n + 1)
    t = dt.*(i - 1);
    if t < t_pk
        v_x(i) = (c_1./6).*(t.^3) + (c_2./2).*(t.^2) + k_a.*t + k_v;
        a_x(i) = (c_1./2).*(t.^2) + c_2.*t + k_a;
        j_x(i) = c_1.*t + c_2;
        s_x(i) = c_1;
    else
        v_x(i) = (c_3./6).*((t - t_pk).^3) + (c_4./2).*((t - t_pk).^2) + k_pk;
        a_x(i) = (c_3./2).*((t - t_pk).^2) + c_4.*(t - t_pk);
        j_x(i) = c_3.*(t - t_pk);
        s_x(i) = c_3;
    end
end

end

