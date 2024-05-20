function [alpha_slip_f, alpha_slip_r] = ...
    slip_angle_bound(tire_f, Fxf,tire_r, Fxr)
% calculate the boundary of slipping angle
zeta_f = sqrt((tire_f.mu*tire_f.Fz)^2 - Fxf^2) / (tire_f.mu*tire_f.Fz);
alpha_slip_f = atan2(3*zeta_f*tire_f.mu*tire_f.Fz, tire_f.Ca);

zeta_r = sqrt((tire_r.mu*tire_r.Fz)^2 - Fxr^2) / (tire_r.mu*tire_r.Fz);
alpha_slip_r = atan2(3*zeta_r*tire_r.mu*tire_r.Fz, tire_r.Ca);
end
