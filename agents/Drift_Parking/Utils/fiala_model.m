function Fy = fiala_model(alpha, tire, Fx)
%   Calculate tire forces with the fiala model

Ca = tire.Ca;
mu = tire.mu;
Fz = tire.Fz;

zeta = sqrt((mu*Fz)^2 - Fx^2) / (mu*Fz);
alpha_sl = atan2(3*zeta*mu*Fz, Ca);

ta = tan(alpha);
if abs(alpha) < alpha_sl
    Fy = -Ca*ta + ...
            Ca^2 / (3*zeta*mu*Fz)*abs(ta)*ta - ...
            Ca^3 / (27*zeta^2*mu^2*Fz^2)*ta^3;
else
    Fy = -zeta * mu * Fz * sign(alpha);
end

end