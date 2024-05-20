function Fy = fiala_tanh(alpha, tire, Fx)
%   Calculate tire forces with the fiala model
% solve the discountinouity issue by using tanh function

Ca = tire.Ca;
mu = tire.mu;
Fz = tire.Fz;

zeta = sqrt((mu*Fz)^2 - Fx^2) / (mu*Fz);
alpha_sl = atan2(3*zeta*mu*Fz, Ca); 
% since 3*zeta*mu*Fz < pi/2, we know tan(alpha) and alpha has the same sign

sharpness = 100;
ta = tan(alpha);
% stitch alpha < 0
slip_fy = zeta * mu * Fz; 
no_slip_fy = -Ca*ta + ...
            Ca^2 / (3*zeta*mu*Fz)*abs(ta)*ta - ...
            Ca^3 / (27*zeta^2*mu^2*Fz^2)*ta^3;
s1 = 0.5 - 0.5*tanh(sharpness*(alpha + alpha_sl));
fiala_lhs = s1*slip_fy + (1-s1)*no_slip_fy;

% stitch alpha > 0
s2 = 0.5 - 0.5*tanh(sharpness*(alpha - alpha_sl));
Fy = s2*fiala_lhs + (1-s2)*(-slip_fy);
end