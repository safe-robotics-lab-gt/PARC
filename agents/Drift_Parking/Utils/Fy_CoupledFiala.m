function [Fy, Fy_max] = Fy_CoupledFiala(alpha, Fz, Fx, f_tire)

% Unpack parameters
mup = f_tire.mu;
Ca = f_tire.Cy;

% calculate derating parameter
inside = (mup*Fz)^2 - Fx^2;
Fy_max = sqrt(inside);
if inside < 0
    inside = 0;
end
zeta = sqrt(inside)/(mup*Fz);

% calculate sliding slip angle
asl = atan( 3 * zeta * mup * Fz / Ca );

% calculate lateral force
if ( abs(alpha) < asl )
    Fy = - Ca * tan( alpha ) ...
        + (Ca^2/(3*zeta*mup*Fz)) * abs(tan(alpha)) * tan(alpha) ...
        - (Ca^3/(27*zeta^2*mup^2*Fz^2)) * tan(alpha)^3;
else
    Fy = - zeta * mup * Fz * sign( alpha );
end
