function [alpha_f, alpha_r] = slip_angles(Ux, Uy, r, delta, veh)
%SLIP_ANGLES 
%   Calculates the front and rear slip angles using the dynamic bicycle model. This function assumes 
%   small angles

%   Inputs
%       Ux:         Longitudinal velocity [m/s]
%       Uy:         Lateral velocity [m/s]
%       r:          Yaw rate [rad/s]
%       delta:      Steer angle [rad]
%       veh:        Vehicle parameters struct
%
%   Output:
%       alpha_f:    Front slip angle [rad]
%       alpha_r:    Rear slip angle [rad]

% Front slip angle
alpha_f = atan2(Uy + veh.a*r, Ux) - delta;

% Rear slip angle
alpha_r = atan2(Uy - veh.b*r, Ux);

end
