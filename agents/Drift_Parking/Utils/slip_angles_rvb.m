function [alpha_f, alpha_r] = slip_angles_rvb(r, V, beta, delta, veh)
%SLIP_ANGLES 
%   Calculates the front and rear slip angles using the dynamic bicycle model. This function assumes 
%   small angles

%   Inputs
%       V:          Center of Mass velocity [m/s]
%       beta:       sideslip angle [m/s]
%       r:          Yaw rate [rad/s]
%       delta:      Steer angle [rad]
%       veh:        Vehicle parameters struct
%
%   Output:
%       alpha_f:    Front slip angle [rad]
%       alpha_r:    Rear slip angle [rad]

% Front slip angle
alpha_f = atan2(V*sin(beta) + veh.a*r, V*cos(beta)) - delta;

% Rear slip angle
alpha_r = atan2(V*sin(beta) - veh.b*r, V*cos(beta));

end
