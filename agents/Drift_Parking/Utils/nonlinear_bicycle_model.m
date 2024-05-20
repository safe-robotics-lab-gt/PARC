function [ ux_dot, uy_dot, r_dot, V_dot, beta_dot, alpha_f, alpha_r, x_dot, y_dot, yaw_dot,fxf] = ...
    nonlinear_bicycle_model(ux, uy, r, V, beta, Fxr, Fxf, delta, veh, tire_f, tire_r, x, y, yaw)
%NONLINEAR_BICYCLE_MODEL
%   Calculates the vehicle state derivatives using the bicycle model
%
%   Inputs
%       r:          Yaw rate [rad/s]
%       Ux:         Longitudinal velocity [m/s]
%       Uy:         Lateral velocity [m/s]
%       Fxr:        Longitudinal force [N]
%       Fxf:        Frontwheel brake force [N]
%       delta:      Steer angle [rad]
%       veh:        Vehicle parameters struct
%       tire_f:     Front tire parameter struct
%       tire_r:     Rear tire parameter struct
%
%   Output:
%       r_dot:      Derivative of yaw rate [rad/s^2]
%       Uy_dot:     Derivative of lateral velocity [m/s^2]
%       Ux_dot:     Derivative of longitudinal velocity [m/s^2]
%       alpha_f:    Front tire slip angle [rad]
%       alpha_r:    Rear tire slip angle [rad]


% slip angles
[alpha_f, alpha_r] = slip_angles(ux, uy, r, delta, veh);

%Split longitudinal force based on drive and brakedistribution
% front wheel brake dynamics
% since front tire is break, it's acting like a damper
% approximating this with a half logistic function
V_f_in_car_frame = [ux;uy+veh.a*r];
V_f_in_tire_dir = V_f_in_car_frame' * [cos(delta);sin(delta)];
% the maximum Fxf however is determained by the alpha_f, the angle between the 
% velocity vector and the orientation. at alpha_f = 0, Fxf_max = Fxf
% assume Fxf to be negative at all time 
Fxf_act = Fxf*cos(alpha_f)^2;
break_stiffness = 10;
fxf = -2*Fxf_act/(1+exp(break_stiffness*V_f_in_tire_dir))+Fxf_act;
fxr = Fxr;

% lateral tire forces
Fyf = fiala_tanh(alpha_f, tire_f, fxf);
Fyr = fiala_tanh(alpha_r, tire_r, fxr);

% dynamics
[ux_dot, uy_dot, r_dot, V_dot, beta_dot, x_dot, y_dot, yaw_dot] = ...
            state_derivatives(fxf, fxr, Fyf, Fyr, ux, uy, r, V, beta, delta, veh, x, y, yaw);

end
