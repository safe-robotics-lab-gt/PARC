function [ux_dot, uy_dot, r_dot, V_dot, beta_dot, x_dot, y_dot, yaw_dot] = ...
            state_derivatives(Fxf, Fxr, Fyf, Fyr, ux, uy, r, V, beta, delta, veh, x, y, yaw)
% note that [ux, uy, r] and [r, V, beta] are two ways of discribing vehicle
% dynamics, however, while [r, V, beta] is better at parametrizing path
% tracking, it has to have a none zero linear velocity.

%Fx_drag = - veh.Cd0 - veh.Cd1*ux - veh.Cd2*ux^2; % drag force 
Fx_drag = 0;

            
ux_dot = (1/veh.m) * (Fxr-Fyf*sin(delta)+Fxf*cos(delta)+Fx_drag) + r*uy;
uy_dot = (1/veh.m) * (Fyf*cos(delta)+Fyr+Fxf*sin(delta)) - r*ux;
r_dot = (1/veh.Iz) * (veh.a*Fyf*cos(delta) +veh.a*Fxf*sin(delta) - veh.b*Fyr);

V_dot = (1/veh.m) * (Fxf*cos(delta-beta) - Fyf*sin(delta-beta)...
                     +Fxr*cos(beta) + Fyr*sin(beta));
beta_dot = (1/(veh.m*V)) * (Fxf*sin(delta-beta) + Fyf*cos(delta-beta)...
                        -Fxr*sin(beta) + Fyr*cos(beta)) - r;


x_dot = ux*cos(yaw) - uy*sin(yaw);
y_dot = ux*sin(yaw) + uy*cos(yaw);
yaw_dot = r;
end