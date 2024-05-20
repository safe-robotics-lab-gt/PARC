function [r_dot, V_dot, beta_dot, x_dot, y_dot, yaw_dot] = ...
            state_derivatives_beta(Fxf, Fxr, Fyf, Fyr,r, V, beta, delta, veh, x, y, yaw)
  
r_dot = (1/veh.Iz) * (veh.a*(Fxf*sin(delta) + Fyf*cos(delta))...
                        - veh.b*Fyr);
V_dot = (1/veh.m) * (Fxf*cos(delta-beta) - Fyf*sin(delta-beta)...
                     +Fxr*cos(beta) + Fyr*sin(beta));
beta_dot = (1/(veh.m*V)) * (Fxf*sin(delta-beta) + Fyf*cos(delta-beta)...
                        -Fxr*sin(beta) + Fyr*cos(beta)) - r;
phi = yaw + beta;  % phi_dot = r + beta_dot -> equation (1)
x_dot = V*cos(phi);
y_dot = V*sin(phi);
yaw_dot = r;
end