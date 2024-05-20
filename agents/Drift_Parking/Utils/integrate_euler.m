function x1 = integrate_euler( x0, x0_dot, dt )
%INTEGRATE_EULER
%   simple zero-hold integration scheme to compute discrete next state

%   Inputs
%       x0:         Initial state
%       x0_dot:     State derivative at initial time
%       dt:         Euler time step
%
%   Output:
%       x1:         State at next time step

%%%%% STUDENT CODE HERE %%%%%
% Take euler step

x1 = x0 + x0_dot*dt;

%%%%% END STUDENT CODE %%%%%
end
