function [T,U,X] = make_desired_trajectory(obj, x0, t_f, varargin)
% [T,U,X] = make_desired_trajectory_pwa_turtlebot(t_f,w_des,v_des)
%
% Create a Piecewise-affine trajectory with 3D single integrator as
% trajectory-producing model
%
% The inputs are:
%   x_0      initial state in low-fidelity space
%   t_f      planning time horizon
%   vx_des   desired velocity in x-direction
%   vy_des   desired velocity in y-direction
%   vz_des   desired velocity in y-direction
%
% The outputs are:
%   T        timing for desired trajectory as a 1-by-N array
%   U        desired input (yaw rate and acceleration) as 2-by-N array
%   X        desired trajectory (x,y,h,v) as a 4-by-N array
%
% Author: Wonsuhk Jung
% Created: Jan 18th 2024

kwargs = parse_function_args(varargin{:});
kwargs = check_sanity_and_set_default_kwargs(kwargs, ...
    'required_key', ["vx_des", "vy_des", "vz_des"]);

vx_des = kwargs.vx_des;
vy_des = kwargs.vy_des;
vz_des = kwargs.vz_des;

dt = obj.affinization_dt;

% set up timing
T = unique([0:dt:t_f,t_f],'stable');
N_T = length(T) ;

% compute desired trajectory
if isfield(kwargs, "ap")
    ap = kwargs.ap;
    X = get_trajectory(ap, obj.system, [x0; vx_des; vy_des; vz_des]);
else
    X = obj.simulate([x0; vx_des; vy_des; vz_des], N_T-1);
end
% append velocity to (x,y,h) trajectory to make it a full-state
% trajectory for the turtlebot
T = T(:, 1:size(X, 2));
U = diff(X(obj.param_indices, :), 1, 2)/dt;
U = [U, zeros(length(obj.param_indices), 1)];
end