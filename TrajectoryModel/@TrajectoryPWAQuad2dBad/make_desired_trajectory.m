function [T,U,X] = make_desired_trajectory(obj, x0, t_f, varargin)
% [T,U,X] = make_desired_trajectory_pwa_turtlebot(t_f,w_des,v_des)
%
% Create a Piecewise-affine path as a full-state trajectory for the TurtleBot.
%
% The inputs are:
%   x_0      initial state in low-fidelity space
%   t_f      planning time horizon
%   w_des    desired yaw rate
%   v_des    desired speed
%
% The outputs are:
%   T        timing for desired trajectory as a 1-by-N array
%   U        desired input (yaw rate and acceleration) as 2-by-N array
%   X        desired trajectory (x,y,h,v) as a 4-by-N array
%
% Author: Wonsuhk Jung
% Created: 20 Oct 2023

kwargs = parse_function_args(varargin{:});
kwargs = check_sanity_and_set_default_kwargs(kwargs, ...
    'required_key', ["F1", "F2"]);

F1 = kwargs.F1;
F2 = kwargs.F2;

dt = obj.affinization_dt;

% set up timing
T = unique([0:dt:t_f,t_f],'stable');
N_t = length(T) ;



% compute desired trajectory
if isfield(kwargs, "ap")
    ap = kwargs.ap;
    X = get_trajectory(ap, obj.system, [x0; F1; F2]);
else
    X = obj.simulate([x0; F1; F2], N_t-1);
end
% append velocity to (x,y,h) trajectory to make it a full-state
% trajectory for the turtlebot

% get inputs for desired trajectories
F1_traj = F1*ones(1,size(X,2)) ;
F2_traj = F2*ones(1,size(X,2)) ;

X = X(obj.state_indices, :);
T = T(:, 1:size(X, 2));
% compute inputs for robot
U = [F1_traj ; F2_traj] ;
end

