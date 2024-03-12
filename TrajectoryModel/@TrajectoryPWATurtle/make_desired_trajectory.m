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
    'required_key', ["w_des", "v_des"]);

w_des = kwargs.w_des;
v_des = kwargs.v_des;

dt = obj.affinization_dt;

% set up timing
T = unique([0:dt:t_f,t_f],'stable');
N_t = length(T) ;


% compute desired trajectory
if isfield(kwargs, "ap")
    ap = kwargs.ap;
    X = get_trajectory(ap, obj.system, [x0; w_des; v_des]);
else
    X = obj.simulate([x0; w_des; v_des], length(T)-1);
end
% append velocity to (x,y,h) trajectory to make it a full-state
% trajectory for the turtlebot
% get inputs for desired trajectories
w_traj = w_des*ones(1,size(X,2)) ;
v_traj = v_des*ones(1,size(X,2)) ;

X = [X(obj.state_indices, :); v_traj] ;

% compute inputs for robot
a_traj = [diff(v_traj)./dt, 0] ;
U = [w_traj ; a_traj] ;
end