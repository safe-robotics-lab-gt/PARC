function obstacle_funnel_vec = get_obstacle_funnels(obj, varargin)
% Get obstacle funnels using custom affine avoid-set computation method
%
% Input
%   obstacles        : array of Polyhedron -- dimension of workspace
%   t_max            : maximum backward reachable step
%   tracking_error   : tracking_error data, its time horizon should be longer
%                      than t_max * obj.affinization_dt
% Output
%   obstacle_funnels : cell array of obstacle funnel (1, N_obstacle)
%       obstacle_funnel is (t_max+1) * N_pwa cell array where each element
%       indicates the avoid set under affine dynamics
%
% Author: Edgar Chung, Wonsuhk Jung
% Created: Jan 18 2023

kwargs = parse_function_args(varargin{:});
kwargs = check_sanity_and_set_default_kwargs(kwargs, ...
    "required_key", {"obstacles", "t_max"}, ...
    "default_key", {"verbose"}, ...
    "default_value", {0});

obstacles      = kwargs.obstacles;
brs_step_max   = kwargs.t_max;
verbose        = kwargs.verbose;

% Parse error polyhedrons from tracking error. If not provided, assume that
% tracking error is zero.
workspace_dim  = obstacles(1).Dim;
if isfield(kwargs, "tracking_error")
    error_data = kwargs.tracking_error;
    E_obs = obj.get_error_polyhedrons('t_max', brs_step_max, ...
                    'time_axis', error_data.stamps, ...
                    'error_function', error_data.errors_max, ...
                    'mode', 'obstacle');
    E_obs = E_obs.projection(1:workspace_dim);
else
    E_obs_i = Polyhedron('A', [eye(workspace_dim); -eye(workspace_dim)], ...
                         'b', zeros(workspace_dim*2, 1));
    E_obs   = repmat(E_obs_i, 1, brs_step_max+1);
end

N_error      = length(E_obs);
N_obstacle   = length(obstacles);
N_pwa_region = obj.system.ndyn;

% unbounded 1D interval

% dimension that is in avoidset dimension, but not in workspace
aug_dim = setdiff(obj.avoidset_indices, obj.position_indices);
aug_polytope = obj.get_bound_polytopes_by_dims(aug_dim);

% dimension that is not in avoidset dimension
z_dim   = setdiff(1:obj.n_state+obj.n_param, obj.avoidset_indices);
N_z     = length(z_dim);
is_avoidset_full_dim = (N_z == 0);
if ~is_avoidset_full_dim
    z_polytope    = obj.get_bound_polytopes_by_dims(z_dim);    
    Rn_polyhedron = Polyhedron("A", zeros(1, N_z), "b", 0);
end

obstacle_funnel_vec = cell(1, N_obstacle);

% for every obstacle
for i = 1:N_obstacle
    if verbose, fprintf("Obstacle funnel for the obstacle %d\n", i); end
    
    obstacle_funnel = cell(N_error, N_pwa_region);
    obstacle_i = obstacles(i);

    for j = 1:N_error
        % Pad obstacle with the tracking error
        obstacle_ij = obstacle_i + E_obs(j);
        obstacle_x  = obstacle_ij * aug_polytope; % obstacle is defined with augmented dimension

        if is_avoidset_full_dim
            obstacle_xz = obstacle_x;
        else
            obstacle_xz = cartesian_product_with_axis(obstacle_x, Rn_polyhedron, ...
                                                obj.avoidset_indices, z_dim);
        end

        for k = 1:N_pwa_region
            %% Implement custom avoid set method for affine system here.
            % this method implements the z-agnostic convex hull method
            % the method can inherit this by implementing the function
            % compute_affine_avoid_set(obstacle, mode = ap)
    
            % (Obs + error) \times R \intersection pwa
            obs_brss = obj.brs_from_ap(k, obstacle_xz);
            
            % proj x {(Obs + error) \times R \intersection pwa}
            obs_brs = obs_brss(end).projection(obj.avoidset_indices);
            
            funnel_jk = PolyUnion('Set', [obs_brs, obstacle_x]).convexHull;

            if ~is_avoidset_full_dim
                funnel_jk = cartesian_product_with_axis(funnel_jk, z_polytope, ...
                                        obj.avoidset_indices, z_dim);
            end

            funnel_jk = minHRep(funnel_jk);
        
            obstacle_funnel{j, k} = funnel_jk;
        end
    end

    obstacle_funnel_vec{i} = obstacle_funnel;
end
end

