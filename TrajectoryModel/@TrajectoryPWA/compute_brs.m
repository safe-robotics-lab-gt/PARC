function br_chains = compute_brs(obj, varargin)
%% Compute Backward Reachable Set using PARC
% Compute the chain of backward reachable sets from X_goal
% whose length is less than t_max`
% Input
%   lookup_points: cell(n_lookup_point, 1), n by 1 vector per each cell
%   X_goal
%   t_max: How much step backward reachable set would we compute? -> int
% Author: Edgar Chung
% Modified, refactored by Wonsuhk Jung

% Parse functions
kwargs = parse_function_args(varargin{:});
kwargs = check_sanity_and_set_default_kwargs(kwargs, ...
        'required_key', {"lookup_points", "X_goal", "t_max"});

lookup_points = kwargs.lookup_points;
X_goal        = kwargs.X_goal;
t_max         = kwargs.t_max;


X_goals(1, t_max+1) = Polyhedron;
X_goals(:) = X_goal;

% Construct Error Polyhedron here
% [X_goal, X_goal - E(1), ...., X_goal - E(t_max)]
if isfield(kwargs, "tracking_error")
    tracking_error = kwargs.tracking_error;
    error_polytopes = obj.get_error_polyhedrons('t_max', t_max, ...
        'time_axis', tracking_error.stamps, ...
        'error_function', tracking_error.errors_max, ...
        'mode', 'goal');
    for i = 1:t_max+1
        X_goals(1, i) = X_goals(1, i) - error_polytopes(i);
    end
end

% Compute BRS for each lookup point
lookup_n = length(lookup_points);

ap_uniques = {};
br_chains = {};

for i = 1:lookup_n
    ap_i = obj.retrieve_ap(lookup_points{i}, X_goals);
    is_duplicate = false;
    if ~isempty(ap_i)
        % Check if ap is unique
        % Note that the computed backward reachable set that follows the
        % activation sequence is exact.
        for j = 1:length(ap_uniques)
            if length(ap_uniques{j}) == length(ap_i) && all(ap_uniques{j} == ap_i)
                is_duplicate = true;
                break;
            end
        end

        if ~is_duplicate
            brs_nodes_i = obj.brs_from_ap(ap_i, X_goals(length(ap_i) + 1));
            br_chain_i = BackwardReachableChain(flip(ap_i), brs_nodes_i);
            br_chains{end+1} = br_chain_i;
            ap_uniques = [ap_uniques, {ap_i}];
        end
    end
end
end

