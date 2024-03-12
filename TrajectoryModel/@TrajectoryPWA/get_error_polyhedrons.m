function error_polyhedrons = get_error_polyhedrons(obj, varargin)
% Return the array of error polyhedrons given the error function
% Input
%   time_axis: time axis of error function (1, n_stamp)
%   error_function: maximum error each time (1, n_stamp)
%   t_max: N-step reachable - Integer
% Output
%   error_polyhedrons: polyhedrons that bound maximum tracking error,
%    polyhedrons length of t_max + 1, the first error polyhedron is empty
%
% Note: We assume the tracking error dimension is same with obj.pos_index
% Author: Wonsuhk Jung
% Created: Dec 11 2023

kwargs = parse_function_args(varargin{:});
kwargs = check_sanity_and_set_default_kwargs(kwargs, ...
    'required_key', {"t_max", "time_axis", "error_function"}, ...
    'default_key', {'mode'}, ...
    'default_value', {'goal'});

t_max = kwargs.t_max;
time_axis = kwargs.time_axis;
error_fn = kwargs.error_function;
mode = kwargs.mode;

error_polyhedrons = [];

stamps = (0:t_max)*obj.affinization_dt;
n_augmented = obj.n_state + obj.n_param;

for i = 1:length(stamps)
    switch(mode)
        case "goal"
            t = stamps(i);
            idx = find(time_axis >= t, 1);
            error_i = error_fn(:, idx);
        case "obstacle"
            t_start = stamps(max(i-1, 1));
            t_end = stamps(i);
            
            idx_start = find(time_axis >= t_start, 1);
            idx_end   = find(time_axis >= t_end , 1);

            error_i = max(error_fn(:, idx_start:idx_end), [], 2);
    end

    A_err = [eye(n_augmented); -eye(n_augmented)];
    b_err = zeros(2* n_augmented, 1);
    b_err(obj.position_indices, :)               = error_i;
    b_err(n_augmented + obj.position_indices, :) = error_i;

    error_polyhedron = Polyhedron('A', A_err, 'b', b_err);
    error_polyhedrons = [error_polyhedrons, error_polyhedron];
end
end

