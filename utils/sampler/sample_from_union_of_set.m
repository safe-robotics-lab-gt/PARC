function samples = sample_from_union_of_set(varargin)
% This function sample points from the union of geometric sets, directly
% using the sample_from_set function.
%
% Input
%   union_of_set: union of geometric set object of the MPT Toolbox
%   n_sample: number of points to sample
%
% Output
%   samples: point vectors -> (set.dim, n_sample)
%
% Author: Wonsuhk Jung

kwargs = parse_function_args(varargin{:});
kwargs = check_sanity_and_set_default_kwargs(kwargs, ...
            'required_key', {'union_of_set'}, ...
            'default_key', {'n_sample'}, ...
            'default_value', {1});

union_of_set = kwargs.union_of_set;
n_sample   = kwargs.n_sample;

set_array = union_of_set.Set;
num_sets  = length(set_array);

dim = set_array(1).Dim;


max_trial = 100;
samples   = zeros(dim, n_sample);

sample_count = 0;
for i = 1:max_trial
    idx = randi([1 num_sets]);
    set_i = set_array(idx);

    point = sample_from_set('set', set_i, 'n_sample', 1);

    if ~isempty(point)
        sample_count = sample_count + 1;
        samples(:, sample_count) = point;
    end

    if sample_count >= n_sample
        break
    end
end
end

