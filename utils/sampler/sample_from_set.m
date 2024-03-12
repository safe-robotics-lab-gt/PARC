function samples = sample_from_set(varargin)
% This function generates random samples within a given set until the 
% desired number of samples is obtained.
%
% Input
%   set: geometric set object of the MPT Toolbox
%   n_sample: number of points to sample
%
% Output
%   samples: point vectors -> (set.dim, n_sample)
%
% Author: Wonsuhk Jung
% TODO: Change it to more efficient code

kwargs = parse_function_args(varargin{:});
kwargs = check_sanity_and_set_default_kwargs(kwargs, ...
            'required_key', {"set"}, ...
            'default_key', {"n_sample"},...
            'default_value', {1});

set      = kwargs.set;
n_sample = kwargs.n_sample;

max_trial = 1000;

% Construct range of samples
vs = set.V;
dim = set.Dim;
v_lb = min(vs, [], 1);
v_ub = max(vs, [], 1);


sample_count = 0;
samples = zeros(dim, n_sample);

% Rejection Sampling
for i = 1:max_trial
    r = rand(1, dim);
    sample = r.*v_lb + (1-r).*v_ub;
    if set.contains(sample')
        sample_count = sample_count + 1;
        samples(:, sample_count) = sample;
    end

    if sample_count >= n_sample
        break
    end
end
end

