function [samples, trials] = sample_from_set_A_not_B(varargin)
% Sample the points that are included in A but not in B
%
% Author: Wonsuhk Jung
% Created: Jan 23th 2024

kwargs = parse_function_args(varargin{:});
kwargs = check_sanity_and_set_default_kwargs(kwargs, ...
         'required_key', {'set1', 'set2'}, ...
         'default_key', {'n_sample'}, ...
         'default_value', {1});


set1     = kwargs.set1;
set2     = kwargs.set2;
n_sample = kwargs.n_sample;

if isfield(kwargs, 'n_sample_from_set1')
    n_sample_from_set1 = kwargs.n_sample_from_set1;
else
    n_sample_from_set1 = 3 * n_sample;
end

maxAttempt = 50;


samples       = zeros(n_sample, set1.Dim);

sample_count = 0;
attempt = 1;

trials = {};

while (attempt <= maxAttempt) && (sample_count < n_sample)
    
    fprintf("[sample_from_set_A_not_B] Attempt %d\n", attempt)

    % Sample from mother distribution
%     n_sample_from_set1 = (n_sample - sample_count) * N_factor;
    candidates = sample_from_set_hit_run('set', set1, ...
                                         'n_sample', n_sample_from_set1);

    % Rejection sampling
    for i = 1:n_sample_from_set1
        candidate = candidates(i, :);
        if any(set2.contains(candidate'))
            continue;
        end

        sample_count = sample_count + 1;
        samples(sample_count, :) = candidate;

        if sample_count == n_sample
            break
        end
    end
    
    attempt = attempt + 1;

    trials{end+1} = candidates;
end

if sample_count == 0 
    samples = [];
else
    samples = samples(1:sample_count, :);
end

end

