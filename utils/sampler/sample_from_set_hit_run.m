function samples = sample_from_set_hit_run(varargin)
% Sample uniformly points contained in the set using hit_and_run algorithm
% Argument
%   set      : polytope
%   n_sample : Number of samples to generate
%
% Output
%   samples  : (polytope.Dim, n_sample) points
%
% Author: Wonsuhk Jung
% Created: Jan 23th 2024
% Note: Binary search and optimization is inferior to analytic method

kwargs = parse_function_args(varargin{:});
kwargs = check_sanity_and_set_default_kwargs(kwargs, ...
    'required_key', {'set', 'n_sample'}, ...
    'default_key', {'reset_period'}, ...
    'default_value', 100);

set = kwargs.set;
n_sample = kwargs.n_sample;
reset_period = kwargs.reset_period;

if set.isEmptySet
    samples  = [];
    return
end

if numel(set) ~= 1
    samples = [];
    fprintf("The set should be a single polytope\n");
    return
end

% compute initial point to start walking
initialPoint = set.interiorPoint.x;

% Preallocate sample matrix
samples = zeros(n_sample, set.Dim);
currentPoint = initialPoint;
samples(1, :) = currentPoint;

for i = 1:n_sample-1
    % To avoid the local trap
    if mod(i, reset_period) == 0
        currentPoint = initialPoint;
    end
    
    % Random direction - uniform sampling from a unit sphere
    direction = randn(set.Dim, 1);
    direction = direction / norm(direction);

    % Analytic solution: find the largest possible step to be contained in
    % the polytope.
    a_ = set.A*direction;
    b_ = set.b-set.A*currentPoint;

    plus_index = a_>0;
    t = min(b_(plus_index)./a_(plus_index), [], 1);
%     nextPoint = currentPoint + t * rand() * direction;
    nextPoint = currentPoint + t * 0.999 * direction;

    if ~set.contains(nextPoint)
        continue;
    end

    % Update currentPoint and store the sample
    currentPoint = nextPoint;
    samples(i+1, :) = currentPoint';
end
end
