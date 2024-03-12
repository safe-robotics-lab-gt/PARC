function [dist, closest_point] = dist_point_to_polytope(point, polytope, varargin)
% Given the polytope and the point, it returns the closest distance from
% point to the polytope by solving the optimization program.
%
% Argument
%   point: (ndim, 1) vector
%   

kwargs = parse_function_args(varargin{:});

metric = "l1";
if isfield(kwargs, 'metric')
    metric = kwargs.metric;
end

ndim = polytope.Dim;

% Preprocessing the dimensions
if size(point, 1) ~= ndim
    if size(point, 2) == ndim
        point = point';
    else
        error("The dimension of the point does not match that of the polytope.")
    end
end

switch metric
    case "l1"
        
        c = [zeros(ndim, 1); ones(ndim, 1)];
        A = [eye(ndim), -eye(ndim);
            -eye(ndim), -eye(ndim);
            polytope.A, zeros(size(polytope.A))];
        b =  [point;
             -point;
             polytope.b];

        options = optimoptions('linprog', 'Display', 'none');
        [x, fval] = linprog(c, A, b, [], [], [], [], options);

        dist = fval;
        closest_point = x(1:ndim);

    otherwise
        error("Other error metric than l1 is not supported.");
end
end

