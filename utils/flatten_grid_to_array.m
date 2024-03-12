function array = flatten_grid_to_array(varargin)
% Create the (N_tuple, dim) array from each grid information.
%
% Input
%   grids : cell array of grids, where each grid is 1D array.
%   format: output format, support mat/cell
%
% Output
%   array : array of combination of each grids as each coordinate
%
% Author: Wonsuhk Jung
% Created: Jan 19th 2023


kwargs = parse_function_args(varargin{:});
kwargs = check_sanity_and_set_default_kwargs(kwargs, ...
        "required_key", ["grids"], ...
        "default_key",  {"format"}, ...
        "default_value", {"mat"});

grids  = kwargs.grids;
format = kwargs.format;

% Flatten the grid
[grids{:}] = ndgrid(grids{:});
numGrids = numel(grids);
array = zeros(numel(grids{1}), numGrids);
for i = 1:numGrids
    array(:, i) = grids{i}(:);
end

if format == "cell"
    array = num2cell(array, 2);
end

end

