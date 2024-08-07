function kwargs = parse_function_args(varargin)
%% kwargs = parse_function_args(varargin{:});
% Adapted from https://github.com/ChoiJangho/mufun
    kwargs = struct();
    for idx = 1:2:length(varargin)
        kwargs.(varargin{idx}) = varargin{idx+1};
    end
end