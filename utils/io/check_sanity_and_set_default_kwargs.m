function kwargs = check_sanity_and_set_default_kwargs(kwargs, varargin)
%% A convenient function to check and format the kwargs
% If "required key" does not exist in kwargs, this function throws error. 
% If "default key" does not exist in the kwargs, this function replace
% it with default value
%
% Input
%   kwargs: keyword-value to check sanity and set default -> struct
%
% Output
%   function_name: which function called this?
%   required_key : this field should be included in the kwargs
%   default_key  : this field is overwritten by default value if not exists
%   default_value: this value overwrites the default key -> cell
%
% Author: Wonsuhk Jung
% Created: 19 Oct 2023
meta_kwargs = parse_function_args(varargin{:});

function_name = "";
if isfield(meta_kwargs, 'function_name')
    function_name = meta_kwargs.function_name;
end

required_key = [];
if isfield(meta_kwargs, "required_key")
    required_key = meta_kwargs.required_key;
end

default_key = {};
if isfield(meta_kwargs, "default_key")
    default_key = meta_kwargs.default_key;
end

default_value = {};
if isfield(meta_kwargs, "default_value")
    default_value = meta_kwargs.default_value;
    if isnumeric(default_value)
        default_value = num2cell(default_value);
    end
end

% Sanity checking
if length(default_key) ~= length(default_value)
    error("The length of the default keys should be same with" + ...
        "the length of the default values");
end

% Sanity checking: Required Keys
for i = 1:length(required_key)
    if ~isfield(kwargs, required_key{i})
        error_msg = sprintf("You must specify [%s] to use the function [%s]", ...
            required_key{i}, function_name);
        error(error_msg);
    end
end

% Set default: Default Keys
for i = 1:length(default_key)
    if ~isfield(kwargs, default_key{i})
        kwargs.(default_key{i}) = default_value{i};
    end
end
end
