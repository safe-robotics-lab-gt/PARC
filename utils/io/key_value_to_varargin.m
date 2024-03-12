function result = key_value_to_varargin(key, value)
% Key and value to varargin format
% Author: Wonsuhk Jung
% Created: Jan 20th 2024
result = cell(1, length(key) + length(value));

for i = 1:length(key)
    result{2*i - 1} = key(i);
    result{2*i} = value(i);
end
end

