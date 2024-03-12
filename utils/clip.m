function s_ = clip(s, min_val, max_val)
%% Clip value to fit in the minimum value and maximum value
% Author: Wonsuhk Jung
% Created: Jan 22 2024
    s_ = s;
    s_(s<min_val) = min_val(s<min_val);
    s_(s>max_val) = max_val(s>max_val);
end