function os = get_current_os()
% Retrieve the current operating system. This function is convenient when
% your code gets different according to your operating system.
%
% Author: Wonsuhk Jung
% Date: Dec 12th, 2023
osType = computer;

if startsWith(osType, 'GLNX')
    os = "linux";
elseif startsWith(osType, 'MACI')
    os = "mac";
elseif startsWith(osType, 'PCWIN')
    os = "window";
else
    error('Unknown operating system');
end
end

