function square2D = make_set_square2D(varargin)
% Make 2-dimensional square with given set representation
% Input
%   center : the center of the square -> R2
%   side   : the side of the square -> R1
%   rotation : the rotation angle (ccw) with respect to the center -> deg
%   set_representation: the set representation to use (e.g., polytope,
%   zonotope)
%
% Output
%   square2D: the set object for the square2D

K = parse_function_args(varargin{:});
K = check_sanity_and_set_default_kwargs(K,...
    'function_name', mfilename,...
    'required_key', ["center", "side"], ...
    'default_key', ["set_representation", "rotation"],...
    'default_value', {"polytope", 0});

% Main
switch(K.set_representation)
    case "polytope"
        square2D = make_set_rectangle2D('center', K.center,...
                                        'rotation', K.rotation,...
                                        'height', K.side, ...
                                        'width', K.side, ...
                                        'set_representation', K.set_representation);
    otherwise
        error_message = sprintf('Current version does not support %s.', K.set_representation);
        error(error_message);
end
end
