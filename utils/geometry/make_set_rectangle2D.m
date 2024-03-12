function rectangle2D = make_set_rectangle2D(varargin)
% Make 2-dimensional rectangle with given set representation
% Input
%   center : the center of the rectangle -> R2
%   height : the height of the rectangle -> R1
%   width  : the width of the rectangle -> R1
%   rotation : the rotation angle (ccw) with respect to the center -> deg
%   set_representation: the set representation to use (e.g., polytope,
%   zonotope)
%
% Output
%   rectangle2D: the set object for the rectangle2D

K = parse_function_args(varargin{:});
K = check_sanity_and_set_default_kwargs(K,...
    'function_name', mfilename,...
    'required_key', ["center", "height", "width"], ...
    'default_key', ["set_representation", "rotation"],...
    'default_value', {"polytope", 0});

deg2rad = pi/180;

% Parse into local variables
x   = K.center(1);
y   = K.center(2);
th  = K.rotation * deg2rad;
h   = K.height;
w   = K.width;

% Main
switch(K.set_representation)
    case "polytope"
        A_ = [eye(2); -eye(2)];
        R = [cos(th), -sin(th);
             sin(th),  cos(th)];
        A = A_ * R';

        b_ = [x + w/2; y + h/2; -x + w/2; -y + h/2];
        b  = b_ - A_ * (eye(2) - R') * [x; y];
        
        rectangle2D = Polyhedron('A', A, 'b', b);

    otherwise
        error_message = sprintf('Current version does not support %s.', K.set_representation);
        error(error_message);
end
end
