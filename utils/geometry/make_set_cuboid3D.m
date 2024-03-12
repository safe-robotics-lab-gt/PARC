function cuboid3D = make_set_cuboid3D(varargin)
% Make 3-dimensional cuboid with given set representation
% Input
%   center : the center of the cuboid -> R3
%   thickness  : the width of the cuboid -> R1
%   height : the height of the cuboid -> R1
%   width  : the width of the cuboid -> R1
%   set_representation: the set representation to use (e.g., polytope,
%   zonotope)
%
% Output
%   cuboid3D: the set object for the cuboid3D

K = parse_function_args(varargin{:});
K = check_sanity_and_set_default_kwargs(K,...
    'function_name', mfilename,...
    'required_key', ["center", "thickness", "width", "height"], ...
    'default_key', ["set_representation", "rotation"],...
    'default_value', {"polytope", 0});

% Parse into local variables
c   = K.center;
t   = K.thickness;
h   = K.height;
w   = K.width;

% process
if size(c, 1) == 1, c = c'; end

% Main
switch(K.set_representation)
    case "polytope"
        A = [eye(3); -eye(3)];
        b = [t/2; w/2; h/2; t/2; w/2; h/2];

        b = b + A * c;
        
        cuboid3D = Polyhedron('A', A, 'b', b);

    otherwise
        error_message = sprintf('Current version does not support %s.', K.set_representation);
        error(error_message);
end
end