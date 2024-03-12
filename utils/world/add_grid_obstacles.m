%% Auxilary Functions
function world = add_grid_obstacles(world, n_grid, d_grid, cx, varargin)
% Create the Nw x Nh grid obstacles that has clearance of cw x ch
% Argument
%   world   : world -> zonotope_box_world
%   n_grid  : number vector of grid -> (nw, nh)
%   d_grid  : clearance vector of grid -> (dw, dh)
%   cx      : x-position of the grid obstacles plane
%   varargin
%       length: the thickness of zonotope obstacle in x-direction

% Parse optional args
kwargs = parse_function_args(varargin{:});
l = 3;
if isfield(kwargs, 'length')
    l = kwargs.length/2;
end

nw = n_grid(1); nh = n_grid(2);
dw = d_grid(1); dh = d_grid(2);
B = world.bounds;

w = (B(4) - B(3) - (nw+1)*dw)/nw;
h = (B(6) - B(5) - (nh+1)*dh)/nh;

% Calculate center vectors
cw_vec = (B(3)+0.5*w+dw : w+dw : B(4)-0.5*w-dw);
ch_vec = (B(5)+0.5*h+dh : h+dh : B(6)-0.5*h-dh);

% Add obstacles
for cw = cw_vec
    for ch = ch_vec
        c = [cx; cw; ch];
        [l,w,h,c] = resize_box_for_world_bounds(l,w,h,c,world.bounds) ;
        world.add_obstacle(l, w, h, c);
    end
end
end