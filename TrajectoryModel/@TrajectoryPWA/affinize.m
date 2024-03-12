function system = affinize(obj, x, dx)
% Affinize the system based on the state bounds, parameter
% bounds and grid information
syss    = [];
na_dims = find(obj.affinization_N_grid~=1);

if isempty(na_dims)
    na_dims = 1:(obj.n_state+obj.n_param); 
end

% make_grids
grids = cell(1, length(na_dims));
steps = zeros(1, length(na_dims));
for i = 1:length(na_dims)
    na_dim = na_dims(i);
    bound  = obj.states_params_bound{na_dim};
    n_grid = obj.affinization_N_grid(na_dim);
    step   = (bound(2) - bound(1)) / n_grid;

    steps(i) = step;
    grids{i} = bound(1) + step/2 : step : bound(2) - step/2;
end

[grids{:}] = ndgrid(grids{:});
numGrids = numel(grids);
coeff_vec = zeros(numel(grids{1}), numGrids);
for i = 1:numGrids
    coeff_vec(:, i) = grids{i}(:);
end

n  = obj.n_state + obj.n_param;
I  = eye(n);
J  = jacobian(dx, x);
dt = obj.affinization_dt;
b  = cat(1, obj.states_params_bound{:});
b  = [b(:, 2); -b(:, 1)];

for coeff = coeff_vec'
    x_i = subs(x, x(na_dims), coeff);
    A_i = subs(J, x(na_dims), coeff);
    A_i = double(A_i);
    f_i = subs(dx, x(na_dims), coeff) - A_i*x_i;
    f_i = double(f_i);
    
    % Make it to the discrete-time
    A_i = I+A_i*dt;
    f_i = f_i*dt;

    sys_i = LTISystem('A', A_i, 'f', f_i, 'Ts', dt);

    b_i = b;
    b_i(na_dims)   = coeff + 0.5 * steps';
    b_i(n+na_dims) = -coeff + 0.5 * steps';
    R_i = Polyhedron('A', [eye(n); -eye(n)], ...
                     'b', b_i);

    sys_i.setDomain('x', R_i);
    syss = [syss, sys_i];
end

system = PWASystem(syss);
end