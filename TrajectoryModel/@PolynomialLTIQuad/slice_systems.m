function obj_slice = slice_systems(obj, int_con)
% USELESS TEST FUNCTION, DO NOT USE
R = obj.domain;
t_n = obj.step_number;
ltis = obj.systems;
dt = obj.timestep;

obj_slice = obj;

dim = R.Dim;
slice_dim = length(int_con);
R_slice = R.slice(1:slice_dim, int_con);
obj_slice.domain = R_slice;

slice_systems(1, t_n) = LTISystem;

for i = 1:t_n
    lti = ltis(i);
    A = lti.A;
    Idx = [zeros(dim - slice_dim, slice_dim), eye(dim - slice_dim)];
    A_new = Idx*(A(:, (slice_dim + 1):end));
    f_new = Idx*(A(:, 1:slice_dim))*int_con;

    slice_system = LTISystem('A', A_new, 'f', f_new, 'Ts', dt);
    slice_system.setDomain('x', R_slice);

    slice_systems(i) = slice_system;
end

obj_slice.systems = slice_systems;

end

