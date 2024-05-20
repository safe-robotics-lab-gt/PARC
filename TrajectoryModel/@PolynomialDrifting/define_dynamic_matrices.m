function [Cs, ds] = define_dynamic_matrices(obj)
% Parse settings
t_n = obj.t_n;
coef = obj.coef;

Cs = zeros(5, 5, t_n - 1);
ds = zeros(5, t_n - 1);

for i = 1:t_n - 1
    C = eye(5);
    d = zeros(5, 1);
    coef_t = coef(i, :);
    coef_tp1 = coef(i + 1, :);
    C(1, 4) = coef_tp1(1) - coef_t(1);
    C(1, 5) = coef_tp1(2) - coef_t(2);
    d(1) = coef_tp1(3) - coef_t(3);
    C(2, 4) = coef_tp1(4) - coef_t(4);
    C(2, 5) = coef_tp1(5) - coef_t(5);
    d(2) = coef_tp1(6) - coef_t(6);
    C(3, 4) = coef_tp1(7) - coef_t(7);
    C(3, 5) = coef_tp1(8) - coef_t(8);
    d(3) = coef_tp1(9) - coef_t(9);
    Cs(:, :, i) = C;
    ds(:, i) = d;
end

end

