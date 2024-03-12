function P = cartesian_product_with_axis(P1, P2, axis1, axis2)
% Cartesian product with the pre-specified axis order
%
% cartesian_product_with_axis(P1, P2, [3, 1], [2, 4]) would result in the
% producting polytopes and change the axis with x1' = x2, x2' = x3, ...
%
% Input
%   axis1: the map of the index to the newly producted polytope axis
%   axis2
% Output
%   polytope
% Author: Wonsuhk Jung
% Created Jan 18 2023
%
    N1 = length(axis1);
    N2 = length(axis2);

    new_axis = zeros(1,N1+N2);
    new_axis(axis1) = 1:N1;
    new_axis(axis2) = N1+1:N1+N2;

    P = P1 * P2;

    % This is same with projection but representation changes.
    P = Polyhedron("A", P.A(:, new_axis), "b", P.b);
end