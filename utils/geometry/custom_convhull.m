function [P_out] = custom_convhull(P_1, P_2)
%CUSTOM_CONVHULL Summary of this function goes here
%   Detailed explanation goes here

P_1.computeVRep();
P_2.computeVRep();
Vn = [P_1.V; P_2.V];
H = Polyhedron(Vn);
H.minVRep();

x0 = H.interiorPoint().x;
V = H.V;
K = convhulln(V); % {'QJ', 'Qx', 'Qs'}
d = size(V, 2);
A = zeros(size(K, 1), d);
B = ones(size(K, 1), 1);
for i = 1:size(K, 1)
    % each row of K contains indices of vertices that lie on the i-th
    % facet
    P = V(K(i, :), :);
    % compute the normal vector and the offset of the facet
    W = [P, -ones(d, 1)];
    [AB, ~] = qr(W'); % qr() is much faster than null()
    a = AB(1:d, end);
    b = AB(end, end);
    
    % determine the sign
    if a'*x0>b
        a = -a;
        b = -b;
    end
    A(i, :) = a';
    B(i) = b;
end
P_out = Polyhedron('A', A, 'b', B);
end

