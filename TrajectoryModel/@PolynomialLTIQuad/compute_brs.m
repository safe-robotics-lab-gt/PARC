function [brs, brss] = compute_brs(obj, G)
%COMPUTE_BRS Summary of this function goes here
%   Detailed explanation goes here
t_n = obj.step_number;
ltis = obj.systems;

brss(1, t_n + 1) = Polyhedron;
brs = G;
brss(1) = brs;

for i = t_n:-1:1
    lti = ltis(i);
    brs = brs.invAffineMap(lti.A);
    brs = brs.intersect(lti.domain);
    % brs.minHRep();
    brss(t_n + 2 - i) = brs;
end
end

