function brss = compute_brs(obj, G, R)
%COMPUTE_BRS Summary of this function goes here
%   Detailed explanation goes here
Cs = obj.Cs;
ds = obj.ds;
t_n = obj.t_n;

brss(1, t_n) = Polyhedron;
brs = G.minHRep();
brss(1) = brs;

for i = 1:(t_n - 1)
    brs = brs.invAffineMap(Cs(:, :, end + 1 - i), ds(:, end + 1 - i));
    brs = brs.intersect(R);
    brs.minHRep();
    brss(i + 1) = brs;
end

end

