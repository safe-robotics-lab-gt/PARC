function ap_brss = brs_from_ap(obj, ap, X_goal)
% Compute the backward reachable set of X_goal for a PWA system pwa
% "activated" by ap of length t_m.
%
% X_goal == ap_brss{1}
% 
% Author: Long Kiu Chung
% Created: 2023/10/18
% Updated: 2023/10/27
%
% INPUTS:
% ap: 1*t_m vector; activation pattern of lookup_point, i.e. lookup_point
%     reaches X_goal in t_m timestep for pwa
%     Assumes ap is non-empty
% X_goal: Polyhedron from MPT3; Corresponds to the goal region
%
% OUTPUTS:
% ap_brss: 1*(t_m + 1) vector of Polyhedron from MPT3; Backward reachable 
%          sets of X_goal through pwa "activated" by ap at each timestep

    t_m = length(ap);
    ap_brs = X_goal;
    ap_brss{1, t_m + 1} = Polyhedron;
    ap_brss{1} = ap_brs;
    pwa = obj.system;

    for i = t_m:-1:1
        idx = ap(i);
        ap_brs = ap_brs.invAffineMap(pwa.A{idx}, pwa.f{idx});
        ap_brs = ap_brs.intersect(pwa.domain(idx));
        ap_brs.minHRep();
        ap_brss{t_m + 2 - i} = ap_brs;
    end

    if length(X_goal) == 1
        ap_brss = [ap_brss{:}];
    end