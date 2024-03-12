function ap = retrieve_ap(obj, lookup_point, X_goals)
% Retrieve the "activation pattern" ap for lookup_point in pwa.
% 
% Author: Long Kiu Chung
% Created: 2023/10/17
% Updated: 2023/10/27
%
% INPUTS:
% lookup_point: 1*n vector; point to compute the activation pattern of
% pwa: n-dimensional PWASystem from MPT3; PWA model of the system
% X_goals: 1*(t_max+1) vector of Polyhedron from MPT3; Corresponds to the goal 
%          of each timestep
%
% OUTPUTS:
% ap: 1*t_m vector; activation pattern of lookup_point, i.e. lookup_point
%     reaches X_goals(t_m + 1) in t_m + 1
%     If lookup_point does not lie in pwa or cannot reach X_goals in t_max,
%     ap = []

% Input-proofing

% if lookup_point is column vector, transpose.
if size(lookup_point, 2) == length(lookup_point)
    lookup_point = lookup_point';
end

t_max = length(X_goals);
next_step = lookup_point;
ap = [];

pwa = obj.system;

for i = 1:t_max
    if X_goals(i).contains(next_step)
        break
    elseif i == t_max % lookup_point cannot reach X_goal in t_max
        ap = [];
        return
    end
    % Wonsuhk: You can definitely accelerate this by having some graph
    % of activation pattern.
    for j = 1:pwa.ndyn
        if pwa.domain(j).contains(next_step)
            ap = [ap, j];
            next_step = pwa.A{j}*next_step + pwa.f{j};
            break
        elseif j == pwa.ndyn % next_step out of bounds
            ap = [];
            disp("The queried state is out of the PWA System's domain.")
            return
        end
    end
end