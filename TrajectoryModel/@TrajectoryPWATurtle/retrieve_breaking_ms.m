function [ms, ms_break, brs_break] = retrieve_breaking_ms(obj, lookup_point, X_break_goals, pwa_break, E)
%   For lookup_point, retrieve the mode sequences ms in obj and ms_break in
%   pwa_break
%
%   Author: Long Kiu Chung
%   Created: 2023/12/09
%   Updated: 2023/12/09
%
%   INPUTS:
%   obj: @TrajectoryPWATurtle; TurtleBot PWA dynamics model in low fidelity
%   lookup_point: n*1 array; point to compute the mode sequence of
%   X_break_goal_final: Polyhedron from MPT3; Corresponds to the goal at 
%                       the final timestep in pwa_break, accounting for 
%                       error
%   pwa_break: @TrajectoryPWATurtle; TurtleBot PWA breaking dynamics model
%              in low fidelity created by
%              obj.define_breaking_system(max_decel)
%   E: 1*(t_max+1) array of Polyhedron from MPT3; Corresponds to the error
%      polyhedron at each timestep in obj
%
%   OUTPUTS:
%   ms: 1*t_ms array; mode sequence of lookup_point in obj
%   ms_break: 1*t_break_max array; mode sequence of lookup_point in 
%             pwa_break
%   brs_break: Polyhedron from MPT3; BRS of X_break_goal_final through
%              pwa_break minkowski differenced by the E at the
%              corresponding timestep in obj
%   If lookup_point does not reach X_break_goals or wanders out of bound in
%   obj or pwa_break, ms, ms_break = []

% Initialization
t_max = length(E) - 1;
t_break_max = length(X_break_goals) - 1;
X_break_goal_final = X_break_goals(end);
next_step = lookup_point;
ms = [];
ms_break = zeros(1, t_break_max);
brs_break = Polyhedron;

pwa = obj.system;
pwa_b = pwa_break.system;

for i = 1:(t_max + 1)
    % Try Breaking
    is_oob_breaking = false;
    break_step = next_step;
    for j = 1:(t_break_max + 1)
        for k = 1:pwa_b.ndyn
            if pwa_b.domain(k).contains(break_step)
                ms_break(j) = k;
                break_step = pwa_b.A{k}*break_step + pwa_b.f{k};
                break
            elseif k == pwa_b.ndyn
                is_oob_breaking = true;
                break
            end
        end
        if is_oob_breaking
            break
        end
    end

    % If break is successful, check if goal is reached
    if ~is_oob_breaking && X_break_goal_final.contains(break_step)
        brss_break = pwa_break.brs_from_ap(ms_break, X_break_goal_final);
        brs_break = brss_break(end) - E(i);

        % If BRS not empty after compounding error, ms and ms_break is found
        if ~brs_break.isEmptySet
            return
        end

    % If max timestep is reached for obj
    elseif i == t_max + 1
        ms = [];
        ms_break = [];
        return
    end

    % If breaking doesn't reach the goal at this timestep, move one step forward
    for j = 1:pwa.ndyn
        if pwa.domain(j).contains(next_step)
            ms = [ms, j];
            next_step = pwa.A{j}*next_step + pwa.f{j};
            break

        % next_step out of bounds
        elseif j == pwa.ndyn
            ms = [];
            ms_break = [];
            return
        end
    end
end

