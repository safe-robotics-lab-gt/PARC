function avoid_sets = compute_avoid_set(obj, varargin)
%% Compute the trajectory that should avoid that is contained in reach set.
% Argument
%   reach_funnel: cell array (1, num_reach_funnel), each funnel is
%                 BackwardReachableChain
%   avoid_funnel: cell array (1, num_obstacle), each funnel is obstacle
%                 funnel that is (max_brs_step+1, n_pwa)
%   obstacle    : this is only required for rough check
% Output
%   avoid_sets  : cell array (1, num_reach_funnel), this is avoid sets
%
% Author: Edgar Chung, Wonsuhk Jung
%
% Created: Jan 18th 2023

kwargs = parse_function_args(varargin{:});
kwargs = check_sanity_and_set_default_kwargs(kwargs, ...
    "required_key", {"reach_funnel", "avoid_funnel", "obstacle"});

reach_funnels = kwargs.reach_funnel;
avoid_funnels = kwargs.avoid_funnel;
obstacles     = kwargs.obstacle;

% Get avoid set
N_reach_funnels = length(reach_funnels);
N_avoid_funnels = length(avoid_funnels);


avoid_sets      = cell(1, N_reach_funnels);

% % brs convexhull precomputation
% brs_convhull_precomputed = cell(N_reach_funnels, 1);
% for i = 1:N_reach_funnels
%     brs_convhull_precomputed_i = [];
%     brs_step_max = reach_funnels{i}.size-1;
%     brs_nodes    = reach_funnels{i}.nodes;
%     for k = 1:brs_step_max
%         brs_convhull_k = PolyUnion('Set', [brs_nodes(k), brs_nodes(k+1)]).convexHull;
%         brs_convhull_k = brs_convhull_k.projection(obj.position_indices);
%         brs_convhull_precomputed_i = [brs_convhull_precomputed_i, brs_convhull_k];
%     end
%     brs_convhull_precomputed{i} = brs_convhull_precomputed_i;
% end


for i = 1:N_avoid_funnels
for j = 1:N_reach_funnels        
    avoid_sets_ij = [];
    
    brs_step_max  = reach_funnels{j}.size-1; % brs chains 1: xgoal, end: brs
    brs_nodes     = reach_funnels{j}.nodes;
    % ms: BRS(T)->BRS(T-1)->...->BRS(1)->Xgoal
    ms            = flip(reach_funnels{j}.activation_pattern);

    for k = 1:brs_step_max
        % k step away from goal
        % append the back-propped avoid sets from obstacle intersecting
        % between brs(k-1), brs(k) funnel where k=1 is goal set.

%         % Rough check
%         brs_convhull_k = brs_convhull_precomputed{j}(k);
%         if intersect(brs_convhull_k, obstacles(i)).isEmptySet
%             continue;
%         end

        % brs(brs_step_max) to brs(k) which travels brs_tep_max + 2 - k
        N_step_to_obstacle = brs_step_max+1-k;
        avoid_t = intersect(brs_nodes(k + 1), ...
                            avoid_funnels{i}{N_step_to_obstacle+1, ...
                                             ms(N_step_to_obstacle)});
            
        if avoid_t.isEmptySet
            continue; 
        end

        avoid_t.minHRep();

        % Accumulate avoid sets through time.
        
        % collides at the first step
        % Note that we do not backpropagate through last dynamics mode.
        % This is implicitly reflected in avoid_t already.
        if N_step_to_obstacle == 1
            avoid_sets_ij = [avoid_sets_ij, avoid_t]; % No backward reachability needed
        else
            ap_brss = obj.brs_from_ap(ms(1:N_step_to_obstacle-1), avoid_t); % BRS from time t
            if ~ap_brss(end).isEmptySet
                avoid_sets_ij = [avoid_sets_ij, ap_brss(end)];
            end
        end
    end
    avoid_sets{j} = [avoid_sets{j}, avoid_sets_ij];
end % end of avoid funnel loop
end % end of reach funnel loop

end

