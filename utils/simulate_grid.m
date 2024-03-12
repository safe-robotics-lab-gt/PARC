function summary = simulate_grid(varargin)
%% Simulate the trajectory tracking at grid of initial states
% This script plans the trajectory, and agent tracks it using the reach set
% and avoid set, evaluating this on grid of states, and returns the summary
% Argument
%   states_grid = [N_states, sdim]
%   agent    : RTD agent
%   planner  : trajectory producing model
%   goal     : polytope representations in workspace-dim
%   reach_set: backward-reachable chain
%
%   Option
%       obstacle: polytopic representations in workspace-dim
%       collision_check: do collision check for validation. Turning this on
%           would be computationally heavy.

kwargs = parse_function_args(varargin{:});
kwargs = check_sanity_and_set_default_kwargs(kwargs, ...
    "required_key", {"agent", "planner", "goal", "states_grid", ...
                     "reach_set"});

do_avoid = false;
if isfield(kwargs, "obstacle") && isfield(kwargs, "avoid_set")
    do_avoid = true;
    obstacles   = kwargs.obstacle;
    avoid_sets  = kwargs.avoid_set;
else
    obstacles = Polyhedron;
end

visualize = false;
if isfield(kwargs, "visualize")
    visualize = kwargs.visualize;
end

collision_check = true;
if isfield(kwargs, "collision_check")
    collision_check = kwargs.collision_check;
end

% vis_option
palette     = get_palette_colors();
trajectory_color   = palette.blue;
control_color      = palette.black;

% parse function arguments
agent       = kwargs.agent;
planner     = kwargs.planner;
goal        = kwargs.goal;
states_grid = kwargs.states_grid;
brs_chains  = kwargs.reach_set;

if visualize
% Visualize the simulation environment.
visualize_simulation_env('agent', agent, 'planner', planner, ...
                         'goal', goal, 'obstacle', obstacles); hold on;
end


n_state = size(states_grid, 1);
options.Display = "off";
summary = {};
progress_bar = waitbar(0, 'Simulating the states');

for i = 1:n_state
    summary_i = struct();
    waitbar(i/n_state, progress_bar);

    s0 = states_grid(i, :)';
    z0 = s0(planner.state_indices, :);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % If online planning, you should do this

    % lookup_points = TPM.advise_lookup_points("state_lookup", z0, ...
    %                                          "goal", goal_center);
    % goal_dim      = TPM.position_indices;
    % non_goal_dim  = setdiff(1:TPM.n_state+TPM.n_param, goal_dim);
    % X_goal_aug    = X_goal * TPM.get_bound_polytopes_by_dims(non_goal_dim);
    % brs_chains    = TPM.compute_brs('X_goal'        , X_goal_aug, ...
    %                               'lookup_points'   , lookup_points, ...
    %                               't_max'           , brs_step_max, ...
    %                               'tracking_error'  , TEF);
    % 
    % obstacle_funnels = TPM.get_obstacle_funnels( ...
    %                          'obstacles', obstacles, ...
    %                          't_max', brs_step_max, ...
    %                          'verbose', true, ...
    %                          'tracking_error', TEF);
    % 
    % avoid_sets = TPM.compute_avoid_set('reach_funnel', brs_chains, ...
    %                                'avoid_funnel', obstacle_funnels,...
    %                                'obstacle', obstacles);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % If offline planning, you should do this
    for j = 1:length(brs_chains)
        BRS_j   = brs_chains{j}.nodes(end);
        
        % compute reach-set
        K_reach = BRS_j.slice(planner.state_indices, z0);

        % If reach-set is empty, continue
        if K_reach.isEmptySet, continue; end

        % Decide if we actually needs to compute avoid trajectory parameter
        exists_avoid_traj = false;
        if do_avoid && ~isempty(avoid_sets{j})
            K_avoid = avoid_sets{j}.slice(planner.state_indices, z0);
            K_avoid = K_avoid(~K_avoid.isEmptySet);
            if ~all(K_avoid.isEmptySet)
                exists_avoid_traj = true;
            end
        end

        % Compute feasible trajectory parameter k_i
        if exists_avoid_traj
            % If there is any unsafe trajectory parameter in reach-set,
            % sample discreetly.

            % 1. If reach set is contained in one of the avoid set, infeasible
            if any(K_avoid.contains(K_reach)), continue; end

            % 2. run hit-and-run & rejection sampling algorithm (efficient)
            k_i = sample_from_set_A_not_B('set1', K_reach, 'set2', K_avoid, ...
                                          'n_sample', 1, 'n_sample_from_set1', 100);

            % 3. if no solution is found, we go to set difference
            if isempty(k_i)
                fprintf("Computing set difference. This may take time...\n");
                K_feas = K_reach \ K_avoid;
                k_i = sample_from_set_hit_run('set', K_feas(1), 'n_sample', 1);
            end
            
            % 4. if no feasible plan is found, there exists no reach-avoid
            % trajectory
            if isempty(k_i), continue; end
        else
            % If there is no unsafe trajectory parameter in reach-set
            k_i = sample_from_set_hit_run('set', K_reach, 'n_sample', 1);
        end

        t_plan_i = (brs_chains{j}.size-1) * planner.affinization_dt;
        
        % Make desired trajectory
        traj_varargin = key_value_to_varargin(planner.param_names, k_i);
        [T_des, U_des, Z_des] = planner.make_desired_trajectory(z0, ...
            t_plan_i, 'ap', flip(brs_chains{j}.activation_pattern), ...
            traj_varargin{:});

        % Track the trajectory
        agent.reset(s0);
        agent.move(T_des(end), T_des, U_des, Z_des);

        % Summary
        pos_real = agent.state(agent.position_indices, :);
        T_real   = agent.time;

        pos_des  = Z_des(planner.position_indices, :);
        pos_des  = match_trajectories(T_real, T_des, pos_des);

        % check if it reached goal
        reach_goal      = goal.contains(pos_real(:, end));

        % checked if it collided with obstacle
        if collision_check
            collision = false;
            for pos = pos_real
                if obstacles.contains(pos)
                    collision = true;
                    break
                end
            end
        end

        if visualize
        % Visualize
        p1 = plot_path(pos_real);
        p1.Color = control_color;

        p2 = plot_path(pos_des);
        p2.Color = trajectory_color;
        p2.LineWidth = 1.5;
        end
        
        % Log it to the summary
        summary_i.reach_goal      = reach_goal;
        summary_i.init_state      = s0;
        summary_i.position_des    = pos_des;
        summary_i.position_real   = pos_real;
        summary_i.stamps          = T_real;
        summary_i.traj_param      = k_i;
        summary_i.states          = agent.state;
        if collision_check
            summary_i.collision       = collision;
        end
        if isfield(agent, 'attitude')
            summary_i.attitudes       = agent.attitude;
        end

        summary{end+1} = summary_i;
        break
    end
end
close(progress_bar);
end
