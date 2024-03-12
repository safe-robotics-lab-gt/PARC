function visualize_simulation_env(varargin)
% Visualize the simulation environment with the agent and trajectory
% producing model and goal and obstacle, projected on the workspace.

kwargs = parse_function_args(varargin{:});
kwargs = check_sanity_and_set_default_kwargs(kwargs, ...
    'required_key' , {'agent', 'planner', 'goal'}, ...
    'default_key'  , {'obstacle'}, ...
    'default_value', {Polyhedron});

agent    = kwargs.agent;
planner  = kwargs.planner;
goal     = kwargs.goal;
obstacle = kwargs.obstacle;

% vis flag
vis_obstacle = ~obstacle.isEmptySet;

% vis config
palette = get_palette_colors();
goal_color = palette.green;
goal_alpha = 0.4;
obstacle_color = palette.magenta;
obstacle_alpha = 0.1;
bound_color = palette.grey;
bound_alpha = 0.03;
vis_margin = 0.5;
plot_bound = false;
obstacle_linewidth = 0.1;
goal_linewidth = 0.1;


workspace_dim  = agent.position_indices;
n_dim          = length(agent.position_indices);
goal_ws        = goal.projection(workspace_dim);
if vis_obstacle
    obstacle_ws    = obstacle.projection(workspace_dim);
end

% visualization bounds in workspace
state_bounds   = cat(1, planner.states_bound{agent.position_indices});
margins        = repmat([-vis_margin, vis_margin], [n_dim, 1]);
vis_bounds     = state_bounds + margins;
bound_polytope = Polyhedron('A', [eye(n_dim); -eye(n_dim)], ...
                            'b', [state_bounds(:, 2); -state_bounds(:,1)]);


figure;
hc = hold_switch();  make_plot_pretty;

% plot goal set
goal_ws.plot('color', goal_color, 'alpha', goal_alpha, ...
             'linewidth', goal_linewidth, 'linestyle', ':');
% plot obstacle
if vis_obstacle    
    obstacle_ws.plot('color', obstacle_color', 'alpha', obstacle_alpha, ...
                    'linewidth', obstacle_linewidth, 'linestyle', '-');
end
% % plot agent
% plot(agent);
% plot trajectory affinization bound
if plot_bound
bound_polytope.plot('color', bound_color, 'alpha', bound_alpha);
end

xlim(vis_bounds(1, :)); ylim(vis_bounds(2, :));
xlabel("$x$", 'Interpreter','Latex');
ylabel("$y$",'Interpreter','latex');


if n_dim == 3
    zlim(vis_bounds(3, :));
    zlabel("$z$", 'Interpreter','latex');
    view(3);
end

% title("Simulation workspace visualization");
grid on; axis equal;

hold_switch(hc);
end
