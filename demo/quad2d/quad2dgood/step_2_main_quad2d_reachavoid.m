%% Perform PARC on 2D-quadrotor with polynomial model
% See Appendix G in the PARC paper
% Run step_1_build_tracking_error_function to collect tracking error first,
% or use one of the provided error in the data file
% Author: Long Kiu Chung
% Created: 2024/02/05
% Updated: 2024/03/17

clear all;
close all;
init_quad2dgood;

load('quad2dgood_error_625.mat');

%% PARC
% Get error polyhedrons
% Error polyhedron for goal set (maximum final error)
Es_end(2, 1) = Polyhedron;
tracking_error = datas.tracking_error;
errors_max = tracking_error.error_function();
error_stamps = tracking_error.stamps;
for i = 1:2
    e_end = errors_max(i, end);
    Es_end(i) = Polyhedron('A', [1; -1], 'b', [e_end; e_end]);
end

% Error polyhedron for obstacles (maximum interval error)
Es(2, t_n) = Polyhedron;
for i = 1:t_n
    idx_lo = find(error_stamps >= dt.*(i - 1), 1);
    idx_hi = find(error_stamps >= dt.*(i), 1);
    for j = 1:2
        e_max = max(errors_max(j, idx_lo:idx_hi));
        Es(j, i) = Polyhedron('A', [1; -1], 'b', [e_max; e_max]);
    end
end

% Reach set
disp("Computing backward reachable set...")
reachs(2, 1) = Polyhedron;
chains(2, t_n + 1) = Polyhedron;
for i = 1:2 % Compute reach set by decoupling
    [brs, brss] = ltis(i).compute_brs((G(i) - Es_end(i))*K(i));
    reachs(i) = brs;
    chains(i, :) = brss;
end
reach = reachs(1)*reachs(2);

% Avoid set
tic
disp("Computing avoid set...")
avoids = [];
O_n = size(O, 2); % Number of obstacles

for i = 1:O_n % Repeat for each obstacles
    disp(['Working on ', num2str(i), ' of ', num2str(O_n), ' obstacles']);

    for j = 2:(t_n + 1) % Repeat for each timesteps
        disp(['Working on ', num2str(j - 1), ' of ', num2str(t_n), ' timesteps'])

        inv_j = t_n + 2 - j; % For indexing

        % "Buffing" obstacles with error
        O_e_x = (O(1, i) + Es(1, inv_j))*K(1);
        O_e_z = (O(2, i) + Es(2, inv_j))*K(2);

        % One-step BRS of obstacles
        lti_x = ltis_x.systems(inv_j);
        lti_z = ltis_z.systems(inv_j);
        O_e_x_brs = O_e_x.invAffineMap(lti_x.A);
        O_e_x_brs = O_e_x_brs.intersect(lti_x.domain);
        O_e_z_brs = O_e_z.invAffineMap(lti_z.A);
        O_e_z_brs = O_e_z_brs.intersect(lti_z.domain);

        % Convex hull
        O_funnel_x = PolyUnion('Set', [O_e_x, O_e_x_brs]).convexHull;
        O_funnel_z = PolyUnion('Set', [O_e_z, O_e_z_brs]).convexHull;

        % Intermediate avoid set
        avoid_t_x = intersect(O_funnel_x, chains(1, j));
        avoid_t_z = intersect(O_funnel_z, chains(2, j));

        % Back propagate intermediate avoid set
        if ~avoid_t_x.isEmptySet() && ~avoid_t_z.isEmptySet()
            avoid_x = avoid_t_x;
            avoid_z = avoid_t_z;
            for k = (inv_j - 1):-1:1 % Repeat until t_0
                % One-step BRS
                lti_x = ltis_x.systems(k);
                lti_z = ltis_z.systems(k);
                avoid_x = avoid_x.invAffineMap(lti_x.A);
                avoid_x = avoid_x.intersect(lti_x.domain);
                avoid_z = avoid_z.invAffineMap(lti_z.A);
                avoid_z = avoid_z.intersect(lti_z.domain);
            end

            % minHRep() for stability
            avoid_x.minHRep();
            avoid_z.minHRep();

            if ~avoid_x.isEmptySet() && ~avoid_z.isEmptySet()
                avoids = [avoids, avoid_x*avoid_z];
            end

        end
    end
end
toc

%% Plotting
% If the following assertion is satisfied, speed up set difference by
% slicing
assert(k_a_x_lo == k_a_x_hi)
assert(k_a_x_lo == 0)
assert(k_v_z_lo == k_v_z_hi)
assert(k_v_z_lo == 0)
reach_slice = reach.slice([3, 6], [0; 0]);
reach_slice.minHRep();
avoids_slice = avoids.slice([3, 6], [0; 0]);
avoids_slice = avoids_slice.intersect(X*K_v_x*K_pk_x*Z*K_a_z*K_pk_z);
avoids_slice.minHRep();

% Set difference
disp("Computing set difference...")
safe_slice = reach_slice \ avoids_slice;
safe_slice = safe_slice(safe_slice.isBounded); % Numerical instability from set difference

% Projection
disp("Computing projection...")
safe_slice_proj = safe_slice.projection([1, 4]);
safe_slice_proj = safe_slice_proj(safe_slice_proj.isBounded);

% Plotting
figure();
hold on; grid on; box on
disp("Plotting...")
O_1.plot('color', 'r','alpha',0.7)
O_2.plot('color', 'r','alpha',0.7)
safe_slice_proj.plot('color','b','alpha',0.4,'linestyle','none')
X_goal.plot('color', 'g','alpha', 1)
axis equal
xlim([x_lo, x_hi]);
ylim([z_lo, z_hi]);
xlabel('$p_x$ (m)', 'Interpreter', 'latex', 'FontSize', 15)
ylabel('$p_z$ (m)', 'Interpreter', 'latex', 'FontSize', 15)

hold off;