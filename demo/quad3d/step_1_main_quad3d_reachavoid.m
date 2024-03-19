%% PARC on quadrotor with 12D polynomial low-fi model for comparison with RTD
% Author: Long Kiu Chung
% Created: 2024/01/14
% Updated: 2024/03/18

clear;
close all;

init_quad3d;

%% Tracking Error
% Parse through the error table
load('quadrotor_tracking_error_table_dt0.02_vmax_5.25.mat'); % This is from the RTD_quadrotor_DSCC_2019 repository

% Convert the error data to a readable format
tracking_error_table = table2array(tracking_error_table);
new_error_table = zeros(t_n, 3);
data_n = size(tracking_error_table, 1);
for i = 1:data_n
    data_row = tracking_error_table(i, :);
    t_lo = data_row(1);
    e_x_lo = abs(data_row(9));
    e_x_hi = abs(data_row(10));
    e_y_lo = abs(data_row(11));
    e_y_hi = abs(data_row(12));
    e_z_lo = abs(data_row(13));
    e_z_hi = abs(data_row(14));

    % Get the maximum error at the desired timesteps
    idx = round((t_lo / 0.02)) + 1;
    if idx <= t_n
        new_error_table(idx, 1) = max(max(e_x_lo, e_x_hi), new_error_table(idx, 1));
        new_error_table(idx, 2) = max(max(e_y_lo, e_y_hi), new_error_table(idx, 2));
        new_error_table(idx, 3) = max(max(e_z_lo, e_z_hi), new_error_table(idx, 3));
    end
end

%% Reach Set
tic
reachs(2, 1) = Polyhedron;
chains(3, t_n + 1) = Polyhedron;

disp("Computing backward reachable set...");
for i = 1:3
    Es_end = Polyhedron('A', [1; -1], 'b', [new_error_table(t_n, i); new_error_table(t_n, i)]);
    [brs, brss] = ltis(i).compute_brs((G(i) - Es_end)*K);
    reachs(i) = brs;
    chains(i, :) = brss;
end
toc

%% Avoid Set
% If the following assertions are true, consider only obstacles in y-axis
% to speed up computation
% If not, implement eq.80 in the PARC paper a la
% step_2_main_quad2d_reachavoid in quad2dgood
assert(G_Y.contains(y_0));
for i = 1:O_n
    assert(~Os(2, i).contains(y_0));
end

tic
% Compute error reachable sets
Es(1, t_n) = Polyhedron;
for j = 1:t_n
    Es(j) = Polyhedron('A', [1; -1], 'b', [new_error_table(j, 2); new_error_table(j, 2)]);
end

% Compute avoid set
disp('Computing avoid set');
avoids_y = [];

for j = 1:O_n % For each obstacle
    disp(['Working on ', num2str(j), ' of ', num2str(O_n), ' obstacles']);
    O = Os(2, j);

    for k = 2:(t_n + 1) % For each timestep
        disp(['Working on ', num2str(k - 1), ' of ', num2str(t_n), ' timesteps'])
        inv_k = t_n + 2 - k; % Convenient index
        O_e = (O + Es(inv_k))*K; % "Buff" obstacle by error

        % One step BRS of obstacle
        lti = ltis(2).systems(inv_k);
        O_e_brs = O_e.invAffineMap(lti.A);
        O_e_brs = O_e_brs.intersect(lti.domain);
        
        if avoid_set_method == "no_convex_hull"
            % Alternate avoid set method without convex hull
            O_funnel = [O_e, O_e_brs];
            if no_convex_hull_condition_check
                isHullNeeded = check_vertex_intersection(O_e, O_e_brs, k_v_lo, k_v_hi, k_a_lo, k_a_hi, k_pk_lo, k_pk_hi);
                if isHullNeeded
                    error("no_convex_hull method cannot be used")
                end
            end
        else
            % Convex hull
            try
                O_funnel = PolyUnion('Set', [O_e, O_e_brs]).convexHull;
            catch
                try
                    % Round vertices to 4 decimal places for numerical stability of
                    % CDD solver
                    Vn = [O_e.V; O_e_brs.V];
                    Vn = round(Vn, 4);
                    H = Polyhedron('V', Vn);
                    H.minVRep();
                    H.minHRep();
                    O_funnel = H;
                catch
                    % Round vertices to 3 decimal places for numerical stability of
                    % CDD solver
                    Vn = [O_e.V; O_e_brs.V];
                    Vn = round(Vn, 3);
                    H = Polyhedron('V', Vn);
                    H.minVRep();
                    H.minHRep();
                    O_funnel = H;
                end
            end
        end

        for n = 1:numel(O_funnel)
            % Intermediate avoid set
            avoid_t = intersect(chains(2, k), O_funnel(n));

            if ~avoid_t.isEmptySet
                avoid = avoid_t;
                for m = (inv_k - 1):-1:1 % Backpropagate avoid set to t_0
                    % One-step BRS
                    lti = ltis(i).systems(m);
                    avoid = avoid.invAffineMap(lti.A);
                    avoid = avoid.intersect(lti.domain);
                end
                if ~avoid.isEmptySet
                    avoids_y = [avoids_y, avoid];
                end
            end
        end

    end
end

toc

%% Plotting
figure(1) ; clf ; hold on ; grid on ; axis equal ;
G_plot = G_X*G_Y*G_Z;
G_plot.plot('color', 'green', 'alpha', 0.1);
O_1_plot = O_1_real_X*O_1_real_Y*O_1_real_Z;
O_2_plot = O_2_real_X*O_2_real_Y*O_2_real_Z;
O_1_plot.plot('color', 'red', 'alpha', 0.1);
O_2_plot.plot('color', 'red', 'alpha', 0.1);

% Find feasible trajectories
good_k = cell(1, 3);
avoidY_slice = avoids_y.slice(1:2, [y_0, 0]); % The tracking error was collected with 0 initial velocity
for i = 1:3
    Omega_slice = reachs(i).slice(1:2, [int_con(i); 0]); % The tracking error was collected with 0 initial velocity

    % Get lower bound and upper bound of reach set by hyperrectangle outer
    % approximation
    Omega_box = PolyUnion(Omega_slice);
    Omega_box = outerApprox(Omega_box);
    k_pk_lb = Omega_box.Internal.lb(2);
    k_pk_ub = Omega_box.Internal.ub(2);
    
    % Trackig error was collected in with initial acceleration
    % between -5 m/s^2 and 5 m/s^2
    k_a_lb = -5;
    k_a_ub = 5;
    
    sample_n = 5;
    if i == 2
        sample_n = 10; % Sample more in y-axis
    end

    for j = linspace(k_a_lb, k_a_ub, sample_n)
        for k = linspace(k_pk_lb, k_pk_ub, sample_n)
            if Omega_slice.contains([j;k])
                if i == 2 % Take into account the avoid set for y-axis
                    isin = avoidY_slice.contains([j;k], true);
                    if ~any(isin)
                        good_k{i} = [good_k{i}; j, k];
                    end
                else
                    good_k{i} = [good_k{i}; j, k];
                end
            end
        end
    end
end

% Plot feasible trajectories
T_ref = 0:dt:t_f;
U_ref = zeros(4,length(T_ref));
for i = 1:size(good_k{1}, 1) % x-axis
    k_a_x = good_k{1}(i, 1);
    k_pk_x = good_k{1}(i, 2);
    x_k = [x_0; 0; k_a_x; k_pk_x];
    x_history = ltis_x.simulate(x_k);
    x_des = x_history(1, :);
    [v_x_des, a_x_des, j_x_des, s_x_des] = ltis_x.differentiate(0, k_a_x, k_pk_x);

    for j = 1:size(good_k{2}, 1) % y-axis
        k_a_y = good_k{2}(j, 1);
        k_pk_y = good_k{2}(j, 2);
        y_k = [y_0; 0; k_a_y; k_pk_y];
        y_history = ltis_y.simulate(y_k);
        y_des = y_history(1, :);
        [v_y_des, a_y_des, j_y_des, s_y_des] = ltis_y.differentiate(0, k_a_y, k_pk_y);

        for k = 1:size(good_k{3}, 1) % z-axis
            k_a_z = good_k{3}(k, 1);
            k_pk_z = good_k{3}(k, 2);
            z_k = [z_0; 0; k_a_z; k_pk_z];
            z_history = ltis_z.simulate(z_k);
            z_des = z_history(1, :);
            [v_z_des, a_z_des, j_z_des, s_z_des] = ltis_z.differentiate(0, k_a_z, k_pk_z);

            Z_ref = [x_des; y_des; z_des;
                     v_x_des; v_y_des; v_z_des;
                     a_x_des; a_y_des; a_z_des;
                     j_x_des; j_y_des; j_z_des;
                     s_x_des; s_y_des; s_z_des];

            % get agent
            A = quadrotor_agent() ;
            A.reset([Z_ref(1:3, 1)]);
            A.integrator_time_discretization = 0.01 ;
            A.LLC = Mellinger_LLC() ;
            
            % try different initial conditon
            A.state(A.velocity_indices) = [Z_ref(4, 1); Z_ref(5, 1); Z_ref(6, 1)] ;
            
            % move agent along spline
            A.move(t_f,T_ref,U_ref,Z_ref)
            plot3(A.state(1, :), A.state(2, :), A.state(3, :), 'b')
        end
    end
end

xlabel('$p_x$ (m)', 'Interpreter', 'latex', 'FontSize', 15)
ylabel('$p_y$ (m)', 'Interpreter', 'latex', 'FontSize', 15)
zlabel('$p_z$ (m)', 'Interpreter', 'latex', 'FontSize', 15)
xlim([0, 10])
zlim([0, 10])
view(3);
hold off;

