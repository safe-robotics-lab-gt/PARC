% Author: Long Kiu Chung, Chuizheng Kong
% Created: 2024/01/20
close all; clear all;
root_dir = what(fullfile('PARC','demo','drift_parking'), '-all').path;
cd(root_dir)
%% Build dynamical model from data
disp("Processing collected data...")
% Parameters
dt = 0.1;
dt_data = 0.01;

% Load data from Kong
load("../../../data/driftParkingTraj_01-22-2024 23-39.mat");

% Find t_f
data_t_n = 0;
data_n = numel(driftTrajData);
for i = 1:data_n
    data_t_n_i = size(driftTrajData{i}.states, 1);
    if data_t_n_i > data_t_n
        data_t_n = data_t_n_i;
    end
end
t_f = dt_data.*(data_t_n - 1);
t_f = ceil(t_f./dt).*dt;
t_n = (t_f./dt) + 1;

% Build data table
x_table = zeros(data_n, t_n);
y_table = zeros(data_n, t_n);
theta_table = zeros(data_n, t_n);
v_table = zeros(data_n, 1);
beta_table = zeros(data_n, 1);

for i = 1:data_n
    datum = driftTrajData{i};
    data_t_n_i = size(datum.states, 1);
    t_end = dt_data.*(data_t_n_i - 1);
    idxs = round((0:dt:t_end)./dt_data) + 1;
    idxs_n = length(idxs);
    x_table(i, 1:idxs_n) = datum.states(idxs, 1);
    y_table(i, 1:idxs_n) = datum.states(idxs, 2);
    theta_table(i, 1:idxs_n) = datum.states(idxs, 3);
    if t_n > idxs_n
        x_table(i, (idxs_n + 1):end) = datum.states(end, 1);
        y_table(i, (idxs_n + 1):end) = datum.states(end, 2);
        theta_table(i, (idxs_n + 1):end) = datum.states(end, 3);
    end
    v_table(i) = datum.TrajParam(1);
    beta_table(i) = datum.TrajParam(2);
end

% Build least squares matrix
disp("Constructing dynamical model...")

coef = zeros(t_n, 9);
A = [v_table, beta_table, ones(data_n, 1)];
for i = 1:t_n
    b_x = x_table(:, i);
    b_y = y_table(:, i);
    b_theta = theta_table(:, i);
    c_x = A\b_x;
    c_y = A\b_y;
    c_theta = A\b_theta;
    coef(i, :) = [c_x', c_y', c_theta'];
end

driftsys = PolynomialDrifting("final_time", t_f, "timestep", dt, "coef", coef);

Cs = driftsys.Cs;
ds = driftsys.ds;

% % figure();
% for i = 1:data_n
%     v = v_table(i);
%     beta = beta_table(i);
%     history = zeros(5, t_n);
%     x = [0; 0; 0; v; beta];
%     history(:, 1) = x;
%     for j = 1:(t_n - 1)
%         x = Cs(:, :, j)*x + ds(:, j);
%         history(:, j + 1) = x;
%     end
%
%     close all;
%     figure();
%     subplot(3, 1, 1);
%     hold on;
%     title(['v = ', num2str(v), ', beta = ', num2str(beta)]);
%     plot(0:dt:t_f, x_table(i, :), 'r');
%     plot(0:dt:t_f, history(1, :), 'b');
%     ylabel('x (m)')
%
%     subplot(3, 1, 2);
%     hold on;
%     plot(0:dt:t_f, y_table(i, :), 'r');
%     plot(0:dt:t_f, history(2, :), 'b');
%     ylabel('y (m)')
%
%     subplot(3, 1, 3);
%     hold on;
%     plot(0:dt:t_f, theta_table(i, :), 'r');
%     plot(0:dt:t_f, history(3, :), 'b');
%     ylabel('theta (rad)')
%
% end

%% Build error model from data
load("../../../data/error_function_drifting.mat");
% disp("Building error model...")
%
% datas = {};
% T_eval = 0:dt_data:t_f;
% T_eval_n = length(T_eval);
% for i = 1:data_n
%     v = v_table(i);
%     beta = beta_table(i);
%     data_i = struct();
%     data_i.timestamps = T_eval;
%     T_des = 0:dt:t_f;
%
%     history = zeros(5, t_n);
%     x = [0; 0; 0; v; beta];
%     history(:, 1) = x;
%     for j = 1:(t_n - 1)
%         x = Cs(:, :, j)*x + ds(:, j);
%         history(:, j + 1) = x;
%     end
%
%     pos_des = history(1:3, :);
%     pos = driftTrajData{i}.states(:, 1:3)';
%     t_data_n = size(driftTrajData{i}.states, 1);
%     if t_data_n < T_eval_n
%         pos(:, (t_data_n + 1):T_eval_n) = repmat(pos(:, end), 1, T_eval_n - t_data_n);
%     end
%
%     pos_des_eval = match_trajectories(T_eval, T_des, pos_des);
%     data_i.error = abs(pos - pos_des_eval);
%     datas{end+1} = data_i;
% end
%
% % Build the error function (time-wise maximizing)
% num_data = length(datas);
% error_dim = size(datas{1}.error, 1);
%
% field_name = "error";
%
% % First extract the time stamp values (time-axis)
% stamps = [];
% for i = 1:num_data
%     stamps_i = datas{i}.timestamps;
%     stamps = [stamps, stamps_i];
% end
%
% tol = 0.0001;
% stamps = uniquetol(stamps, tol);
%
% length_T = length(stamps);
%
% % Aggregate errors according to the stamps by interpolating
% errors_agg = [];
% for i = 1:num_data
%     if mod(i, 100) == 0
%         disp(i)
%     end
%     errors_i = datas{i}.error;
%     stamps_i = datas{i}.timestamps;
%
%     % Interpolate the error vectors
%     errors_interp_i = zeros(error_dim, length_T);
%
%     for j = 1:error_dim
%         errors_ij = errors_i(j, :);
%         errors_ij = interp1(stamps_i, errors_ij, stamps, 'linear');
%         errors_interp_i(j, :) = errors_ij;
%     end
%     errors_interp_i = reshape(errors_interp_i, [1, error_dim, length_T]);
%
%     errors_agg = [errors_agg; errors_interp_i];
% end
%
% [errors_max, arg_errors_max] = max(errors_agg, [], 1);
% errors_max = squeeze(errors_max);
% arg_errors_max = squeeze(arg_errors_max);
%
%
% error_data.errors_max = errors_max;
% error_data.arg_errors_max = arg_errors_max;
% error_data.stamps = stamps;
%
% % Plot the error function
% figure;
%
% subplot(3, 1, 1)
% hold on;
% % for i = 1:length(datas)
% %     plot(stamps, datas{i}.error(1, :), '-k')
% % end
% plot(stamps, errors_max(1, :), 'r--', 'Linewidth', 3);
%
% subplot(3, 1, 2)
% hold on;
% % for i = 1:length(datas)
% %     plot(stamps, datas{i}.error(2, :), '-k')
% % end
% plot(stamps, errors_max(2, :), 'r--', 'Linewidth', 3);
%
% subplot(3, 1, 3)
% hold on;
% % for i = 1:length(datas)
% %     plot(stamps, datas{i}.error(3, :), '-k')
% % end
% plot(stamps, errors_max(3, :), 'r--', 'Linewidth', 3);

%% PARC
%% Parameters
% World bounds
x_lo = -5;
x_hi = 40;
y_lo = -20;
y_hi = 5;
theta_lo = -1.5.*pi;
theta_hi = 0.5.*pi;
v_lo = min(v_table);
v_hi = max(v_table);
beta_lo = min(beta_table);
beta_hi = max(beta_table);

X = Polyhedron('A', [1; -1], 'b', [x_hi; -x_lo]);
Y = Polyhedron('A', [1; -1], 'b', [y_hi; -y_lo]);
Theta = Polyhedron('A', [1; -1], 'b', [theta_hi; -theta_lo]);
V = Polyhedron('A', [1; -1], 'b', [v_hi; -v_lo]);
Beta = Polyhedron('A', [1; -1], 'b', [beta_hi; -beta_lo]);
VB = V*Beta;
R = X*Y*Theta*VB; % Domain

% Car parameters
car_l = 4.267;
car_w = 1.988;
% Overapproximate car volume with a circle
r = sqrt(car_l.^2 + car_w.^2)./2;
sides = 10;
A_car = zeros(sides, 2);
theta_car = linspace(0, 2.*pi, sides + 1);
for i = 1:sides
    A_car(i, :) = [cos(theta_car(i)), sin(theta_car(i))];
end
V_car = Polyhedron('A', A_car, 'b', r.*ones(sides, 1));

% Goals and obstacles
G = make_set_cuboid3D("center", [30.5; -16; -pi], "thickness", 3.7, "width", 1.6, "height", pi./3);
% G = make_set_cuboid3D("center", [30.5; -16; -pi], "thickness", 3, "width", 1, "height", pi./2);
O_1_real = make_set_rectangle2D("center", [24; -16], "width", 4.267, "height", 1.988);
O_2_real = make_set_rectangle2D("center", [37; -16], "width", 4.267, "height", 1.988);
% O_1_real = make_set_rectangle2D("center", [26; -14.4], "width", 2.4, "height", 4.8);
% O_2_real = make_set_rectangle2D("center", [35; -14.4], "width", 2.4, "height", 4.8);
O_1 = O_1_real + V_car;
O_2 = O_2_real + V_car;
Os = [O_1, O_2];

%% Error Polyhedrons
disp("Constructing error polyhedrons...");
error_fn = error_data.errors_max;
E_end = make_set_cuboid3D("center", [0; 0; 0], "thickness", error_fn(1, end).*2, "width", error_fn(2, end).*2, "height", error_fn(3, end).*2);
E_obs(1, t_n - 1) = Polyhedron;
time_axis = error_data.stamps;
for i = 1:(t_n - 1)
    t_start = dt.*(i - 1);
    t_end = dt.*i;
    idx_start = find(time_axis >= t_start, 1);
    idx_end = find(time_axis >= t_end , 1);
    error_i = max(error_fn(:, idx_start:idx_end), [], 2);
    E_obs(i) = make_set_cuboid3D("center", [0; 0; 0], "thickness", error_i(1).*2, "width", error_i(2).*2, "height", error_i(3).*2);
end

%% Backward Reachable Sets
tic
disp("Computing backward reachable sets...");
brss = driftsys.compute_brs((G - E_end)*VB, R);
reach_set = brss(end);


% Avoid set
figure();
hold on;
% brss.projection(1:2).plot('color', 'm', 'alpha', 0.1);
disp("Computing avoid set...");
O_n = numel(Os);
avoids = [];
for i = 1:O_n
    disp(['Working on ', num2str(i), ' of ', num2str(O_n), ' obstacles']);
    O = Os(i)*Theta;
    for j = 2:t_n
        disp(['Working on ', num2str(j - 1), ' of ', num2str(t_n - 1), ' timesteps']);
        O_e = (O + E_obs(end + 2 - j))*VB;
        O_e_brs = O_e.invAffineMap(Cs(:, :, end + 2 - j), ds(:, end + 2 - j));
        O_e_brs = O_e_brs.intersect(R);
        O_e_brs.minHRep();
        try
            obs_funnel = PolyUnion('Set', [O_e, O_e_brs]).convexHull;
        catch
            try
                % Round vertices to 4 decimal places for numerical stability of
                % CDD solver
                Vn = [O_e.V; O_e_brs.V];
                Vn = round(Vn, 4);
                H = Polyhedron('V', Vn);
                H.minVRep();
                H.minHRep();
                obs_funnel = H;
            catch
                try
                    % Round vertices to 3 decimal places for numerical stability of
                    % CDD solver
                    Vn = [O_e.V; O_e_brs.V];
                    Vn = round(Vn, 3);
                    H = Polyhedron('V', Vn);
                    H.minVRep();
                    H.minHRep();
                    obs_funnel = H;
                catch
                    try
                        % Round vertices to 2 decimal places for numerical stability of
                        % CDD solver
                        Vn = [O_e.V; O_e_brs.V];
                        Vn = round(Vn, 2);
                        H = Polyhedron('V', Vn);
                        H.minVRep();
                        H.minHRep();
                        obs_funnel = H;
                    catch
                        % Round vertices to 2 decimal places for numerical stability of
                        % CDD solver
                        Vn = [O_e.V; O_e_brs.V];
                        Vn = round(Vn, 1);
                        H = Polyhedron('V', Vn);
                        H.minVRep();
                        H.minHRep();
                        obs_funnel = H;
                    end
                end
            end
        end
        % obs_funnel = custom_convhull(O_e, O_e_brs);
        % close all;
        % figure();
        % hold on;
        % obs_funnel.projection(1:2).plot('color', 'r', 'alpha', 0.1);
        % O_e.projection(1:2).plot('color', 'b', 'alpha', 0.1);
        % O_e_brs.projection(1:2).plot('color', 'b', 'alpha', 0.1);
        % brss(j).projection(1:2).plot('color', 'g', 'alpha', 0.1);
        % hold off;
        avoid_t = intersect(obs_funnel, brss(j));
        if ~avoid_t.isEmptySet
            avoid_t.minHRep();
            % avoid_t.projection(1:2).plot('color', 'r', 'alpha', 0.1);
            avoid = avoid_t;
            for k = (t_n - j):-1:1
                avoid = avoid.invAffineMap(Cs(:, :, k), ds(:, k));
                % avoid = avoid.intersect(brss(t_n - k + 1));
                avoid = avoid.intersect(R);
            end
            avoids = [avoids, avoid];
        end
    end
end
toc;

%% Plotting
disp("Plotting...")
figure();
hold on;
O_1_real.plot('color', 'r');
O_2_real.plot('color', 'r');
G.projection(1:2).plot('color', 'g', 'alpha', 0.1);
safe_set = reach_set\avoids;
U = PolyUnion('Set', safe_set.slice(3, 0)).outerApprox;
% U = PolyUnion('Set', reach_set).outerApprox;
x_lb = U.Internal.lb(1);
x_ub = U.Internal.ub(1);
y_lb = U.Internal.lb(2);
y_ub = U.Internal.ub(2);
theta_lb = U.Internal.lb(3);
theta_ub = U.Internal.ub(3);
planned_states = [];
act_end_states = [];

theta = 0;
for x = linspace(x_lb, x_ub, 10)
    for y = linspace(y_lb, y_ub, 10)
        for i = 1:data_n
            v = v_table(i);
            beta = beta_table(i);
            int_con = [x; y; theta; v; beta];
            % if reach_set.contains(int_con, 1) && ~any(avoids.contains(int_con, 1))
            if any(safe_set.contains(int_con, 1))
                history = zeros(5, t_n);
                history(:, 1) = int_con;
                states = int_con;
                for j = 1:(t_n - 1)
                    states = Cs(:, :, j)*states + ds(:, j);
                    history(:, j + 1) = states;
                end
                planned_states = [planned_states; {history}];
                plot(history(1, :), history(2, :), '-k');

                xy_real = driftTrajData{i}.states(:, 1:2)';
                xy_real = xy_real + [x; y];
                act_end_states = [act_end_states, xy_real(:,end)];

                plot(xy_real(1, :), xy_real(2, :), '--b');

                break
            end
        end
    end
end
xlim([x_lo, x_hi])
ylim([y_lo, y_hi])
hold off;
%% Sampling control params from the BRS and verifying it on the hi-fi dynamics
% samples = sample_from_set_A_not_B("set1", reach_set.slice(3, 0), "set2", avoids.slice(3, 0), "n_sample", 100);
% samples = sample_from_set("set", safe_set.slice(3, 0), "n_sample", 100);
traj_found = 0;
v_lb = U.Internal.lb(3);
v_ub = U.Internal.ub(3);
beta_lb = U.Internal.lb(4);
beta_ub = U.Internal.ub(4);
samples = [];
planned_states = {};
act_end_states = [];
act_pos_states_lumped = [];
for x = linspace(x_lb, x_ub, 10)
    for y = linspace(y_lb, y_ub, 10)
        for v = linspace(v_lb, v_ub, 100)
            for beta = linspace(beta_lb, beta_ub, 100)
                int_con = [x; y; theta; v; beta];

                if any(safe_set.contains(int_con, 1))
                    traj_found = traj_found+1;
                    disp(['obtained trajectory ', string(traj_found)])
                    states = int_con;
                    history = zeros(5, t_n);
                    history(:, 1) = int_con;
                    for j = 1:(t_n - 1)
                        states = Cs(:, :, j)*states + ds(:, j);
                        history(:, j + 1) = states;
                    end
                    planned_states = [planned_states; {history}];
                    A_selected = exeDriftParkingCtrlParams(int_con(4), int_con(5), int_con(1:3)');
                    act_end_states = [act_end_states, A_selected.states(end,1:3)'];
                    act_pos_states_lumped = [act_pos_states_lumped,NaN(2,1), A_selected.states(:,1:2)'];
                    samples = [samples, int_con];
                end
            end
        end
    end
end
%% find min error traj
goal_state = [30.5; -16; -pi];
diff_to_goal = vecnorm((act_end_states - goal_state).*[1;1;2], 2, 1); % emphasis angle a bit more
[~, min_idx] = max(diff_to_goal)
traj_num = min_idx;
A_drift = exeDriftParkingCtrlParams(samples(4,traj_num), samples(5,traj_num), samples(1:3,traj_num)');
root_dir = what(fullfile('RTD_BRS','demo','drifting_parallel_parking'), '-all').path;
cd(root_dir)
opt_plan_traj = planned_states{traj_num};
%%
close all
ax_animation = A_drift.visualize();
veh_obs = A_drift.veh;
veh_obs.body_color = [1,0,0];
veh_obs.pos = [37,-16;24,-16];
plot_car(ax_animation, veh_obs.pos(1,:),-pi,0,veh_obs,1);
plot_car(ax_animation, veh_obs.pos(2,:),-pi,0,veh_obs,1);
c1 = length(get(ax_animation, 'Children'));
pause(0.1)
plot_world_road(ax_animation, [x_lo, x_hi], [y_lo, y_hi])
chd = get(ax_animation, 'Children');
c2 = length(chd);
uistack(chd(1:(c2-c1)),'bottom');
plot(ax_animation, opt_plan_traj(1,:), opt_plan_traj(2,:),'Color',[0.3,0.3,0.3,1],'LineWidth',2,'LineStyle','--');
% G.projection(1:2).plot('color', 'g', 'alpha', 0.1);
set(ax_animation,'xlim',[x_lo,x_hi]);
set(ax_animation,'ylim',[y_lo,y_hi]);

% plot drift sequence for paper plot
ax_seq = A_drift.plot_drift_sequence();
% opt_plan_traj = planned_states{traj_num};
z = plot(ax_seq, A_drift.states(:,1), A_drift.states(:,2),'Color',[0,0,1,1],'LineWidth',1.5);
p = plot(ax_seq, opt_plan_traj(1,:), opt_plan_traj(2,:),'Color',[0.3,0.3,0.3,1],'LineWidth',3,'LineStyle','--');
plot_car(ax_seq, veh_obs.pos(1,:),-pi,0,veh_obs,1);
plot_car(ax_seq, veh_obs.pos(2,:),-pi,0,veh_obs,1);
G.projection(1:2).plot('color', 'g', 'alpha', 0.3);
set(ax_seq,'xlim',[x_lo,x_hi]);
set(ax_seq,'ylim',[y_lo,y_hi]);
legend([p,z],{'Planning Model','Tracking Model'})
save_figure_to_pdf(gcf, fullfile(root_dir,"DriftParkingFinalTraj.pdf"))
%% visualize safe set
close all
ax_set = A_drift.plot_drift_sequence();
plot_car(ax_set, veh_obs.pos(1,:),-pi,0,veh_obs,1);
plot_car(ax_set, veh_obs.pos(2,:),-pi,0,veh_obs,1);
plot(ax_set,act_pos_states_lumped(1,:),act_pos_states_lumped(2,:),'color',[0,0.7,0,0.05],'Linewidth',0.1);
G.projection(1:2).plot('color', 'g', 'alpha', 0.7);
safe_set_2d = safe_set.slice(3,0).projection(1:2);
safe_set_vis = PolyUnion('Set', safe_set_2d).convexHull;
safe_set_vis.plot('color', 'g', 'alpha', 0.7);
z = plot(ax_set, A_drift.states(:,1), A_drift.states(:,2),'Color',[0,0,1,1],'LineWidth',1.5);
set(ax_set,'xlim',[x_lo,x_hi]);
set(ax_set,'ylim',[y_lo,y_hi]);
save_figure_to_pdf(gcf, fullfile(root_dir,"DriftParkingfromBRAS.pdf"))