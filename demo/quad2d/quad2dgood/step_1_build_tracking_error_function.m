%% Compute tracking error for 2D-quadrotor with polynomial model
% Corresponds to Section IV D in the PARC paper
% Author: Long Kiu Chung
% Created: 2024/02/05
% Updated: 2024/03/17

clear all;
close all;
init_quad2dgood;

%% Collect error
A = quad2dAgent();

T_des = 0:dt:t_f;
U_des = 0.5*g*ones(2,t_n + 1); % Initialize drone with some stabilizing control
quad2d_error_datas = {};
count = 0;

% To change the number of errors collected, change the third argument of
% the linspaces in the for loop
for k_v_x = linspace(k_v_x_lo, k_v_x_hi, 10)
    for k_pk_x = linspace(k_pk_x_lo, k_pk_x_hi, 10)
        for k_a_x = linspace(k_a_x_lo, k_a_x_hi, 1)
            xk = [(x_lo + x_hi)./2; k_v_x; k_a_x; k_pk_x];
            % Doesn't matter what x_0 is because system is translation invariant
            % Here we pick mid-point between x_lo and x_hi so trajectory would
            % not get out of bounds (hopefully)
            x_history = ltis_x.simulate(xk);
            x_des = x_history(1, :);

            [v_x_des, a_x_des, ~, ~] = ltis_x.differentiate(k_v_x, k_a_x, k_pk_x);
            for k_v_z = linspace(k_v_z_lo, k_v_z_hi, 1)
                for k_a_z = linspace(k_a_z_lo, k_a_z_hi, 10)
                    for k_pk_z = linspace(k_pk_z_lo, k_pk_z_hi, 10)
                        zk = [(z_lo + z_hi)./2; k_v_z; k_a_z; k_pk_z];
                        % Doesn't matter what z_0 is because system is translation invariant
                        % Here we pick mid-point between z_lo and z_hi so trajectory would
                        % not get out of bounds (hopefully)
                        z_history = ltis_z.simulate(zk);
                        z_des = z_history(1, :);
                        
                        [v_z_des, a_z_des, ~, ~] = ltis_z.differentiate(k_v_z, k_a_z, k_pk_z);
                        theta_des = zeros(1, t_n + 1);
                        omega_des = zeros(1, t_n + 1);
        
                        for i = 1:(t_n + 1)
                            theta_des(i) = atan(-a_x_des(i)./(a_z_des(i) + g));
                        end
        
                        omega_des(1:t_n) = diff(theta_des)./dt;
        
                        % trajectory vars
                        Z_des = [x_des;z_des;theta_des;v_x_des;v_z_des;omega_des];
        
                        A.reset([x_des(1); ...
                                 z_des(1); ...
                                 0; ...
                                 v_x_des(1); ...
                                 v_z_des(1); ...
                                 0]);
                        A.move(t_f,T_des,U_des,Z_des);
        
                        % evaluate at the finer grid
                        T_eval       = 0:A.integrator_time_discretization:max(T_des);
                        pos_eval     = match_trajectories(T_eval, A.time, A.state([1, 2], :));
                        pos_des_eval = match_trajectories(T_eval, T_des,  Z_des([1, 2], :));
                        err_eval     = abs(pos_eval - pos_des_eval);
        
                        % data logging: write custom data struct here
                        data_ij = TrackingErrorData('errors', err_eval, ...
                                                    'stamps', T_eval, ...
                                                    'traj_param', [k_v_x; k_a_x; k_pk_x; k_v_z; k_a_z; k_pk_z]);
        
                        quad2d_error_datas{end+1} = data_ij;
        
                        count = count + 1;
                        disp(['Simulating trajectory number ', num2str(count)]);
                    end
                end
            end
        end
    end
end

% concatenate tracking error data
error_data = quad2d_error_datas{1}.extend('datas', quad2d_error_datas(2:end));
% error_data.plot();
datas.tracking_error = error_data;
save('demo/quad2d/quad2dgood/data/quad2dgood_error_10000.mat', 'datas','-v7.3');