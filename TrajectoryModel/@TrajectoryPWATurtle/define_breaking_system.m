function pwa_breaking_system = define_breaking_system(obj, max_decel)
% Define breaking PWA system for Turtlebot trajectory model
% Same as the system of TrajectoryPWATurtle except k_2_t+1 = 0 if k_2_t <
% max_decel*dt, k_2_t+1 = k_2_t - max_decel*dt otherwise.
% 
% Author: Long Kiu Chung
% Created: 2023/12/07
% Updated: 2023/12/07
%
% OUTPUTS:
% pwa_breaking_system: Custom class TrajectoryPWATurtle; Same as obj, only
%                      obj.system is changed into breaking behaviour

syss = [];

% Parse affine grid setting
theta_idx       = 3;
theta_grid_num  = obj.affinization_N_grid(theta_idx);
theta_min       = obj.states_params_bound{theta_idx}(1);
theta_max       = obj.states_params_bound{theta_idx}(2);
theta_step      = (theta_max - theta_min) / theta_grid_num;

v_des_idx       = 5;
v_des_grid_num  = obj.affinization_N_grid(v_des_idx);
v_des_min       = obj.states_params_bound{v_des_idx}(1);
v_des_max       = obj.states_params_bound{v_des_idx}(2);
v_des_step      = (v_des_max - v_des_min) / v_des_grid_num;

dt = obj.affinization_dt;

for i = 1:v_des_grid_num 
    v_des_star = v_des_min + (i - 0.5) * v_des_step;
    % Compute lower bound and upper bound of PWA region for k_2 to
    % determine the case for deceleration
    v_des_lb = v_des_star - 0.5.*v_des_step;
    v_des_ub = v_des_star + 0.5.*v_des_step;
    
    for j = 1:theta_grid_num
        theta_star = theta_min + (j - 0.5) * theta_step;
        
        % Breaking dynamics
        if v_des_lb <= max_decel*dt
            % Jacobian
            A_stop = [1, 0, 0, 0, 0;
                      0, 1, 0, 0, 0;
                      0, 0, 1, dt, 0;
                      0, 0, 0, 1, 0;
                      0, 0, 0, 0, 0];
            f_stop = [0;
                      0;
                      0;
                      0;
                      0];
            sys_stop = LTISystem('A', A_stop, 'f', f_stop, 'Ts', dt);
        end

        % Deceleration dynamics
        if v_des_ub >= max_decel*dt
            % Jacobian
            A_decel = [1, 0, -dt.*v_des_star.*sin(theta_star), 0, dt.*cos(theta_star);
                       0, 1, dt.*v_des_star.*cos(theta_star), 0, dt.*sin(theta_star);
                       0, 0, 1, dt, 0;
                       0, 0, 0, 1, 0;
                       0, 0, 0, 0, 1];
            f_decel = [theta_star.*dt.*v_des_star.*sin(theta_star);
                       -theta_star.*dt.*v_des_star.*cos(theta_star);
                       0;
                       0;
                       -max_decel*dt];
            sys_decel = LTISystem('A', A_decel, 'f', f_decel, 'Ts', dt);
        end

        % If bound interval includes switch point
        if v_des_lb <= max_decel*dt && v_des_ub >= max_decel*dt
            R_decel = Polyhedron('A', [eye(5); -eye(5)],...
                                 'b', [obj.states_params_bound{1}(2); ...       % x_max
                                       obj.states_params_bound{2}(2); ...       % y_max
                                       theta_star + 0.5.*theta_step; ...      % theta* + 0.5*dtheta
                                       obj.states_params_bound{4}(2); ...       % k1_max
                                       v_des_ub; ...                          % k2* + 0.5*dk2
                                       -obj.states_params_bound{1}(1); ...      % -x_min
                                       -obj.states_params_bound{2}(1); ...      % -y_min
                                       -theta_star + 0.5.*theta_step; ...     % -theta* + 0.5*dtheta
                                       -obj.states_params_bound{4}(1); ...      % -k1_min
                                       -max_decel*dt]);                       % -k2* + 0.5*dk2
            sys_decel.setDomain('x', R_decel);
            R_stop = Polyhedron('A', [eye(5); -eye(5)],...
                                'b', [obj.states_params_bound{1}(2); ...       % x_max
                                      obj.states_params_bound{2}(2); ...       % y_max
                                      theta_star + 0.5.*theta_step; ...      % theta* + 0.5*dtheta
                                      obj.states_params_bound{4}(2); ...       % k1_max
                                      max_decel*dt; ...                          % k2* + 0.5*dk2
                                      -obj.states_params_bound{1}(1); ...      % -x_min
                                      -obj.states_params_bound{2}(1); ...      % -y_min
                                      -theta_star + 0.5.*theta_step; ...     % -theta* + 0.5*dtheta
                                      -obj.states_params_bound{4}(1); ...      % -k1_min
                                      -v_des_lb]);  
            sys_stop.setDomain('x', R_stop);
            syss = [syss, sys_decel, sys_stop];
        else
            R = Polyhedron('A', [eye(5); -eye(5)],...
                           'b', [obj.states_params_bound{1}(2); ...       % x_max
                                 obj.states_params_bound{2}(2); ...       % y_max
                                 theta_star + 0.5.*theta_step; ...      % theta* + 0.5*dtheta
                                 obj.states_params_bound{4}(2); ...       % k1_max
                                 v_des_ub; ...                          % k2* + 0.5*dk2
                                 -obj.states_params_bound{1}(1); ...      % -x_min
                                 -obj.states_params_bound{2}(1); ...      % -y_min
                                 -theta_star + 0.5.*theta_step; ...     % -theta* + 0.5*dtheta
                                 -obj.states_params_bound{4}(1); ...      % -k1_min
                                 -v_des_lb]);   

            % If both bounds under switch point
            if v_des_ub <= max_decel*dt
                sys_stop.setDomain('x', R);
                syss = [syss, sys_stop];
            
            % If both bounds above switch point
            else
                sys_decel.setDomain('x', R);
                syss = [syss, sys_decel];
            end
        end        
    end %j
end %i

% Everything is same as TrajectoryPWATurtle except the PWA system
pwa_breaking_system = obj;
pwa_breaking_system.system = PWASystem(syss);

end

