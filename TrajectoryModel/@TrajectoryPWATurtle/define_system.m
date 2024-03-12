function pwa_system = define_system(obj)
% Linearize the TurtleBot trajectory model into a PWA system
% 
% Author: Long Kiu Chung
% Created: 2023/10/10
% Updated: 2023/10/20, Wonsuhk Jung
%
%
% OUTPUTS:
% turtle_pwa: PWASystem from MPT3; PWA model of TurtleBot trajectory model

fprintf("Creating PWA System. This may take some time...\n")
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
    
    for j = 1:theta_grid_num
        theta_star = theta_min + (j - 0.5) * theta_step;
        
        % Jacobian
        A = [1, 0, -dt.*v_des_star.*sin(theta_star), 0, dt.*cos(theta_star);
             0, 1, dt.*v_des_star.*cos(theta_star), 0, dt.*sin(theta_star);
             0, 0, 1, dt, 0;
             0, 0, 0, 1, 0;
             0, 0, 0, 0, 1];
        f = [theta_star.*dt.*v_des_star.*sin(theta_star);
             -theta_star.*dt.*v_des_star.*cos(theta_star);
             0;
             0;
             0];
        
        % Define the linearized dynamics at each region
        sys_ij = LTISystem('A', A, 'f', f, 'Ts', dt);
        
        R   = Polyhedron('A', [eye(5); -eye(5)],...
                         'b', [obj.states_params_bound{1}(2); ...       % x_max
                               obj.states_params_bound{2}(2); ...       % y_max
                               theta_star + 0.5.*theta_step; ...      % theta* + 0.5*dtheta
                               obj.states_params_bound{4}(2); ...       % k1_max
                               v_des_star + 0.5.*v_des_step; ...      % k2* + 0.5*dk2
                              -obj.states_params_bound{1}(1); ...       % -x_min
                              -obj.states_params_bound{2}(1); ...       % -y_min
                              -theta_star + 0.5.*theta_step; ...      % -theta* + 0.5*dtheta
                              -obj.states_params_bound{4}(1); ...       % -k1_min
                              -v_des_star + 0.5.*v_des_step]);        % -k2* + 0.5*dk2
        
        sys_ij.setDomain('x', R);
        
        syss = [syss, sys_ij];
    end % j

end % i 

pwa_system = PWASystem(syss);
end
