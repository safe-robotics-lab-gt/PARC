initialization;
%% create equilibrium functions to solve for equilibrium points
% create equilibrium function using r V beta frame
syms delta Fxr real
syms r V beta real

vars = [delta; Fxr; r];
params = [V; beta];

x = 0; y =0; yaw = 0;
Fxf = 0;

ux = V*cos(beta);
uy = V*sin(beta);

 [ ux_dot, uy_dot, r_dot, V_dot, beta_dot, alpha_f, alpha_r, x_dot, y_dot, yaw_dot] = ...
    nonlinear_bicycle_model(ux, uy, r, V, beta, Fxr, Fxf, delta, veh, tire_f, tire_r, x, y, yaw);

%V = r * R_curve;

traj_rvb = [r_dot; V_dot; beta_dot];
trajfun_rvb = matlabFunction(traj_rvb,...
    'File',fullfile('TrajFuncs','trajfun_rvb'),...
    'Comments', 'Used tanh function to join slip / non-slip tire model',...
    'vars',[{vars}, {params}],...
    'outputs', {'F'});
options = optimoptions('fsolve');
options.MaxIterations = 1000;
options.MaxFunctionEvaluations = 5000;
% options.Display = 'off';

beta_valid = [];
delta_arr = [];
Fxr_arr = [];
r_arr = [];
alpha_f_arr = [];
alpha_r_arr = [];
alpha_slip_f_arr = [];
alpha_slip_r_arr = [];

for beta_sub = 25:1:55
%     vars = [delta; Fxr; r];
%     params = [V; beta];
    param_act = [7; deg2rad(beta_sub)];
    x0 = [deg2rad(20); 6000; -1];
    vars_solve = fsolve(@(x)trajfun_rvb(x,param_act), x0,options);
    
    if isreal(vars_solve)
        [alpha_f, alpha_r] = slip_angles_rvb(vars_solve(3), param_act(1), param_act(2),...
            vars_solve(1), veh);
        [alpha_slip_f, alpha_slip_r] = slip_angle_bound(tire_f, 0,tire_r, vars_solve(2));
        
        beta_valid = [beta_valid; beta_sub];
        delta_arr = [delta_arr; vars_solve(1)];
        Fxr_arr = [Fxr_arr; vars_solve(2)];
        r_arr = [r_arr; vars_solve(3)];
        alpha_f_arr = [alpha_f_arr; alpha_f];
        alpha_r_arr = [alpha_r_arr; alpha_r];
        alpha_slip_f_arr = [alpha_slip_f_arr; alpha_slip_f];
        alpha_slip_r_arr = [alpha_slip_r_arr; alpha_slip_r];
    end
    
end
%% Plotting
figure;
subplot(3,1,1)
title('Equilibrium Analysis')
plot(beta_valid, rad2deg(delta_arr), 'r', 'LineWidth', 2);
ylabel('delta, deg')
subplot(3,1,2)
plot(beta_valid, Fxr_arr, 'b', 'LineWidth', 2);
ylabel('Fxr, N')
subplot(3,1,3)
plot(beta_valid, r_arr, 'Color', [0 0.6 0], 'LineWidth', 2);
ylabel('r, rad/s')
xlabel('beta (deg)');

figure;
subplot(2,1,1)
plot(beta_valid, abs(alpha_f_arr),'LineWidth',1.5)
hold on
plot(beta_valid, alpha_slip_f_arr,'LineWidth',1.5)
hold off
legend('\alpha_f (rad)', '\alpha_f_{slip}')

subplot(2,1,2)
plot(beta_valid, abs(alpha_r_arr),'LineWidth',1.5)
hold on
plot(beta_valid, alpha_slip_r_arr,'LineWidth',1.5)
hold off
legend('\alpha_r (rad)', '\alpha_r_{slip}')


%slipping = [abs(alpha_f), abs(alpha_r)] >= [alpha_slip_f, alpha_slip_r];
%display('front wheel slipping:'+string(slipping(1))+', rear wheel slipping:'+string(slipping(2)));

