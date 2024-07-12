function equilib = getDriftState_w_V_beta(V_des,beta_rad)
initialization;
% create equilibrium functions to solve for equilibrium points

param_act = [V_des; beta_rad];

x0 = [deg2rad(20); 6000; -1];

syms delta_ Fxr real
syms r V beta_ real

vars = [delta_; Fxr; r];
params = [V; beta_];

x = 0; y =0; yaw = 0;
Fxf = 0;

ux = V*cos(beta_);
uy = V*sin(beta_);

[ ux_dot, uy_dot, r_dot, V_dot, beta_dot, alpha_f, alpha_r, x_dot, y_dot, yaw_dot] = ...
    nonlinear_bicycle_model(ux, uy, r, V, beta_, Fxr, Fxf, delta_, veh, tire_f, tire_r, x, y, yaw);


traj_rvb = [r_dot; V_dot; beta_dot];
trajfun_rvb = matlabFunction(traj_rvb,...
    'File',fullfile('TrajFuncs','trajfun_rvb'),...
    'Comments', 'Used tanh function to join slip / non-slip tire model',...
    'vars',[{vars}, {params}]);%,...
    % 'outputs', {'F'});
options = optimoptions('fsolve');
options.MaxIterations = 1000;
options.MaxFunctionEvaluations = 5000;
options.Display = 'off';

vars_solve = fsolve(@(x)trajfun_rvb(x,param_act), x0,options);

[alpha_f, alpha_r] = slip_angles_rvb(vars_solve(3), param_act(1), param_act(2),...
    vars_solve(1), veh);
[alpha_slip_f, alpha_slip_r] = slip_angle_bound(tire_f, 0,tire_r, vars_solve(2));

equilib.delta = vars_solve(1);
equilib.Fxr = vars_solve(2);
equilib.r = vars_solve(3);
equilib.alpha_f = alpha_f;
equilib.alpha_r = alpha_r;
equilib.alpha_slip_f = alpha_slip_f;
equilib.alpha_slip_r = alpha_slip_r;
end
