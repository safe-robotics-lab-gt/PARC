% This implements the smooth iLQR technique from paper,
% iLQR for Piecewise-Smooth Hybrid Dynamical Systems
% https://arxiv.org/pdf/2103.14584.pdf
%
% code implementation can be found here:
% https://github.com/Kongx231/ilqr

syms x y v theta v_dot theta_dot dt

% Define the states and inputs
states = [x;y;theta; v];
inputs = [theta_dot; v_dot];

% Store any physical parameters needed here e.g. mass, lengths...
parameters = sym([]); 

% Define the dynamics
f = [v*cos(theta);
    v*sin(theta);
    theta_dot;
    v_dot];

% Eueler integration to define discrete update
f_disc = states + f*dt;

% Linearized discrete dynamical matrices
A_disc = jacobian(f_disc,states);
B_disc = jacobian(f_disc,inputs);


%% Write functions for the dynamics
sym_files = ["calc_A_disc.m", "calc_B_disc.m", "calc_f.m", "calc_f_disc.m"];
if ~all(arrayfun(@(x) (exist(x)==2), sym_files))
    matlabFunction(A_disc,'File','calc_A_disc','Vars',[{states},{inputs},{dt},{parameters}]);
    matlabFunction(B_disc,'File','calc_B_disc','Vars',[{states},{inputs},{dt},{parameters}]);
    
    matlabFunction(f,'File','calc_f','Vars',[{states},{inputs},{parameters}]);
    matlabFunction(f_disc,'File','calc_f_disc','Vars',[{states},{inputs},{dt},{parameters}]);
end