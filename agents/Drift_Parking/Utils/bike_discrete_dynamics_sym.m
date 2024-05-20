% This implements the smooth iLQR technique from paper,
% iLQR for Piecewise-Smooth Hybrid Dynamical Systems
% https://arxiv.org/pdf/2103.14584.pdf
%
% code implementation can be found here:
% https://github.com/Kongx231/ilqr
initialization;
syms V_dot delta real
syms x y yaw V dt real

% Define the states and inputs
states = [x;y;yaw;V];
inputs = [V_dot; delta];

% Store any physical parameters needed here e.g. mass, lengths...
parameters = sym([]); 

% Define the dynamics

f = [V*cos(yaw); 
    V*sin(yaw);
    V/veh.L*tan(delta);
    V_dot];

% Eueler integration to define discrete update
f_disc = states + f*dt;

% Linearized discrete dynamical matrices
A_disc = jacobian(f_disc,states);
B_disc = jacobian(f_disc,inputs);


%% Write functions for the dynamics
save_dir = what(fullfile('PARC','agents','Drift_Parking','TrajFuncs')).path;
matlabFunction(A_disc,'File',fullfile(save_dir,'bike_A_disc'),'Vars',[{states},{inputs},{dt},{parameters}]);
matlabFunction(B_disc,'File',fullfile(save_dir,'bike_B_disc'),'Vars',[{states},{inputs},{dt},{parameters}]);

matlabFunction(f,'File',fullfile(save_dir,'bike_f'),'Vars',[{states},{inputs},{parameters}]);
matlabFunction(f_disc,'File',fullfile(save_dir, 'bike_f_disc'),'Vars',[{states},{inputs},{dt},{parameters}]);