% Test Controller
% add /.../RTD_BRS/, .../simulator/ to path
%% Shreyas Dubin's Car Traj
A = turtlebot_agent('LLC', turtlebot_iLQR_LLC);
z_init = [0;0;0;0.5];
A.reset(z_init)
figure(1) ; clf ; axis equal ;
plot(A)
% make trajectory
t_f = 10 ; % 20
w_des = 0.5; % 1.5 rad/s 
v_des = 0.8; % m/s
[T,U,Z] = turtlebot_discretized_trajectory(t_f, w_des, v_des, 500);

% plot
hold on
plot(Z(1,:),Z(2,:),'b--','LineWidth',1.5)
hold off

% move the robot
t_total = t_f ;
%A.integrator_type = 'ode4';
[states,inputs,k_feedforward,K_feedback, cost_arr] = A.LLC.iterate(z_init, T, U, Z, 2000);
A.move(t_total,T,U,Z)
A.animate()
% plotting speed 
figure(2)
plot(A.time,A.state(4,:),'LineWidth',1.5)
hold on
plot(T,Z(4,:),'b--','LineWidth',1.5);
hold off
figure;
% get the realized position trajectory
T_act = A.time ;
Z_act = A.state(A.position_indices,:) ;

% interpolate the realized trajectory to match the braking traj timing
pos = match_trajectories(T,T_act,Z_act) ;

% get the desired trajectory
pos_des = Z(1:2,:) ;

% compute the tracking error
pos_err = abs(pos - pos_des) ;
x_err = pos_err(1,:) ;
hold on;
y_err = pos_err(2,:) ;
plot(T,x_err, 'r')
plot(T,y_err, 'k')
legend('x_{err}', 'y_{err}')
hold off

%% Braking Trajectory Test
% make trajectory
t_f = 10 ; % 20
w_des = 0.5; % 1.5 rad/s 
v_des = 0.8; % m/s

A_brk = turtlebot_agent('LLC', turtlebot_iLQR_LLC);
z_init = [0;0;0;0.3];
A_brk.reset(z_init)
t_stop = 1;
[T_brk,U_brk,Z_brk] = make_turtlebot_braking_trajectory(t_f,t_stop,w_des,v_des) ;

% plot
figure(3);  clf ; axis equal ;
hold on
plot(Z_brk(1,:),Z_brk(2,:),'b--','LineWidth',1.5)
hold off

% move the robot
t_total = t_f + t_stop;
[states,inputs,k_feedforward,K_feedback, cost_arr]=A_brk.LLC.iterate(z_init, T_brk, U_brk, Z_brk, 2000);
A_brk.move(t_total+2,T_brk,U_brk,Z_brk)
A_brk.animate()
% plotting speed 
figure(4)
plot(A_brk.time,A_brk.state(4,:),'LineWidth',1.5)
hold on
plot(T_brk,Z_brk(4,:),'b--','LineWidth',1.5);
hold off
figure;
% get the realized position trajectory
T_act = A_brk.time ;
Z_act = A_brk.state(A_brk.position_indices,:) ;

% interpolate the realized trajectory to match the braking traj timing
pos = match_trajectories(T_brk,T_act,Z_act) ;

% get the desired trajectory
pos_des = Z_brk(1:2,:) ;

% compute the tracking error
pos_err = abs(pos - pos_des) ;
x_err = pos_err(1,:) ;
hold on;
y_err = pos_err(2,:) ;
plot(T_brk,x_err, 'r')
plot(T_brk,y_err, 'k')
legend('x_{err}', 'y_{err}')
hold off

%% Test S path
load("S_letter_path.mat")
figure(4) ; clf ; axis equal ;
hc = hold_switch();
hold_switch(hc)
Sls = Sls+[-1;1];
[xT,yT] = setR0T(Sls);
plot(Sls(1,:), Sls(2,:));
quiver(Sls(1,1:end-1),Sls(2,1:end-1),yT(1,:), yT(2,:))
theta = atan2(yT(2,:),yT(1,:));
% make trajectory
t_f = 20;
w_des = 0; % rad/s
v_des = 0.2 ; % m/s
N_t = size(yT,2);
% get inputs for desired trajectories
w_traj = w_des*ones(1,N_t);
v_traj = v_des*ones(1,N_t);
U = [w_traj ; v_traj];
Z = [Sls(:,1:end-1); theta; v_traj];

% create turtlebot
%A = turtlebot_agent('LLC',turtlebot_local_LQR_LLC);
A = turtlebot_agent('LLC',turtlebot_iLQR_LLC);

% initial condition
%v_0 = 0.75 ; % m/s
% create the initial condition
z0 = Z(:,1) ; % (x,y,h,v)
z_init = [0;0;0;0.4];
A.reset(z_init)
T = linspace(0, t_f, N_t);

% move the robot
t_total = t_f ;
[states,inputs,k_feedforward,K_feedback, cost_arr] = A.LLC.iterate(z_init, T, U, Z, 2000);
A.move(t_total,T,U,Z)
A.animate()
hold_switch(hc)

% plotting error 
figure(5)
hc2 = hold_switch();
hold_switch(hc2);
X_real = match_trajectories(T,A.time,A.state(1:2,:));
X_des = Z(1:2,:);
pos_err = X_real - X_des;
plot(T,pos_err,'LineWidth',1.5);
legend('x_{err}','y_{err}')

function [xT,yT]=setR0T(Sls)
zz=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];
% xT and yT of the T link
path_prime = diff(Sls')'./vecnorm(diff(Sls')');
yT = [path_prime; zeros(1,size(path_prime,2))];
xT = -crossmat(ez)*yT;
end

function khat = crossmat(k)
  
if length(k)==3
  khat=[0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
elseif length(k)==6
  khat=[[0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0] k(4:6);zeros(1,4)];
else
    khat=[];
    disp('input vector is of wrong dimension! ');
end
end