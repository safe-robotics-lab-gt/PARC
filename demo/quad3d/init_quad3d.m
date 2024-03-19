%% Initializing script for PARC on quadrotor with 12D polynomial
% Author: Long Kiu Chung
% Created: 2024/01/14
% Updated: 2024/03/18

%% Parameters
% USER-DEFINED PARAMETERS
% Discretization parameters
t_pk = 1;
t_f = 3;
dt = 0.02;

% World bounds
x_lo = 0;
x_hi = 10;
y_lo = -10;
y_hi = 10;
z_lo = 0;
z_hi = 10;
% Assume all k-parameter bounds the same across x, y, z
k_v_lo = -5.25; % Initial velocity
k_v_hi = 5.25;
k_a_lo = -10; % Initial acceleration
k_a_hi = 10;
k_pk_lo = -5.25; % Peak velocity
k_pk_hi = 5.25;

% Drone parameters
V_X = Polyhedron('A', [1; -1], 'b', [0.54*0.5; 0.54*0.5]); % Drone volume
V_Y = V_X;
V_Z = V_X;
% Initial condition
x_0 = 1;
y_0 = 0;
z_0 = 5;

% Goal set
G_X = Polyhedron('A', [1; -1], 'b', [(8.5 + 1.5.*sqrt(2)*0.5); (-(8.5 - 1.5.*sqrt(2)*0.5))]);
G_Y = Polyhedron('A', [1; -1], 'b', [(1.5.*sqrt(2)*0.5); (-(-1.5.*sqrt(2)*0.5))]);
G_Z = Polyhedron('A', [1; -1], 'b', [(5 + 1.5.*sqrt(2)*0.5); (-(5 - 1.5.*sqrt(2)*0.5))]);

% Obstacles before accounting for drone volume
O_n = 2; % Number of obstacles

O_1_real_X = Polyhedron('A', [1; -1], 'b', [(5 + 3./2); (-(5 - 3./2))]);
O_1_real_Y = Polyhedron('A', [1; -1], 'b', [(-4.75 + 8.5./2); (-(-4.75 - 8.5./2))]);
O_1_real_Z = Polyhedron('A', [1; -1], 'b', [(5 + 8./2); (-(5 - 8./2))]);

O_2_real_X = Polyhedron('A', [1; -1], 'b', [(5 + 3./2); (-(5 - 3./2))]);
O_2_real_Y = Polyhedron('A', [1; -1], 'b', [(4.75 + 8.5./2); (-(4.75 - 8.5./2))]);
O_2_real_Z = Polyhedron('A', [1; -1], 'b', [(5 + 8./2); (-(5 - 8./2))]);

% Avoid set method
avoid_set_method = "no_convex_hull";
% avoid_set_method = "convex_hull";
no_convex_hull_condition_check = false; % Whether to enable check if no_convex_hull method can be used


% HELPFUL VARIABLES FROM USER-DEFINED PARAMETERS
% Discretization parameters
t_n = floor(t_f./dt);

% World bounds
X = Polyhedron('A', [1; -1], 'b', [x_hi; -x_lo]);
Y = Polyhedron('A', [1; -1], 'b', [y_hi; -y_lo]);
Z = Polyhedron('A', [1; -1], 'b', [z_hi; -z_lo]);
K_v = Polyhedron('A', [1; -1], 'b', [k_v_hi; (-k_v_lo)]);
K_a = Polyhedron('A', [1; -1], 'b', [k_a_hi; (-k_a_lo)]);
K_pk = Polyhedron('A', [1; -1], 'b', [k_pk_hi; (-k_pk_lo)]);
K = K_v*K_a*K_pk;

% Drone parameters
int_con = [x_0; y_0; z_0];

% Domain
XK = X*K;
YK = Y*K;
ZK = Z*K;
R = [XK; YK; ZK];

% Goals and obstacles
G = [G_X; G_Y; G_Z];
O_1_real = [O_1_real_X; O_1_real_Y; O_1_real_Z];
O_2_real = [O_2_real_X; O_2_real_Y; O_2_real_Z];
Os(3, O_n) = Polyhedron;

% Account for drone volume in obstacles
O_1 = [O_1_real_X + V_X; O_1_real_Y + V_Y; O_1_real_Z + V_Z];
O_2 = [O_2_real_X + V_X; O_2_real_Y + V_Y; O_2_real_Z + V_Z];

Os(:, 1) = O_1;
Os(:, 2) = O_2;

% Define LTI systems
ltis_x = PolynomialLTIQuad('peak_time', t_pk, 'final_time', t_f, 'timestep', dt, 'domain', R(1));
ltis_y = PolynomialLTIQuad('peak_time', t_pk, 'final_time', t_f, 'timestep', dt, 'domain', R(2));
ltis_z = PolynomialLTIQuad('peak_time', t_pk, 'final_time', t_f, 'timestep', dt, 'domain', R(3));
ltis = [ltis_x, ltis_y, ltis_z];
