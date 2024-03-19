%% Initializing script for 2D-quadrotor with polynomial model
% Corresponds to Appendix G in the PARC paper
% See Mueller et al. "A computationally efficient motion primitive for
% quadrocopter trajectory generation" for details of the polynomial
% planning model
% Author: Long Kiu Chung
% Created: 2024/02/05
% Updated: 2024/03/14

%% Parameters (User Defined)
% Drone parameters
g = 9.81; % gravity
I = 0.01; % inertia
r = 0.25; % radius

% Discretization parameters
t_pk = 1.5; % peak time
t_f = 2; % final time
dt = 0.1; % timestep

% World bounds
% Assume world bound is hyperrectangle
x_lo = -2;
x_hi = 2;
z_lo = -0.3;
z_hi = 2;
k_v_x_lo = -1.5; % Initial velocity in x
k_v_x_hi = 1.5;
k_a_x_lo = 0; % Initial acceleration in x
k_a_x_hi = 0; 
k_pk_x_lo = -0.1; % Peak velocity in x
k_pk_x_hi = 0.1;
k_v_z_lo = 0; % Initial velocity in z
k_v_z_hi = 0;
k_a_z_lo = -1; % Initial acceleration in z
k_a_z_hi = 1;
k_pk_z_lo = -1.5; % Peak velocity in z
k_pk_z_hi = 1.5;

% Goals and obstacles
% Here I'm using Polyhedron() instead of make_set_rectangle2D. The rotation
% matrix makes the computation numerically unstable
X_goal_x = Polyhedron('A', [1; -1], 'b', [sqrt(2).*0.15; sqrt(2).*0.15]);
X_goal_z = Polyhedron('A', [1; -1], 'b', [sqrt(2).*0.15; sqrt(2).*0.15]);
O_1_x = Polyhedron('A', [1; -1], 'b', [-0.5, 1]);
O_1_z = Polyhedron('A', [1; -1], 'b', [0.5, 0.3]);
O_2_x = Polyhedron('A', [1; -1], 'b', [1, 0]);
O_2_z = Polyhedron('A', [1; -1], 'b', [1.4, -0.8]);

%% Helpful Variables (Dependent on User-Defined Parameters)
% Discretization parameters
t_n = floor(t_f./dt); % number of timesteps

% World bounds
X = Polyhedron('A', [1; -1], 'b', [x_hi; -x_lo]);
Z = Polyhedron('A', [1; -1], 'b', [z_hi; -z_lo]);
K_v_x = Polyhedron('A', [1; -1], 'b', [k_v_x_hi; (-k_v_x_lo)]);
K_a_x = Polyhedron('A', [1; -1], 'b', [k_a_x_hi; (-k_a_x_lo)]);
K_pk_x = Polyhedron('A', [1; -1], 'b', [k_pk_x_hi; (-k_pk_x_lo)]);
K_v_z = Polyhedron('A', [1; -1], 'b', [k_v_z_hi; (-k_v_z_lo)]);
K_a_z = Polyhedron('A', [1; -1], 'b', [k_a_z_hi; (-k_a_z_lo)]);
K_pk_z = Polyhedron('A', [1; -1], 'b', [k_pk_z_hi; (-k_pk_z_lo)]);
K = [K_v_x*K_a_x*K_pk_x; K_v_z*K_a_z*K_pk_z];

R_x = X*K(1);
R_z = Z*K(2);

% Goals and obstacles
G = [X_goal_x; X_goal_z];
X_goal = X_goal_x*X_goal_z;
O_1 = O_1_x*O_1_z;
O_2 = O_2_x*O_2_z;
O(1, 1:2) = [O_1_x, O_2_x];
O(2, 1:2) = [O_1_z, O_2_z];

%% Define PWA Model
ltis_x = PolynomialLTIQuad('peak_time', t_pk, 'final_time', t_f, 'timestep', dt, 'domain', R_x);
ltis_z = PolynomialLTIQuad('peak_time', t_pk, 'final_time', t_f, 'timestep', dt, 'domain', R_z);
ltis = [ltis_x; ltis_z];
