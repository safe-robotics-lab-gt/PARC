%% Toy Example Script: Van Der Pole system in reverse-time
%
% This script creates step-by-step illustrative examples of how to
% obtain piecewise-affine system using van der pole system as a planning
% model.
%
% Author: Wonsuhk Jung
% Created: 07 Feb 2024
% Updated: 11 Mar 2024
clear all; close all;

%% Step 0. Visualization Setting
palette = get_palette_colors();
rand_seed = 50;
rng(rand_seed);

axis_font_size = 20;
tick_font_size = 20;

%% Step 1. Define the states bound and center of affinization
B = make_set_cuboid3D('center', [0; 0; 0], 'thickness', 3, 'width', 4, 'height', 2);
S = sample_from_set('set', B, 'n_sample', 4);

figure(1);

plot(B, "color", 'gray', "alpha", 0.1, "linewidth", 3); hold on;
xlabel('$x$', 'interpreter', 'latex', 'FontSize', axis_font_size);
ylabel('$y$', 'interpreter', 'latex', 'FontSize', axis_font_size);
zlabel('$k$', 'interpreter', 'latex', 'FontSize', axis_font_size);
scatter3(S(1, :), S(2, :), S(3, :), 'red', 'filled'); 
set(gca, 'FontSize', tick_font_size);
hold off;


%% Step 2. Voronoi Partition
% vis_settings
mode_alpha     = 0.3;
mode_linewidth = 3.0;

% action
[V, pwa_modes] = mpt_voronoi(S, 'bound', B);

% plot
figure(2);
plot(pwa_modes, "alpha", mode_alpha, "linewidth", mode_linewidth);
xlabel('$x$', 'interpreter', 'latex', 'FontSize', axis_font_size);
ylabel('$y$', 'interpreter', 'latex', 'FontSize', axis_font_size);
zlabel('$k$', 'interpreter', 'latex', 'FontSize', axis_font_size);
set(gca, 'FontSize', tick_font_size);

%% Step 3. 3D vector field
% vis_settings
x_margin = 0.3;
y_margin = 0.3;
quiver_linewidth = 2.5;
quiver_color     = palette.black;

% action
x_range = linspace(-1.5+x_margin, 1.5-x_margin, 10);
y_range = linspace(-2.0+y_margin, 2.0-y_margin, 10);
k_range = linspace(-1, 1, 5);

[X, Y, K] = meshgrid(x_range, y_range, k_range);

DX = -exp(K).*(X-1/3*X.^3-Y);
DY = -X./exp(K);
DK = zeros(size(DX));

% plot
figure(3);
plot(pwa_modes, "alpha", mode_alpha, "linewidth", mode_linewidth); hold on;
q = quiver3(X, Y, K, DX, DY, DK, 'AutoScale', 'on', 'AutoScaleFactor', 4);
q.Color = quiver_color;
q.LineWidth = quiver_linewidth;

xlabel('$x$', 'interpreter', 'latex', 'FontSize', axis_font_size);
ylabel('$y$', 'interpreter', 'latex', 'FontSize', axis_font_size);
zlabel('$k$', 'interpreter', 'latex', 'FontSize', axis_font_size);
set(gca, 'FontSize', tick_font_size);
view(45, 15);
axis tight; grid on;
camlight;

%% Step 4. 3D PWA Vectorfield
[X, Y, K] = meshgrid(x_range, y_range, k_range);
% Initialize DX and DY matrices
DX_pwa = zeros(size(X));
DY_pwa = zeros(size(Y));
DK_pwa = zeros(size(K));

% Loop through each element in X and Y
for i = 1:numel(X)
    [dx, dy] = affinize_van_der_pol(X(i), Y(i), K(i), pwa_modes);
    DX_pwa(i) = dx;
    DY_pwa(i) = dy;
end

figure(4);
plot(pwa_modes, "alpha", mode_alpha, "linewidth", mode_linewidth); hold on;
q = quiver3(X, Y, K, DX_pwa, DY_pwa, DK_pwa, 'AutoScale', 'on', 'AutoScaleFactor', 4);
q.Color = quiver_color;
q.LineWidth = quiver_linewidth;


goal = make_set_square2D('center', [1; -0.25], 'side', 1);
goal = goal * Polyhedron('A', [1; -1], 'b', [1; 1]);
plot(goal, 'color', 'blue');

xlabel('$x$', 'interpreter', 'latex', 'FontSize', axis_font_size);
ylabel('$y$', 'interpreter', 'latex', 'FontSize', axis_font_size);
zlabel('$k$', 'interpreter', 'latex', 'FontSize', axis_font_size);
set(gca, 'FontSize', tick_font_size);
view(45, 15);
axis tight; grid on;
camlight;


%% 3D nonlinear vector field without modes
figure(5);

plot(B, "color", 'white', "alpha", mode_alpha, "linewidth", mode_linewidth); hold on;

q = quiver3(X, Y, K, DX, DY, DK, 'AutoScale', 'on', 'AutoScaleFactor', 4);
q.Color = quiver_color;
q.LineWidth = quiver_linewidth;

xlabel('$x$', 'interpreter', 'latex', 'FontSize', axis_font_size);
ylabel('$y$', 'interpreter', 'latex', 'FontSize', axis_font_size);
zlabel('$k$', 'interpreter', 'latex', 'FontSize', axis_font_size);

view(45, 15)
set(gca, 'FontSize', tick_font_size);
hold off;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% CHAPTER II. Expert Trajectory: Slice of vector field   %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% II-1. Sliced nonlinear vector field
% vis_settings
x_margin = 0.2;
y_margin = 0.2;

% action
k_expert         = 0.3;
pwa_modes_sliced = pwa_modes.slice(3, k_expert);

x_range          = linspace(-1.5+x_margin, 1.5-x_margin, 16);
y_range          = linspace(-2.0+y_margin, 2.0-y_margin, 16);
[X, Y]           = meshgrid(x_range, y_range);

% vanderpole nonlinear vector field
k  = k_expert;
DX_s = -exp(k) * (X-1/3*X.^3-Y);
DY_s = -X./exp(k);

figure(6);
plot(pwa_modes_sliced, "alpha", mode_alpha, "LineWidth", mode_linewidth); hold on;
q = quiver(X, Y, DX_s, DY_s, 'AutoScale', 'on', 'AutoScaleFactor', 1);
q.Color = quiver_color;
q.LineWidth = quiver_linewidth;
xlabel('$x$', 'interpreter', 'latex', 'FontSize', axis_font_size);
ylabel('$y$', 'interpreter', 'latex', 'FontSize', axis_font_size);
set(gca, 'FontSize', tick_font_size);
axis tight; grid on; axis equal;


%% II-2. Sliced pwa vector field visualized with the target set
% Initialize DX and DY matrices
DX_pwa_s = zeros(size(X));
DY_pwa_s = zeros(size(Y));

% Loop through each element in X and Y
for i = 1:numel(X)
    [dx, dy] = affinize_van_der_pol(X(i), Y(i), k_expert, pwa_modes);
    DX_pwa_s(i) = dx;
    DY_pwa_s(i) = dy;
end

figure(7);
plot(pwa_modes_sliced, "alpha", mode_alpha, "LineWidth", mode_linewidth); hold on;
plot(goal.projection(1:2), "color", 'blue');
q = quiver(X, Y, DX_pwa_s, DY_pwa_s, 'AutoScale', 'on', 'AutoScaleFactor', 1);
q.Color = quiver_color;
q.LineWidth = quiver_linewidth;
xlabel('$x$', 'interpreter', 'latex', 'FontSize', axis_font_size);
ylabel('$y$', 'interpreter', 'latex', 'FontSize', axis_font_size);
set(gca, 'FontSize', tick_font_size);
axis tight; grid on; axis equal;

%% II-3. Sliced pwa vector field visualized with the original vector field
% Initialize DX and DY matrices
DX_pwa_s = zeros(size(X));
DY_pwa_s = zeros(size(Y));

% Loop through each element in X and Y
for i = 1:numel(X)
    [dx, dy] = affinize_van_der_pol(X(i), Y(i), k_expert, pwa_modes);
    DX_pwa_s(i) = dx;
    DY_pwa_s(i) = dy;
end

figure(8);
plot(pwa_modes_sliced, "alpha", mode_alpha, "LineWidth", mode_linewidth); hold on;
q = quiver(X, Y, DX_pwa_s, DY_pwa_s, 'AutoScale', 'on', 'AutoScaleFactor', 1);
q.Color = quiver_color;
q.LineWidth = quiver_linewidth;

q_nl = quiver(X, Y, DX_s, DY_s, 'AutoScale', 'on', 'AutoScaleFactor', 1);
q_nl.Color = 'red';
q_nl.LineWidth = quiver_linewidth;

xlabel('$x$', 'interpreter', 'latex', 'FontSize', axis_font_size);
ylabel('$y$', 'interpreter', 'latex', 'FontSize', axis_font_size);
set(gca, 'FontSize', tick_font_size);
axis tight; grid on; axis equal; hold off;

%% Auxilary functions
function [dx, dy] = affinize_van_der_pol(x, y, k, modes)
% Piecewise-affinize the vanderpole system with analytic formula.
for idx = 1:length(modes)
    mode_i = modes(idx);
    if mode_i.contains([x; y; k])
        c_i = mode_i.interiorPoint.x;
        x0 = c_i(1); y0 = c_i(2); k0 = c_i(3);

        mu0 = exp(k0); mu = exp(k);

        dx = -(mu0*(1-x0^2)*x - mu0*y + (x0-1/3*x0^3-y0)*mu + mu0*x0^3-mu0*x0+mu0*y0);
        dy = -x/mu0 + x0/mu0^2*mu - x0/mu0;
        break
    end
end
end

