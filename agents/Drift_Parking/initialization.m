% Initialize hardware and software parameters for drifting
root_dir = what(fullfile('PARC','agents','Drift_Parking'), '-all').path;
cd(root_dir)

% Vehicle parameters (Marty)
veh.Caf = 60000;      % [N/rad]
veh.Car = 160000;     % [N/rad]
veh.m  = 1450;        % [kg]
veh.Iz = 2300;        % [m^2]
veh.L  = 2.4;         % [m]
veh.a  = 0.67*veh.L;  % [m]
veh.b  = veh.L-veh.a; % [m]
veh.mu = 1.1;         % [-]
tire_f.Ca = veh.Caf;
tire_f.mu = 1.1;
tire_r.Ca = veh.Car;
tire_r.mu = 1.1;

veh.body_L = 4.267;
veh.body_W = 1.988;

% Constants
g   = 9.81;       % [m/s^2]

% Compute the static normal load
tire_f.Fz = veh.m * g * veh.b/veh.L;
tire_r.Fz = veh.m * g * veh.a/veh.L;

% set input bounds
delta_max = 38 * pi / 180; % [rad]
Fxf_max = min(tire_f.mu*tire_f.Fz-1,7684); % [N] *6684
Fxr_max = min(tire_r.mu*tire_r.Fz-1,7684); % [N]

% create vertices of the car for plotting
vt.center = 0;%-veh.b + veh.L/2;
vt.aw = 0.35;
vt.bw = veh.body_W/2;
vt.cw = 0.25;
vt.dw = 0.7;
vt.ew = vt.bw;
vt.al = vt.center-veh.body_L/2;
vt.bl = -0.3;
vt.cl = 0.9;
vt.dl = vt.cl+0.2;
vt.el = vt.center+veh.body_L/2;
vt.fl = vt.center+veh.body_L/2;
vt.gl = vt.al + 0.74;
veh.body_vertices = ...
    [vt.al,vt.gl,vt.gl, vt.bl, vt.bl, vt.cl, vt.dl, vt.el,vt.el,vt.dl,vt.cl,vt.bl,vt.bl,vt.gl,vt.gl,vt.al, vt.al;
    -vt.ew,-vt.ew,-vt.aw,-vt.aw,-vt.bw,-vt.bw,-vt.aw,-vt.cw,vt.cw,vt.aw,vt.bw,vt.bw,vt.aw,vt.aw,vt.ew,vt.ew,-vt.ew];
veh.body_color = [0 0.4470 0.7410];

veh.tire_color  = [120 120 120]/255;
vt.tire_size = [0.6, 0.3];
vt.tire_single = [-vt.tire_size(1)/2, vt.tire_size(1)/2, vt.tire_size(1)/2, -vt.tire_size(1)/2,-vt.tire_size(1)/2;
    vt.tire_size(2)/2, vt.tire_size(2)/2, -vt.tire_size(2)/2, -vt.tire_size(2)/2,vt.tire_size(2)/2];
vt.t_diag = norm(vt.tire_size)/2;
vt.t_ang = atan2(vt.tire_size(2), vt.tire_size(1));
vt.t_w = veh.body_W/2-vt.t_diag*sin(vt.t_ang+delta_max); % ensuring max steering within car width
vt.tire_loc = [-veh.b, -veh.b, veh.a, veh.a;
    -vt.t_w,   vt.t_w,  vt.t_w, -vt.t_w];
veh.tire_f_vertices = [];
veh.tire_r_vertices = [];
for i = 1:4
    if i == 3 || i == 4
        veh.tire_f_vertices = [veh.tire_f_vertices, vt.tire_single+repmat(vt.tire_loc(:,i),[1,5]) [NaN;NaN]];
    else
        veh.tire_r_vertices = [veh.tire_r_vertices, vt.tire_single+repmat(vt.tire_loc(:,i),[1,5]) [NaN;NaN]];
    end
end

%% plot car
% figure; axis equal; ax = gca;
% plot_car(ax, [0,0], 0, delta_max, veh);