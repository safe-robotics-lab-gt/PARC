function [Ux_1, Uy_1, r_1, s_1, e_1, dpsi_1, delta, Fx_f, Fx_r, Fy_f, Fy_r,...
    Fyf_max, Fyr_max, ax, ay, atot] =...
    simulate_step(delta_0, Fx_0, kappa, dt, veh, f_tire, r_tire, mode)
%SIMULATE_STEP
%   Take a single time step in simulation updating vehicle states.

%   Inputs
%       delta_0:    Current steer angle [rad]
%       Fx_0:       Current longitudinal force [N]
%       kappa:      Road curvature at current position [1/m]
%       dt:         Length of time step [s]
%       veh:        Vehicle parameters struct
%       f_tire:     Front tire parameters struct
%       r_tire:     Rear tire parameters struct
%       mode:       Simulation mode [0, 1, 2, 3]
%                   Mode 0 - Nonlinear bicycle model
%                   Mode 1 - Nonlinear bicycle model with actuator dynamics
%                   Mode 2 - Nonlinear bicycle model with actuator dynamics
%                            and measurement noise
%                   Mode 3 - Nonlinear bicycle model with actuator dynamics
%                            and measurement noise and hold at beginning of
%                            sim (Mode 2 and 3 equivalent in here)
%
%   Output:
%       Ux_1:       Updated longitudinal velocity [m/s]
%       Uy_1:       Updated lateral velocity [m/s]
%       r_1:        Updated yaw rate [rad/s]
%       s_1:        Updated distance along path [m]
%       e_1:        Updated error from path [m]
%       dpsi_1:     Update heading error [rad]

%--------------------------------------------------------------------------
%% PERSISTENT VARIABLES TO HOLD THE TRUE STATE
%--------------------------------------------------------------------------
persistent Ux_ground_truth;
persistent Uy_ground_truth;
persistent r_ground_truth;
persistent dpsi_ground_truth;
persistent s_ground_truth;
persistent e_ground_truth;

persistent brake_command_;
persistent brake_actual_;
persistent engine_command_;
persistent engine_actual_;
persistent delta_command_;
persistent delta_actual_;

persistent elapsed_time;

% initialize ground truth variables at the first function call
if isempty(Ux_ground_truth)
    
    Ux_ground_truth = 1;
    Uy_ground_truth = 0;
    r_ground_truth = 0;
    dpsi_ground_truth = deg2rad(2);
    s_ground_truth = 0;
    e_ground_truth = 0.15;
    
    % These vectors will hold the last 0.5 seconds of actual postitions and
    % commands. We will shift right and store the newest value in the
    % leftmost element each time through the simulation
    brake_command_ = zeros(1,0.5/dt);
    engine_command_ = zeros(1,0.5/dt);
    delta_command_ = zeros(1,0.5/dt);
    
    brake_actual_ = zeros(1,0.5/dt);
    engine_actual_ = zeros(1,0.5/dt);
    delta_actual_ = zeros(1,0.5/dt);
    
    elapsed_time = 0;
end

%--------------------------------------------------------------------------
%% SHIFT ACTUAL HISTORY
%--------------------------------------------------------------------------
brake_command_ = circshift(brake_command_, 1);
engine_command_ = circshift(engine_command_, 1);
delta_command_ = circshift(delta_command_, 1);

brake_actual_ = circshift(brake_actual_, 1);
engine_actual_ = circshift(engine_actual_, 1);
delta_actual_ = circshift(delta_actual_, 1);

%--------------------------------------------------------------------------
%% SPLIT LONGITUDINAL FORCES
%--------------------------------------------------------------------------
[Fx_lim] = Fx_limits(Fx_0, veh, f_tire, r_tire);

if Fx_lim > 0
    brake_command_(1) = 0;
    engine_command_(1) = Fx_lim;
else
    brake_command_(1) = Fx_lim;
    engine_command_(1) = 0;
end

delta_command_(1) = delta_0;

maxF_f = f_tire.mu*veh.Wf;
maxF_r = r_tire.mu*veh.Wr;
maxF_xf = veh.maxPower_W/Ux_ground_truth;

%--------------------------------------------------------------------------
%% IMPOSE ACTUATOR DYNAMICS & LIMITS
%--------------------------------------------------------------------------
if (mode >= 1)
    brake_actual_(1) = brakeDynamics(brake_command_, brake_actual_, veh, dt);
    engine_actual_(1) = engineDynamics(engine_command_, engine_actual_, Ux_ground_truth, veh, dt);
    delta_actual_(1) = steeringDynamics(delta_command_, delta_actual_, veh, dt);
else
    brake_actual_(1) = brake_command_(1);
    engine_actual_(1) = engine_command_(1);
    delta_actual_(1) = delta_command_(1);
end

% Calculate forces
Fx_f = engine_actual_(1)*veh.driveDistro(1) + brake_actual_(1)*veh.brakeDistro(1);
Fx_r = engine_actual_(1)*veh.driveDistro(2) + brake_actual_(1)*veh.brakeDistro(2);
delta = delta_actual_(1);


%----------------------------------------------------------------------
%% STEADY-STATE WEIGHT TRANSFER
%----------------------------------------------------------------------
if (mode >= 1)
    [Wf, Wr] = CalcWeightTransferLong(Fx_f + Fx_r, veh);
else
    Wf = veh.Wf;
    Wr = veh.Wr;
end

%----------------------------------------------------------------------
%% CALCULATE TIRE FORCES
%----------------------------------------------------------------------
[alpha_f, alpha_r] = slip_angles(Ux_ground_truth, ...
                                 Uy_ground_truth, ...
                                 r_ground_truth, ...
                                 delta, ...
                                 veh);


%----------------------------------------------------------------------
%% CALCULATE TIRE FORCES
%----------------------------------------------------------------------
if (mode >= 1)
    [Fy_f, Fyf_max] = Fy_CoupledFiala(alpha_f, Wf, Fx_f, f_tire);
    [Fy_r, Fyr_max] = Fy_CoupledFiala(alpha_r, Wr, Fx_r, r_tire);
else
    [Fy_f, Fyf_max] = Fy_Fiala(alpha_f, Wf, f_tire);
    [Fy_r, Fyr_max] = Fy_Fiala(alpha_r, Wr, r_tire);
end

%----------------------------------------------------------------------
%% CALCULATE STATE DERIVATIVES
%----------------------------------------------------------------------
[Ux_dot, Uy_dot, r_dot, s_dot, e_dot, dpsi_dot] =...
    nonlinear_bicycle_model(Fy_f, Fy_r, Fx_f, Fx_r, Ux_ground_truth, ...
                            Uy_ground_truth, r_ground_truth, e_ground_truth,...
                            dpsi_ground_truth, delta_0, kappa, veh);
                        
%--------------------------------------------------------------------------
%% CALCULATE VEHICLE ACCELERATIONS
%--------------------------------------------------------------------------
ax = Ux_dot - r_ground_truth*Uy_ground_truth;
ay = Uy_dot + r_ground_truth*Ux_ground_truth;
atot = sqrt(ax^2 + ay^2);

                        
%--------------------------------------------------------------------------
%% UPDATE VEHICLE STATES
%--------------------------------------------------------------------------
Ux_ground_truth = integrate_euler( Ux_ground_truth, Ux_dot, dt);
Uy_ground_truth = integrate_euler( Uy_ground_truth, Uy_dot, dt);
r_ground_truth  = integrate_euler( r_ground_truth,  r_dot,  dt);
e_ground_truth  = integrate_euler( e_ground_truth,  e_dot,  dt);
s_ground_truth  = integrate_euler( s_ground_truth,  s_dot,  dt);
dpsi_ground_truth  = integrate_euler( dpsi_ground_truth,  dpsi_dot,  dt);


%--------------------------------------------------------------------------
%% IF MODE 2, RETURN CORRUPTED STATE ESTIMATES
%--------------------------------------------------------------------------
Ux_1 = Ux_ground_truth;
Uy_1 = Uy_ground_truth;
r_1 = r_ground_truth;
e_1 = e_ground_truth;
s_1 = s_ground_truth;
dpsi_1 = dpsi_ground_truth;

if (mode > 1)
    [Ux_1, Uy_1, r_1, e_1, s_1, dpsi_1] = gps_model(Ux_ground_truth,...
                                                    Uy_ground_truth,...
                                                    r_ground_truth,...
                                                    e_ground_truth,...
                                                    s_ground_truth,...
                                                    dpsi_ground_truth,...
                                                    elapsed_time);
    % need to clamp s_1 >= 0 to not mess up lookup table for curvature
    s_1 = max(s_1, 0);
end



%--------------------------------------------------------------------------
%% UPDATE ELAPSED TIME FOR SENSOR MODELS
%--------------------------------------------------------------------------
elapsed_time = elapsed_time + dt;

end
