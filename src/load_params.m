function [veh, pth, sim_opt] = load_params(scenario)
%LOAD_PARAMS  Design parameters of the baseline path-following project.
%
%   [veh, pth, sim_opt] = load_params()            baseline scenario
%   [veh, pth, sim_opt] = load_params(scenario)
%
%   veh     : vehicle parameters (single-track / bicycle model)
%   pth     : S-shaped (lane-change) path parameters
%   sim_opt : simulation options (initial state, duration, step size)
%
%   scenario : 'baseline'           Xoff = 80 m, eta = 40 m  (baseline project)
%              'sharp_lane_change'  Xoff = 50 m, eta = 20 m  (README Example 3)

if nargin < 1 || isempty(scenario), scenario = 'baseline'; end

% --- Vehicle (single-track model, cornering stiffness is per axle) -------
veh.lf  = 1.4;          % [m]       CG -> front axle
veh.lr  = 1.6;          % [m]       CG -> rear axle
veh.m   = 1600;         % [kg]      vehicle mass
veh.Izz = 2000;         % [kg m^2]  yaw moment of inertia
veh.Cf  = 50000;        % [N/rad]   front axle cornering stiffness
veh.Cr  = 50000;        % [N/rad]   rear axle cornering stiffness
veh.Vx  = 20;           % [m/s]     constant cruising (longitudinal) speed
veh.delta_max = deg2rad(30);  % [rad] road-wheel steering angle limit
veh.g   = 9.81;         % [m/s^2]   gravity (static axle loads of the Simulink block)

% --- Parametrisation of the Simulink "Vehicle Body 3DOF Single Track" block --
%   The block scales the cornering stiffness with the axle load,
%       Fy_axle = Cy * alpha * Fz_axle / Fznom        (see vdb_block_params).
%   'match_design' : Cy_f, Cy_r, Fznom chosen so that the EFFECTIVE per-axle
%                    stiffnesses at the static loads equal Cf, Cr above
%                    -> the plant is the vehicle the controller was designed for
%   'raw'          : Cf, Cr typed directly into Cy_f, Cy_r with block defaults
%                    (Fznom = 5000 N, aero on) -> ~1.7 Cf / ~1.5 Cr effective,
%                    used as a robustness check in run_baseline
veh.block_mode = 'match_design';

% --- S-shaped path: Y = WH*tanh((X - Xoff)/eta) + WH ----------------------
pth.WH = 2;             % [m]
switch lower(scenario)
    case 'baseline'
        pth.Xoff = 80;  % [m]
        pth.eta  = 40;  % [m]
    case 'sharp_lane_change'
        pth.Xoff = 50;  % [m]   larger path curvature (README Example 3)
        pth.eta  = 20;  % [m]
    otherwise
        error('load_params:scenario', ...
              'Unknown scenario "%s" (use ''baseline'' or ''sharp_lane_change'').', scenario);
end
pth.scenario = lower(scenario);

% --- Simulation ------------------------------------------------------------
sim_opt.Tend = 15;                  % [s]  required: at least 15 s
sim_opt.dt   = 1e-3;                % [s]  fixed step (RK4 / ode4)
sim_opt.z0   = [0; 0; 0; 0; 0];     % [X; Y; psi; Vy; r] start at origin, zero heading
end
