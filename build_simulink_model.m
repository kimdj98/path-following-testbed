function mdl_file = build_simulink_model(mdl)
%BUILD_SIMULINK_MODEL  Create the baseline Simulink model programmatically.
%
%   mdl_file = build_simulink_model()                -> path_following_baseline.slx
%   mdl_file = build_simulink_model('other_name')
%
%   Block diagram:
%
%      State Feedback Controller --WhlAngF--> Vehicle Body 3DOF Single Track
%      (Vehicle Dynamics Blockset, "External longitudinal velocity", v_x = 20 m/s)
%         --[xdot, ydot, psi]--> Vehicle Global Coordinate --[X, Y]-->
%         Path Tracking Errors --[y_e, y_e_dot, psi_e, psi_e_dot]--> controller
%
%   Layout: forward path left -> right on the top row, the Path Tracking
%   Errors block is flipped (inputs on the right) so that the feedback flows
%   right -> left along the bottom row.  Signals are recorded with signal
%   logging (out.logsout) instead of To Workspace blocks, which keeps the
%   diagram free of logging wires; simulink_log_to_struct reads them back.
%
%   The two custom blocks (Vehicle Global Coordinate, Path Tracking Errors)
%   are MATLAB Function blocks that call the functions in src/, so exactly
%   the same code is shared with the pure-MATLAB simulation.  Block
%   parameters reference base-workspace variables created by
%   init_model_workspace (veh, blk, pth_vec, Vx, K, delta_max, Ts_mon,
%   xy_monitor_on); the model's PreLoadFcn callback creates them when the
%   model opens.
%
%   Requires: Simulink, Control System Toolbox, Vehicle Dynamics Blockset.

if nargin < 1, mdl = 'path_following_baseline'; end
proj = fileparts(mfilename('fullpath'));
addpath(fullfile(proj, 'src'));
init_model_workspace();

mdl_file = fullfile(proj, [mdl '.slx']);
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist(mdl_file, 'file'), delete(mdl_file); end          % generated file: rebuild from scratch

load_system('vehdynlibeom');                                % Vehicle Dynamics Blockset library
new_system(mdl);

% ---------------------------------------------------------------- settings
set_param(mdl, 'Solver', 'ode4', 'FixedStep', '1e-3', 'StopTime', '15', ...
               'SaveTime', 'on', 'TimeSaveName', 'tout', 'ReturnWorkspaceOutputs', 'on', ...
               'SignalLogging', 'on', 'SignalLoggingName', 'logsout', ...
               'SignalLoggingSaveFormat', 'Dataset');
set_param(mdl, 'PreLoadFcn', sprintf(['addpath(fullfile(fileparts(which(''%s'')), ''src''));' ...
                                      ' init_model_workspace;'], mdl));
set_param(mdl, 'Description', ['Baseline project: vehicle path-following control of an ' ...
        'S-shaped lane change with full-state feedback (Vehicle Body 3DOF Single Track plant).']);

% ------------------------------------------------------------ block names
ctrlS = [mdl '/State Feedback Controller'];
vehB  = [mdl '/Nonlinear Vehicle Body 3DOF Model'];
gcS   = [mdl '/Vehicle Global Coordinate'];
errS  = [mdl '/Path Tracking Errors'];
monS  = [mdl '/X-Y Monitor'];

% ------------------------------------------------- top row: forward path
build_controller_subsystem(ctrlS, [150 110 330 230]);

% NOTE: the block computes Fy_axle = Cy * alpha * Fz_axle / Fznom, so the
% cornering stiffness, nominal load and aero coefficients come from
% vdb_block_params (base-workspace struct blk) to obtain the effective
% per-axle stiffnesses Cf, Cr of the design model ('match_design') or the
% naive parametrisation ('raw').  See src/vdb_block_params.m.
add_block('vehdynlibeom/Vehicle Body 3DOF Single Track', vehB, 'Position', [500 60 720 320]);
set_param(vehB, 'inputMode', 'External longitudinal velocity', ...
                'm',    'veh.m',    'a',    'veh.lf',   'b',   'veh.lr',   'Izz', 'veh.Izz', 'g', 'veh.g', ...
                'Cy_f', 'blk.Cy_f', 'Cy_r', 'blk.Cy_r', 'Fznom', 'blk.Fznom', ...
                'Cd',   'blk.Cd',   'Cl',   'blk.Cl',   'Cpm', 'blk.Cpm', ...
                'X_o', '0', 'Y_o', '0', 'psi_o', '0', 'r_o', '0', 'ydot_o', '0');

add_block('simulink/Sources/Constant', [mdl '/v_x'], 'Value', 'Vx', 'Position', [400 218 440 248]);
add_block('simulink/Sinks/Scope', [mdl '/Scope steering'], 'NumInputPorts', '1', 'Position', [400 60 430 90]);

build_global_coordinate_subsystem(gcS, [900 130 1060 230]);

add_block('simulink/Sinks/Terminator', [mdl '/term_Info'], 'Position', [780  82 800 102]);
add_block('simulink/Sinks/Terminator', [mdl '/term_FzF'],  'Position', [780 245 800 265]);
add_block('simulink/Sinks/Terminator', [mdl '/term_FzR'],  'Position', [780 278 800 298]);

% ---------------------------------------------- bottom row: feedback path
build_path_error_subsystem(errS, [500 430 720 590]);
set_param(errS, 'Orientation', 'left');                    % inputs on the right, outputs on the left
add_block('simulink/Sinks/Terminator', [mdl '/term_path_info'], 'Orientation', 'left', ...
          'Position', [450 556 470 576]);

% ------------------------------------- monitoring (fed by global Goto tags
% X_glob, Y_glob, psi, y_e, psi_e that are set inside the two custom
% subsystems, so that the top level only shows the control-loop wiring)
build_xy_monitor_subsystem(monS, [150 465 330 600]);
add_block('simulink/Signal Routing/From', [mdl '/from_X'],   'GotoTag', 'X_glob', 'Position', [60 478 110 498]);
add_block('simulink/Signal Routing/From', [mdl '/from_Y'],   'GotoTag', 'Y_glob', 'Position', [60 508 110 528]);
add_block('simulink/Signal Routing/From', [mdl '/from_psi'], 'GotoTag', 'psi',    'Position', [60 538 110 558]);
add_block('simulink/Signal Routing/From', [mdl '/from_y_e'], 'GotoTag', 'y_e',    'Position', [60 568 110 588]);
add_block('simulink/Sinks/Scope', [mdl '/Scope errors'], 'NumInputPorts', '2', 'Position', [230 620 260 650]);
add_block('simulink/Signal Routing/From', [mdl '/from_y_e2'],  'GotoTag', 'y_e',   'Position', [120 615 170 635]);
add_block('simulink/Signal Routing/From', [mdl '/from_psi_e'], 'GotoTag', 'psi_e', 'Position', [120 640 170 660]);

% ------------------------------------------------------------------ wiring
% wire(src, dst, name, log): named lines get their signal name; log = true
% marks the signal for logging (available as out.logsout after a run)
V = 'Nonlinear Vehicle Body 3DOF Model';
wire(mdl, 'State Feedback Controller/1', [V '/1'],                     'WhlAngF',  true);
wire(mdl, 'State Feedback Controller/1', 'Scope steering/1',           '',         false);
wire(mdl, 'v_x/1',                       [V '/2'],                     'xdotin',   false);

wire(mdl, [V '/1'], 'term_Info/1',                  'Info',  true);
wire(mdl, [V '/2'], 'Vehicle Global Coordinate/1',  'xdot',  false);
wire(mdl, [V '/2'], 'Path Tracking Errors/4',       '',      false);
wire(mdl, [V '/3'], 'Vehicle Global Coordinate/2',  'ydot',  true);
wire(mdl, [V '/3'], 'Path Tracking Errors/5',       '',      false);
wire(mdl, [V '/4'], 'Vehicle Global Coordinate/3',  'psi',   true);
wire(mdl, [V '/4'], 'Path Tracking Errors/3',       '',      false);
wire(mdl, [V '/5'], 'Path Tracking Errors/6',       'r',     true);
wire(mdl, [V '/6'], 'term_FzF/1',                   'FzF',   false);
wire(mdl, [V '/7'], 'term_FzR/1',                   'FzR',   false);

wire(mdl, 'Vehicle Global Coordinate/1', 'Path Tracking Errors/1', 'X_glob', true);
wire(mdl, 'Vehicle Global Coordinate/2', 'Path Tracking Errors/2', 'Y_glob', true);

wire(mdl, 'Path Tracking Errors/1', 'State Feedback Controller/1', 'y_e',       true);
wire(mdl, 'Path Tracking Errors/2', 'State Feedback Controller/2', 'y_e_dot',   true);
wire(mdl, 'Path Tracking Errors/3', 'State Feedback Controller/3', 'psi_e',     true);
wire(mdl, 'Path Tracking Errors/4', 'State Feedback Controller/4', 'psi_e_dot', true);
wire(mdl, 'Path Tracking Errors/5', 'term_path_info/1',            'path_info', true);

wire(mdl, 'from_X/1',     'X-Y Monitor/1',  '', false);
wire(mdl, 'from_Y/1',     'X-Y Monitor/2',  '', false);
wire(mdl, 'from_psi/1',   'X-Y Monitor/3',  '', false);
wire(mdl, 'from_y_e/1',   'X-Y Monitor/4',  '', false);
wire(mdl, 'from_y_e2/1',  'Scope errors/1', '', false);
wire(mdl, 'from_psi_e/1', 'Scope errors/2', '', false);

% tidy the routing of all top-level lines
try
    Simulink.BlockDiagram.routeLine(find_system(mdl, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'line'));
catch
end

% From / Goto blocks already display their tag and terminators need no label:
% hide the block names so they do not collide with the signal names
hide = [find_system(mdl, 'LookUnderMasks', 'none', 'FollowLinks', 'off', 'BlockType', 'From'); ...
        find_system(mdl, 'LookUnderMasks', 'none', 'FollowLinks', 'off', 'BlockType', 'Goto'); ...
        find_system(mdl, 'LookUnderMasks', 'none', 'FollowLinks', 'off', 'BlockType', 'Terminator')];
for k = 1:numel(hide), set_param(hide{k}, 'ShowName', 'off'); end

try
    note = Simulink.Annotation([mdl '/Baseline: path following of an S-shaped lane change, ' ...
        'V = 20 m/s, u = -K x with x = [e_y, e_y_dot, e_psi, e_psi_dot]']);
    note.Position = [150 20];
catch
end

save_system(mdl, mdl_file);
fprintf('Simulink model written: %s\n', mdl_file);

% picture of the diagrams for the report / README
try
    res = fullfile(proj, 'results'); if ~exist(res, 'dir'), mkdir(res); end
    print(['-s' mdl],  '-dpng', '-r110', fullfile(res, 'simulink_diagram.png'));
    print(['-s' monS], '-dpng', '-r110', fullfile(res, 'simulink_xy_monitor.png'));
catch
end
end

% =========================================================================
function wire(mdl, src, dst, name, log_signal)
h = add_line(mdl, src, dst, 'autorouting', 'smart');
if ~isempty(name), set_param(h, 'Name', name); end
if log_signal, set_param(get_param(h, 'SrcPortHandle'), 'DataLogging', 'on'); end
end

function set_fcn_script(blk, script)
ch = find(slroot, '-isa', 'Stateflow.EMChart', 'Path', blk);
ch.Script = script;
end

% ---------------------------------------------------------------- controller
function build_controller_subsystem(sub, pos)
% u = -K x, x = [y_e; y_e_dot; psi_e; psi_e_dot], saturated at +-delta_max
add_block('built-in/Subsystem', sub, 'Position', pos);
names = {'y_e', 'y_e_dot', 'psi_e', 'psi_e_dot'};
for i = 1:4
    add_block('built-in/Inport', [sub '/' names{i}], 'Port', num2str(i), ...
              'Position', [30 40*i 60 40*i+14]);
end
add_block('simulink/Signal Routing/Mux', [sub '/x'], 'Inputs', '4', 'DisplayOption', 'bar', ...
          'Position', [130 40 135 190]);
add_block('simulink/Math Operations/Gain', [sub '/Gain -K'], 'Gain', '-K', ...
          'Multiplication', 'Matrix(K*u)', 'Position', [190 100 250 130]);
add_block('simulink/Discontinuities/Saturation', [sub '/steering limit'], ...
          'UpperLimit', 'delta_max', 'LowerLimit', '-delta_max', 'Position', [300 100 350 130]);
add_block('built-in/Outport', [sub '/WhlAngF'], 'Port', '1', 'Position', [400 108 430 122]);

for i = 1:4, add_line(sub, [names{i} '/1'], sprintf('x/%d', i), 'autorouting', 'smart'); end
add_line(sub, 'x/1', 'Gain -K/1');
add_line(sub, 'Gain -K/1', 'steering limit/1');
add_line(sub, 'steering limit/1', 'WhlAngF/1');
end

% -------------------------------------------------------- global coordinate
function build_global_coordinate_subsystem(sub, pos)
% [Xdot; Ydot] = R(psi) [v_x; v_y]  ->  integrate  ->  X, Y   (start at 0,0)
add_block('built-in/Subsystem', sub, 'Position', pos);
add_block('built-in/Inport', [sub '/v_x'], 'Port', '1', 'Position', [30 40 60 54]);
add_block('built-in/Inport', [sub '/v_y'], 'Port', '2', 'Position', [30 80 60 94]);
add_block('built-in/Inport', [sub '/psi'], 'Port', '3', 'Position', [30 120 60 134]);

fcn = [sub '/kinematics'];
add_block('simulink/User-Defined Functions/MATLAB Function', fcn, 'Position', [120 40 240 140]);
set_fcn_script(fcn, sprintf([ ...
    'function [dX, dY] = kinematics(v_x, v_y, psi)\n' ...
    '%%#codegen\n' ...
    '%% Body-frame velocity -> inertial-frame velocity of the CG\n' ...
    '%% (see src/global_coordinate.m)\n' ...
    '[dX, dY] = global_coordinate(v_x, v_y, psi);\n']));

add_block('simulink/Continuous/Integrator', [sub '/int_X'], 'InitialCondition', '0', 'Position', [290 45 320 75]);
add_block('simulink/Continuous/Integrator', [sub '/int_Y'], 'InitialCondition', '0', 'Position', [290 105 320 135]);
add_block('built-in/Outport', [sub '/X'], 'Port', '1', 'Position', [370 53 400 67]);
add_block('built-in/Outport', [sub '/Y'], 'Port', '2', 'Position', [370 113 400 127]);

add_line(sub, 'v_x/1', 'kinematics/1'); add_line(sub, 'v_y/1', 'kinematics/2'); add_line(sub, 'psi/1', 'kinematics/3');
add_line(sub, 'kinematics/1', 'int_X/1'); add_line(sub, 'kinematics/2', 'int_Y/1');
add_line(sub, 'int_X/1', 'X/1');          add_line(sub, 'int_Y/1', 'Y/1');

% global tags for the monitoring blocks at the top level (X-Y Monitor):
% CG position X, Y and the yaw angle psi (heading of the ego-vehicle glyph)
add_block('simulink/Signal Routing/Goto', [sub '/goto_X'], 'GotoTag', 'X_glob', 'TagVisibility', 'global', ...
          'Position', [370 160 420 180]);
add_block('simulink/Signal Routing/Goto', [sub '/goto_Y'], 'GotoTag', 'Y_glob', 'TagVisibility', 'global', ...
          'Position', [370 190 420 210]);
add_block('simulink/Signal Routing/Goto', [sub '/goto_psi'], 'GotoTag', 'psi', 'TagVisibility', 'global', ...
          'Position', [370 220 420 240]);
add_line(sub, 'int_X/1', 'goto_X/1', 'autorouting', 'smart');
add_line(sub, 'int_Y/1', 'goto_Y/1', 'autorouting', 'smart');
add_line(sub, 'psi/1',   'goto_psi/1', 'autorouting', 'smart');
end

% ------------------------------------------------------ path tracking errors
function build_path_error_subsystem(sub, pos)
% e_y, e_y_dot, e_psi, e_psi_dot of the CG w.r.t. the S-shaped path
add_block('built-in/Subsystem', sub, 'Position', pos);
in = {'X_glob', 'Y_glob', 'ego_psi', 'v_x', 'v_y', 'r'};
for i = 1:6
    add_block('built-in/Inport', [sub '/' in{i}], 'Port', num2str(i), 'Position', [30 40*i 60 40*i+14]);
end
add_block('simulink/Sources/Constant', [sub '/path params [WH Xoff eta]'], 'Value', 'pth_vec', ...
          'Position', [10 290 80 310]);

fcn = [sub '/errors'];
add_block('simulink/User-Defined Functions/MATLAB Function', fcn, 'Position', [150 40 330 310]);
set_fcn_script(fcn, sprintf([ ...
    'function [y_e, y_e_dot, psi_e, psi_e_dot, path_info] = errors(X_glob, Y_glob, ego_psi, v_x, v_y, r, pth)\n' ...
    '%%#codegen\n' ...
    '%% Lateral offset and heading error of the CG w.r.t. the S-shaped path\n' ...
    '%%   Y = WH*tanh((X - Xoff)/eta) + WH,   pth = [WH, Xoff, eta]\n' ...
    '%% path_info = [Xp, Yp, psi_p, kappa, psi_dot_des]  (see src/path_tracking_errors.m)\n' ...
    '[y_e, y_e_dot, psi_e, psi_e_dot, path_info] = path_tracking_errors(X_glob, Y_glob, ego_psi, v_x, v_y, r, pth);\n']));

out = {'y_e', 'y_e_dot', 'psi_e', 'psi_e_dot', 'path_info'};
for i = 1:5
    add_block('built-in/Outport', [sub '/' out{i}], 'Port', num2str(i), 'Position', [400 50*i 430 50*i+14]);
end
for i = 1:6, add_line(sub, [in{i} '/1'], sprintf('errors/%d', i), 'autorouting', 'smart'); end
add_line(sub, 'path params [WH Xoff eta]/1', 'errors/7', 'autorouting', 'smart');
for i = 1:5, add_line(sub, sprintf('errors/%d', i), [out{i} '/1'], 'autorouting', 'smart'); end

% global tags for the monitoring blocks at the top level (X-Y Monitor, Scope errors)
add_block('simulink/Signal Routing/Goto', [sub '/goto_y_e'],   'GotoTag', 'y_e',   'TagVisibility', 'global', ...
          'Position', [400 330 450 350]);
add_block('simulink/Signal Routing/Goto', [sub '/goto_psi_e'], 'GotoTag', 'psi_e', 'TagVisibility', 'global', ...
          'Position', [400 360 450 380]);
add_line(sub, 'errors/1', 'goto_y_e/1',   'autorouting', 'smart');
add_line(sub, 'errors/3', 'goto_psi_e/1', 'autorouting', 'smart');
end

% -------------------------------------------------------------- X-Y monitor
function build_xy_monitor_subsystem(sub, pos)
% Live X-Y monitoring of the vehicle position:
%   (1) XY Graph block (Simulink native): CG trace X vs Y, double-click the
%       block during / after a run to open it.
%   (2) "live path plot" MATLAB Function block: animated MATLAB figure with
%       the reference path, the growing CG trace and the ego vehicle drawn as
%       a triangle whose nose points along the yaw angle psi
%       (src/xy_monitor_update.m), refreshed every Ts_mon seconds and
%       switched off with xy_monitor_on = 0 (used by the batch script run_baseline).
add_block('built-in/Subsystem', sub, 'Position', pos);
in = {'X', 'Y', 'psi', 'y_e'};
for i = 1:4
    add_block('built-in/Inport', [sub '/' in{i}], 'Port', num2str(i), 'Position', [30 70*i 60 70*i+14]);
end

% (1) native XY Graph: vehicle CG trace
add_block('simulink/Sinks/XY Graph', [sub '/XY Graph vehicle trace'], 'Position', [320 50 380 110]);
add_line(sub, 'X/1', 'XY Graph vehicle trace/1', 'autorouting', 'smart');
add_line(sub, 'Y/1', 'XY Graph vehicle trace/2', 'autorouting', 'smart');

% (2) animated figure: sample the signals every Ts_mon, then call the extrinsic plotter
for i = 1:4
    add_block('simulink/Discrete/Zero-Order Hold', [sub '/hold_' in{i}], 'SampleTime', 'Ts_mon', ...
              'Position', [130 70*i-8 170 70*i+22]);
    add_line(sub, [in{i} '/1'], ['hold_' in{i} '/1'], 'autorouting', 'smart');
end
add_block('simulink/Sources/Clock', [sub '/t'], 'Position', [30 360 60 390]);
add_block('simulink/Discrete/Zero-Order Hold', [sub '/hold_t'], 'SampleTime', 'Ts_mon', 'Position', [130 360 170 390]);
add_line(sub, 't/1', 'hold_t/1');
add_block('simulink/Sources/Constant', [sub '/monitor on'], 'Value', 'xy_monitor_on', 'Position', [130 420 170 450]);
add_block('simulink/Sources/Constant', [sub '/path params'], 'Value', 'pth_vec', 'Position', [130 480 170 510]);

fcn = [sub '/live path plot'];
add_block('simulink/User-Defined Functions/MATLAB Function', fcn, 'Position', [320 160 460 510]);
set_fcn_script(fcn, sprintf([ ...
    'function live_path_plot(X, Y, psi, y_e, t, enable, pth)\n' ...
    '%%#codegen\n' ...
    '%% Animated X-Y monitor figure: reference path + vehicle CG trace + ego\n' ...
    '%% vehicle triangle (nose along the yaw angle psi).  Implemented in\n' ...
    '%% src/xy_monitor_update.m (called extrinsically, i.e. in MATLAB, every\n' ...
    '%% Ts_mon seconds).  Disabled when xy_monitor_on = 0.\n' ...
    'coder.extrinsic(''xy_monitor_update'');\n' ...
    'if enable > 0\n' ...
    '    xy_monitor_update(t, X, Y, psi, y_e, pth);\n' ...
    'end\n']));
add_line(sub, 'hold_X/1',     'live path plot/1', 'autorouting', 'smart');
add_line(sub, 'hold_Y/1',     'live path plot/2', 'autorouting', 'smart');
add_line(sub, 'hold_psi/1',   'live path plot/3', 'autorouting', 'smart');
add_line(sub, 'hold_y_e/1',   'live path plot/4', 'autorouting', 'smart');
add_line(sub, 'hold_t/1',     'live path plot/5', 'autorouting', 'smart');
add_line(sub, 'monitor on/1', 'live path plot/6', 'autorouting', 'smart');
add_line(sub, 'path params/1','live path plot/7', 'autorouting', 'smart');
end
