%MY_DESIGN_TEMPLATE  Starting point for your own path-following design.
%
%   Copy this file (e.g. to my_design.m), edit the sections marked EDIT and
%   run it from the project folder:      >> my_design
%
%   1. choose the scenario
%   2. design your controller (default: the reference pole-placement design)
%   3. quick check with the pure-MATLAB simulation, against the reference design
%   4. validation in Simulink (Vehicle Body 3DOF Single Track plant)
%   5. metrics table + standard plots  ->  results/<tag>/

clear; clc;
proj = fileparts(mfilename('fullpath'));
addpath(fullfile(proj, 'src'));
vis = 'on'; if ~usejava('desktop'), vis = 'off'; end

%% 1. scenario --------------------------------------------------------- EDIT
scenario = 'baseline';          % 'baseline' | 'sharp_lane_change'  (see load_params)
tag      = 'my_design';         % results are written to results/<tag>/

[veh, pth, sim_opt] = load_params(scenario);
[A, B, Bd, C] = error_model_linear(veh);   % x_dot = A x + B delta + Bd psi_dot_des,  y = C x

%% 2. your controller -------------------------------------------------- EDIT
% (a) state feedback  delta = -K x : set ctrl.K (your own poles, LQR weights, ...)
ctrl = design_state_feedback(A, B, 'place', struct('poles', [-2.5+2.5i, -2.5-2.5i, -6, -8]));
ctrl.name = 'my design';

% (b) any other control law  delta = law(e, aux)  (pure-MATLAB simulation only;
%     for Simulink, build the law into the "State Feedback Controller" subsystem)
%       e   = [ey; ey_dot; epsi; epsi_dot]
%       aux = [Xp, Yp, psi_p, kappa, psi_dot_des]   closest path point, curvature, ...
% ctrl.law = @(e, aux) -ctrl.K * e;

ref = design_state_feedback(A, B, 'place', struct('poles', default_poles()));
ref.name = 'reference (pole placement)';

%% 3. quick check: pure-MATLAB simulation (a few seconds) --------------------
out_dir = fullfile(proj, 'results', tag);
log_ref = simulate_closed_loop(veh, pth, ref,  sim_opt);
log_my  = simulate_closed_loop(veh, pth, ctrl, sim_opt);
fprintf('\n=== MATLAB simulation, scenario ''%s'' ===\n', scenario);
print_metrics_table({ref.name, ctrl.name}, {performance_metrics(log_ref), performance_metrics(log_my)});
plot_comparison({log_ref, log_my}, {ref.name, ctrl.name}, out_dir, [tag '_vs_reference'], ...
                sprintf('MATLAB simulation, scenario ''%s''', scenario), vis);

%% 4. Simulink validation (Vehicle Body 3DOF Single Track block) --------------
run_simulink = ~isfield(ctrl, 'law');       % the model implements delta = -K x
if run_simulink
    mdl = 'path_following_baseline';
    if ~exist(fullfile(proj, [mdl '.slx']), 'file'), build_simulink_model(mdl); end
    xy_monitor_on = 0;                      % 1: watch the live X-Y figure (slower)
    init_model_workspace([], 'match_design', scenario);
    assignin('base', 'K', ctrl.K);
    load_system(mdl);
    out = sim(mdl, 'StopTime', num2str(sim_opt.Tend));
    log_final = simulink_log_to_struct(out);
    source = 'Simulink';
else
    log_final = log_my;
    source = 'MATLAB simulation';
end

%% 5. results ------------------------------------------------------------------
M = performance_metrics(log_final);
fprintf('\n=== %s: %s, scenario ''%s'' ===\n', source, ctrl.name, scenario);
print_metrics_table({ctrl.name}, {M});
fid = fopen(fullfile(out_dir, 'metrics.txt'), 'w');
fprintf(fid, '%s, scenario ''%s''\n\n', source, scenario);
print_metrics_table({ctrl.name}, {M}, fid);
fclose(fid);
plot_results(log_final, pth, ctrl, out_dir, tag, vis);
save(fullfile(out_dir, [tag '.mat']), 'ctrl', 'log_final', 'M', 'scenario', 'source');
fprintf('\nDone. Figures and data written to %s\n', out_dir);
