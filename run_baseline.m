%RUN_BASELINE  Baseline project: model-based vehicle path-following controller.
%
%   1. control-oriented path-tracking error model (A, B, Bd) + open-loop analysis
%   2. full-state feedback design: eigenvalue assignment (primary) and LQR (alternative)
%   3. design sweep with the fast pure-MATLAB simulation (own single-track plant)
%   4. validation in Simulink with the Vehicle Body 3DOF Single Track block
%      (Vehicle Dynamics Blockset) -- the model is built on first use
%   5. robustness check: same controller, block parametrised the "raw" way
%   6. standard plots and performance metrics -> results/
%
%   Run from the project folder:   >> run_baseline
%   Headless:                      matlab -batch run_baseline

clear; clc;
xy_monitor_on = 0;      % no live X-Y Monitor figure in batch runs (open the model to watch it)
proj = fileparts(mfilename('fullpath'));
addpath(fullfile(proj, 'src'));
res = fullfile(proj, 'results');
if ~exist(res, 'dir'), mkdir(res); end
vis = 'on'; if ~usejava('desktop'), vis = 'off'; end

[veh, pth, sim_opt] = load_params();

%% 1. control-oriented model --------------------------------------------------
[A, B, Bd, C] = error_model_linear(veh);
fprintf('\n=== Control-oriented path-tracking error model (Vx = %g m/s) ===\n', veh.Vx);
fprintf('x = [ey; ey_dot; epsi; epsi_dot],  x_dot = A x + B delta + Bd psi_dot_des\n');
disp('A  ='); disp(A);
disp('B'' ='); disp(B');
disp('Bd''='); disp(Bd');
fprintf('open-loop eigenvalues : %s\n', mat2str(eig(A).', 4));
fprintf('controllability rank  : %d / 4\n', rank(ctrb(A, B)));

%% 2. controller design -------------------------------------------------------
poles = default_poles();
ctrl  = design_state_feedback(A, B, 'place', struct('poles', poles));
fprintf('\n=== Eigenvalue assignment (primary design) ===\n');
fprintf('desired poles : %s\n', mat2str(poles, 4));
fprintf('K             : %s\n', mat2str(ctrl.K, 5));

Q = diag([1/0.1^2, 0, 1/deg2rad(1)^2, 0]);   % Bryson: 0.1 m lateral / 1 deg heading tolerated
R = 1/deg2rad(2)^2;                           %         2 deg steering tolerated
ctrl_lqr = design_state_feedback(A, B, 'lqr', struct('Q', Q, 'R', R));
fprintf('\n=== LQR (alternative design) ===\n');
fprintf('closed-loop poles : %s\n', mat2str(ctrl_lqr.poles.', 4));
fprintf('K                 : %s\n', mat2str(ctrl_lqr.K, 5));

%% 3. design sweep (pure-MATLAB RK4 simulation) --------------------------------
cand = { ...
    'place slow  [-1.5+-1.5i, -4, -5]',  design_state_feedback(A, B, 'place', struct('poles', [-1.5+1.5i, -1.5-1.5i, -4, -5]));
    'place medium (default)',            ctrl;
    'place fast  [-4+-4i, -10, -12]',    design_state_feedback(A, B, 'place', struct('poles', [-4+4i, -4-4i, -10, -12]));
    'lqr',                               ctrl_lqr };
names = cand(:, 1);
mets  = cell(size(names)); logs_rk4 = cell(size(names));
for i = 1:numel(names)
    logs_rk4{i} = simulate_closed_loop(veh, pth, cand{i, 2}, sim_opt);
    mets{i}     = performance_metrics(logs_rk4{i});
end
fprintf('\n=== Design sweep (MATLAB RK4 simulation, own single-track plant) ===\n');
print_metrics_table(names, mets);
fid = fopen(fullfile(res, 'design_sweep.txt'), 'w');
print_metrics_table(names, mets, fid); fclose(fid);
plot_comparison(logs_rk4, names, res, 'design_sweep', 'Design sweep (MATLAB RK4 simulation)', vis);

%% 4. save the design (read by the Simulink model's PreLoadFcn) ----------------
design_file = fullfile(res, 'baseline_design.mat');
save(design_file, 'ctrl', 'ctrl_lqr', 'veh', 'pth', 'sim_opt', 'A', 'B', 'Bd', 'C');

%% 5. Simulink validation with the Vehicle Body 3DOF Single Track block --------
mdl = 'path_following_baseline';
if ~exist(fullfile(proj, [mdl '.slx']), 'file')
    build_simulink_model(mdl);
end
init_model_workspace(design_file, 'match_design');
blk = vdb_block_params(veh, 'match_design');
load_system(mdl);
fprintf('\n=== Simulink run: %s.slx (Vehicle Dynamics Blockset plant) ===\n', mdl);
fprintf('block parametrisation ''match_design'': Cy_f = %.0f, Cy_r = %.0f N/rad at Fznom = %.0f N\n', blk.Cy_f, blk.Cy_r, blk.Fznom);
fprintf('  -> effective stiffness at static axle loads (%.0f / %.0f N): Cf = %.0f, Cr = %.0f N/rad\n', ...
        blk.Fz_f, blk.Fz_r, blk.Cf_eff, blk.Cr_eff);
out_place = sim(mdl, 'StopTime', num2str(sim_opt.Tend));
log_place = simulink_log_to_struct(out_place);

assignin('base', 'K', ctrl_lqr.K);                       % same model, LQR gain
out_lqr = sim(mdl, 'StopTime', num2str(sim_opt.Tend));
log_lqr = simulink_log_to_struct(out_lqr);
assignin('base', 'K', ctrl.K);

% robustness check: Cf, Cr typed directly into the block (default Fznom, aero on)
init_model_workspace(design_file, 'raw');
blk_raw = vdb_block_params(veh, 'raw');
fprintf('block parametrisation ''raw'': Cy_f = Cy_r = %.0f N/rad at Fznom = %.0f N\n', blk_raw.Cy_f, blk_raw.Fznom);
fprintf('  -> effective stiffness at static axle loads: Cf = %.0f, Cr = %.0f N/rad (model mismatch)\n', ...
        blk_raw.Cf_eff, blk_raw.Cr_eff);
out_raw = sim(mdl, 'StopTime', num2str(sim_opt.Tend));
log_raw = simulink_log_to_struct(out_raw);
init_model_workspace(design_file, 'match_design');       % restore the default configuration

%% 6. results -----------------------------------------------------------------
M_place = performance_metrics(log_place);
M_lqr   = performance_metrics(log_lqr);
M_raw   = performance_metrics(log_raw);
tab_names = {'pole placement (Simulink)', 'LQR (Simulink)', 'pole placement, raw block config'};
tab_mets  = {M_place, M_lqr, M_raw};
fprintf('\n=== Closed-loop performance (Simulink, Vehicle Body 3DOF Single Track) ===\n');
print_metrics_table(tab_names, tab_mets);
fprintf('final values (pole placement): ey = %.4f m, epsi = %.4f deg\n', M_place.ey_final, M_place.epsi_final);

fid = fopen(fullfile(res, 'metrics.txt'), 'w');
fprintf(fid, 'Baseline path-following controller - Simulink validation (%s)\n\n', char(datetime('now')));
fprintf(fid, 'pole placement: poles %s, K = %s\n', mat2str(poles, 4), mat2str(ctrl.K, 5));
fprintf(fid, 'LQR           : Q = diag(%s), R = %.4g, K = %s\n', mat2str(diag(Q).', 4), R, mat2str(ctrl_lqr.K, 5));
fprintf(fid, 'block config  : match_design -> Cf_eff = %.0f, Cr_eff = %.0f N/rad;  raw -> Cf_eff = %.0f, Cr_eff = %.0f N/rad\n\n', ...
        blk.Cf_eff, blk.Cr_eff, blk_raw.Cf_eff, blk_raw.Cr_eff);
print_metrics_table(tab_names, tab_mets, fid);
fprintf(fid, '\nfinal values (pole placement): ey = %.4f m, epsi = %.4f deg\n', M_place.ey_final, M_place.epsi_final);
fclose(fid);

plot_results(log_place, pth, ctrl,     res, 'baseline_place', vis);
plot_results(log_lqr,   pth, ctrl_lqr, res, 'baseline_lqr',   vis);
plot_comparison({log_place, log_lqr}, {'pole placement', 'LQR'}, res, 'place_vs_lqr', ...
                'Pole placement vs. LQR (Simulink, Vehicle Body 3DOF block)', vis);
plot_comparison({log_place, log_raw}, {'block matched to design model (Cf = Cr = 50 kN/rad)', ...
                sprintf('raw block config (Cf,Cr eff. = %.0f, %.0f kN/rad)', blk_raw.Cf_eff/1e3, blk_raw.Cr_eff/1e3)}, ...
                res, 'block_config_match_vs_raw', 'Robustness: same controller, two block parametrisations', vis);

% cross-check: Simulink (VDB block) vs. pure-MATLAB RK4 (own plant), same gain K
log_rk4 = logs_rk4{2};
d_ey  = max(abs(interp1(log_rk4.t, log_rk4.ey,   log_place.t) - log_place.ey));
d_psi = rad2deg(max(abs(interp1(log_rk4.t, log_rk4.epsi, log_place.t) - log_place.epsi)));
fprintf('\ncross-check Simulink vs MATLAB RK4 : max|d ey| = %.2e m, max|d epsi| = %.2e deg\n', d_ey, d_psi);
plot_comparison({log_place, log_rk4}, {'Simulink: Vehicle Body 3DOF block', 'MATLAB RK4: own single-track plant'}, ...
                res, 'crosscheck_simulink_vs_matlab', 'Same controller on the two plants', vis);
if isfield(log_place, 'X_block')
    fprintf('Vehicle Global Coordinate block vs. Info bus position: max|dX| = %.2e m, max|dY| = %.2e m\n', ...
        max(abs(log_place.X - log_place.X_block)), max(abs(log_place.Y - log_place.Y_block)));
end

save(fullfile(res, 'baseline_run.mat'), 'log_place', 'log_lqr', 'log_raw', 'log_rk4', ...
     'M_place', 'M_lqr', 'M_raw', 'ctrl', 'ctrl_lqr', 'blk', 'blk_raw');
fprintf('\nDone. Figures and data written to %s\n', res);
