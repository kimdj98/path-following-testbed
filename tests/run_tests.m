function run_tests()
%RUN_TESTS  Verification checks for the baseline path-following project.
%
%   >> run_tests                      (from the project folder or tests/)
%   matlab -batch "cd tests; run_tests"
%
%   1. path geometry derivatives vs. finite differences
%   2. error block: sign convention and error rates
%   3. linear error model consistent with the nonlinear single-track equations
%   4. eigenvalue assignment places the poles where requested
%   5. Vehicle Body 3DOF Single Track block (Vehicle Dynamics Blockset):
%      a) 'match_design' parametrisation reproduces the linear single-track
%         model in an open-loop step-steer test
%      b) 'raw' parametrisation behaves as predicted by the block's load
%         scaling  Fy = Cy*alpha*Fz/Fznom  (effective stiffness Cy*Fz/Fznom)
%   6. closed loop: Simulink model (VDB plant) vs. pure-MATLAB RK4 (own plant),
%      Vehicle Global Coordinate block vs. the block's own position, and
%      convergence within the steering limit

proj = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(proj, 'src')); addpath(proj);
warning('off', 'all');

[veh, pth, sim_opt] = load_params();
pv = path_to_vec(pth);
T  = {};   % {name, pass, detail}

% ---------------------------------------------------------------- 1. path
Xs = linspace(-20, 200, 23); h = 1e-4; e1 = 0; e2 = 0;
for X = Xs
    [~, ~, ~, dY, d2Y] = path_reference(X, pv);
    Yp = path_reference(X + h, pv); Ym = path_reference(X - h, pv); Y0 = path_reference(X, pv);
    e1 = max(e1, abs((Yp - Ym) / (2*h) - dY));
    e2 = max(e2, abs((Yp - 2*Y0 + Ym) / h^2 - d2Y));
end
T(end+1, :) = {'path derivatives vs finite differences', e1 < 1e-7 && e2 < 1e-4, ...
               sprintf('dY err %.1e, d2Y err %.1e', e1, e2)};

% --------------------------------------------------- 2. error block signs
X0 = 75; [Y0, psi0, kappa0] = path_reference(X0, pv);
d  = 0.3; n = [-sin(psi0); cos(psi0)];                 % left normal of the path
P  = [X0; Y0] + d * n;                                 % CG 0.3 m to the LEFT of the path
dpsi = 0.02;
[ey, ey_dot, epsi, epsi_dot] = path_tracking_errors(P(1), P(2), psi0 + dpsi, veh.Vx, 0, veh.Vx*kappa0, pv);
ok = abs(ey - d) < 1e-6 && abs(epsi - dpsi) < 1e-9 && ...
     abs(ey_dot - veh.Vx*sin(dpsi)) < 1e-9 && abs(epsi_dot) < 1e-5;
T(end+1, :) = {'error block: ey left-positive, epsi, ey_dot, epsi_dot', ok, ...
               sprintf('ey=%.6f (exp %.3f), epsi=%.4f (exp %.4f), epsi_dot=%.1e', ey, d, epsi, dpsi, epsi_dot)};

% ------------------------------------------- 3. linear model consistency
vv = veh_to_vec(veh); Vx = veh.Vx; hh = 1e-6;
g   = @(Vy, r, dl) rhs2(dl, Vx, Vy, r, vv);
JVy = (g(hh, 0, 0) - g(-hh, 0, 0)) / (2*hh);
Jr  = (g(0, hh, 0) - g(0, -hh, 0)) / (2*hh);
Jd  = (g(0, 0, hh) - g(0, 0, -hh)) / (2*hh);
[A, B] = error_model_linear(veh);
chk = [A(2,2) - JVy(1);  A(4,2) - JVy(2);  A(2,4) - (Jr(1) + Vx);  A(4,4) - Jr(2); ...
       B(2) - Jd(1);     B(4) - Jd(2);     A(2,3) + A(2,2)*Vx;     A(4,3) + A(4,2)*Vx];
scale = max(abs([A(:); B(:)]));
T(end+1, :) = {'linear error model vs nonlinear single-track Jacobian', max(abs(chk)) < 1e-6*scale, ...
               sprintf('max deviation %.1e (scale %.3g)', max(abs(chk)), scale)};

% ------------------------------------------------------ 4. pole placement
p_des = default_poles();
ctrl  = design_state_feedback(A, B, 'place', struct('poles', p_des));
perr  = max(abs(sortc(ctrl.poles) - sortc(p_des)));
T(end+1, :) = {'eigenvalue assignment accuracy', perr < 1e-6, ...
               sprintf('max |pole error| %.1e, K = %s', perr, mat2str(ctrl.K, 4))};

% ------------------------------- 5. VDB block vs linear model, step steer
d_step = deg2rad(1); t_step = 0.5;
Av = [A(2,2), A(2,4) - Vx; A(4,2), A(4,4)];  Bv = [B(2); B(4)];   % (Vy, r) dynamics
try
    % a) block matched to the design model
    blk = vdb_block_params(veh, 'match_design');
    [tp, rp] = probe_step_steer(veh, blk, d_step, t_step, 5);
    r_lin   = lsim(ss(Av, Bv, [0 1], 0), d_step * (tp >= t_step), tp);
    rel_ss  = abs(rp(end) - r_lin(end)) / abs(r_lin(end));
    rel_max = max(abs(rp - r_lin)) / abs(r_lin(end));
    T(end+1, :) = {'VDB block (match_design) vs linear single-track model, 1 deg step', rel_ss < 0.01 && rel_max < 0.05, ...
                   sprintf('steady-state yaw rate: block %.5f, model %.5f rad/s (rel err %.2e); max transient dev %.2e', ...
                           rp(end), r_lin(end), rel_ss, rel_max)};

    % b) raw parametrisation: predicted with the effective stiffnesses Cy*Fz/Fznom
    blk_raw = vdb_block_params(veh, 'raw');
    [tp2, rp2] = probe_step_steer(veh, blk_raw, d_step, t_step, 5);
    veh_eff = veh; veh_eff.Cf = blk_raw.Cf_eff; veh_eff.Cr = blk_raw.Cr_eff;
    [A2, B2] = error_model_linear(veh_eff);
    r_lin2  = lsim(ss([A2(2,2), A2(2,4) - Vx; A2(4,2), A2(4,4)], [B2(2); B2(4)], [0 1], 0), d_step * (tp2 >= t_step), tp2);
    rel_ss2 = abs(rp2(end) - r_lin2(end)) / abs(r_lin2(end));
    T(end+1, :) = {'VDB block (raw) explained by load-scaled stiffness Cy*Fz/Fznom', rel_ss2 < 0.03, ...
                   sprintf('block %.5f rad/s vs prediction with Cf_eff=%.0f, Cr_eff=%.0f: %.5f (rel err %.2e); design model %.5f', ...
                           rp2(end), blk_raw.Cf_eff, blk_raw.Cr_eff, r_lin2(end), rel_ss2, r_lin(end))};
catch e
    T(end+1, :) = {'VDB block step-steer probes', false, e.message};
end

% ------------------------------------------ 6. closed loop Simulink vs RK4
try
    mdl = 'path_following_baseline';
    if ~exist(fullfile(proj, [mdl '.slx']), 'file'), build_simulink_model(mdl); end
    ctrl = init_model_workspace([], 'match_design', 'baseline');
    assignin('base', 'xy_monitor_on', 1);                   % exercise the live X-Y Monitor too
    xy_monitor_update('close');
    load_system(mdl);
    out  = sim(mdl, 'StopTime', num2str(sim_opt.Tend));
    logS = simulink_log_to_struct(out);
    hfig = xy_monitor_update('figure');
    mon_ok = ~isempty(hfig) && ishandle(hfig);
    if mon_ok
        print(hfig, fullfile(proj, 'results', 'xy_monitor_check.png'), '-dpng', '-r110');
    end
    T(end+1, :) = {'X-Y Monitor figure driven by the Simulink run', mon_ok, ...
                   'figure saved to results/xy_monitor_check.png'};
    logM = simulate_closed_loop(veh, pth, ctrl, sim_opt);
    d_ey  = max(abs(interp1(logM.t, logM.ey,   logS.t) - logS.ey));
    d_psi = rad2deg(max(abs(interp1(logM.t, logM.epsi, logS.t) - logS.epsi)));
    T(end+1, :) = {'closed loop: Simulink (VDB plant) vs MATLAB RK4 (own plant)', d_ey < 5e-3 && d_psi < 0.05, ...
                   sprintf('max|d ey| %.2e m, max|d epsi| %.2e deg, final ey %.4f m', d_ey, d_psi, logS.ey(end))};
    [~, ~, kap_chk] = path_reference(logS.Xp, pv);          % path_info log layout + curvature
    kap_max_analytic = (2*pth.WH/pth.eta^2) * 2/(3*sqrt(3));  % max |Y''| of the tanh path
    T(end+1, :) = {'Simulink path_info log (N x 5) and path curvature along the run', ...
                   numel(logS.kappa) == numel(logS.t) && max(abs(logS.kappa - kap_chk)) < 1e-12 && ...
                   abs(max(abs(logS.kappa)) - kap_max_analytic) < 0.01*kap_max_analytic, ...
                   sprintf('N = %d, max|kappa| = %.3e 1/m (analytic %.3e), max dev %.1e', ...
                           numel(logS.kappa), max(abs(logS.kappa)), kap_max_analytic, max(abs(logS.kappa - kap_chk)))};
    if isfield(logS, 'X_block')
        dXY = max([abs(logS.X - logS.X_block); abs(logS.Y - logS.Y_block)]);
        T(end+1, :) = {'Vehicle Global Coordinate block vs VDB Info position', dXY < 1e-3, ...
                       sprintf('max |dX|,|dY| = %.2e m', dXY)};
    else
        T(end+1, :) = {'Vehicle Global Coordinate block vs VDB Info position', false, 'Info bus position not found in log'};
    end
    T(end+1, :) = {'closed loop stays within steering limit and converges', ...
                   max(abs(logS.delta)) < veh.delta_max && abs(logS.ey(end)) < 0.02, ...
                   sprintf('max|delta| %.2f deg, final ey %.4f m', rad2deg(max(abs(logS.delta))), logS.ey(end))};
catch e
    T(end+1, :) = {'closed loop: Simulink (VDB plant) vs MATLAB RK4 (own plant)', false, e.message};
end

% ---------------------------------------------------------------- report
fprintf('\n%-72s %s\n', 'TEST', 'RESULT');
npass = 0;
for i = 1:size(T, 1)
    if T{i, 2}, s = 'PASS'; npass = npass + 1; else, s = 'FAIL'; end
    fprintf('%-72s %s\n    %s\n', T{i, 1}, s, T{i, 3});
end
fprintf('\n%d / %d tests passed\n', npass, size(T, 1));
if npass < size(T, 1)
    error('run_tests:failed', '%d test(s) failed', size(T, 1) - npass);
end
end

% =========================================================================
function dz = rhs2(dl, Vx, Vy, r, vv)
[dVy, dr] = vehicle_body_3dof(dl, Vx, Vy, r, vv);
dz = [dVy; dr];
end

function s = sortc(p)
p = p(:); [~, i] = sortrows([real(p), imag(p)]); s = p(i);
end

function [tp, rp] = probe_step_steer(veh, blk, d_step, t_step, Tend)
% open-loop step-steer test of the Vehicle Body 3DOF Single Track block alone
pm = 'vdb_step_probe'; if bdIsLoaded(pm), close_system(pm, 0); end
load_system('vehdynlibeom'); new_system(pm);
add_block('vehdynlibeom/Vehicle Body 3DOF Single Track', [pm '/veh'], 'Position', [200 50 400 250]);
set_param([pm '/veh'], 'inputMode', 'External longitudinal velocity', ...
    'm', num2str(veh.m), 'a', num2str(veh.lf), 'b', num2str(veh.lr), 'Izz', num2str(veh.Izz), 'g', num2str(veh.g), ...
    'Cy_f', num2str(blk.Cy_f, 10), 'Cy_r', num2str(blk.Cy_r, 10), 'Fznom', num2str(blk.Fznom, 10), ...
    'Cd', num2str(blk.Cd), 'Cl', num2str(blk.Cl), 'Cpm', num2str(blk.Cpm));
add_block('simulink/Sources/Step', [pm '/steer'], 'Time', num2str(t_step), 'Before', '0', ...
          'After', num2str(d_step, 10), 'Position', [50 80 80 110]);
add_block('simulink/Sources/Constant', [pm '/vx'], 'Value', num2str(veh.Vx), 'Position', [50 160 80 190]);
add_block('simulink/Sinks/To Workspace', [pm '/log_r'], 'VariableName', 'r_probe', ...
          'SaveFormat', 'Array', 'Position', [500 170 560 200]);
for k = [1 2 3 4 6 7]
    add_block('simulink/Sinks/Terminator', [pm sprintf('/t%d', k)], 'Position', [500 20+30*k 520 40+30*k]);
    add_line(pm, sprintf('veh/%d', k), sprintf('t%d/1', k));
end
add_line(pm, 'steer/1', 'veh/1'); add_line(pm, 'vx/1', 'veh/2'); add_line(pm, 'veh/5', 'log_r/1');
set_param(pm, 'Solver', 'ode4', 'FixedStep', '1e-3', 'StopTime', num2str(Tend), 'SaveTime', 'on', ...
              'ReturnWorkspaceOutputs', 'on');
out = sim(pm);
tp = reshape(out.get('tout'), [], 1); rp = reshape(out.get('r_probe'), [], 1);
close_system(pm, 0);
end
