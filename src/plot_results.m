function figs = plot_results(log, pth, ctrl, out_dir, tag, visible)
%PLOT_RESULTS  Standard plots of one closed-loop run.
%
%   figs = plot_results(log, pth, ctrl, out_dir, tag, visible)
%
%   Produces and saves (PNG) the four required plots
%     1. lateral offset ey(t)                         <tag>_ey.png
%     2. heading error epsi(t)                        <tag>_epsi.png
%     3. X-Y trace of the CG overlaid on the S-path   <tag>_xy.png
%     4. feedback steering command delta(t)           <tag>_steer.png
%   plus a 2x2 summary figure                          <tag>_summary.png
%
%   ctrl    : controller struct; the figure titles show ctrl.name if it
%             exists, else the method and gain K
%   visible : 'on' | 'off'  (use 'off' for headless / batch runs)

if nargin < 6, visible = 'on'; end
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

t   = log.t;
if isfield(ctrl, 'name')
    ttl = ctrl.name;
elseif isfield(ctrl, 'K') && ~isfield(ctrl, 'law')
    ttl = sprintf('%s   K = [%s]', upper(ctrl.method), num2str(ctrl.K, '%.3g  '));
else
    ttl = 'user-defined control law';
end

Xg = linspace(min(log.X), max(log.X), 800)';
Yg = path_reference(Xg, path_to_vec(pth));

figs = gobjects(5, 1);

figs(1) = figure('Visible', visible, 'Name', 'Lateral offset', 'Color', 'w');
plot(t, log.ey, 'LineWidth', 1.5); grid on;
xlabel('time [s]'); ylabel('e_y [m]');
title({'Lateral offset of the CG from the path', ttl});
save_fig(figs(1), out_dir, [tag '_ey']);

figs(2) = figure('Visible', visible, 'Name', 'Heading error', 'Color', 'w');
plot(t, rad2deg(log.epsi), 'LineWidth', 1.5); grid on;
xlabel('time [s]'); ylabel('e_\psi [deg]');
title({'Heading angle error', ttl});
save_fig(figs(2), out_dir, [tag '_epsi']);

figs(3) = figure('Visible', visible, 'Name', 'X-Y trace', 'Color', 'w');
plot(Xg, Yg, 'k--', 'LineWidth', 1.5); hold on;
plot(log.X, log.Y, 'LineWidth', 1.5);
plot(log.X(1), log.Y(1), 'go', 'MarkerFaceColor', 'g');
plot(log.X(end), log.Y(end), 'rs', 'MarkerFaceColor', 'r');
grid on; xlabel('X [m]'); ylabel('Y [m]');
legend('S-shaped reference path', 'vehicle CG trace', 'start', 'end', 'Location', 'southeast');
title({'Vehicle X-Y trace vs. reference path', ttl});
save_fig(figs(3), out_dir, [tag '_xy']);

figs(4) = figure('Visible', visible, 'Name', 'Steering command', 'Color', 'w');
plot(t, rad2deg(log.delta), 'LineWidth', 1.5); grid on;
xlabel('time [s]'); ylabel('\delta_f [deg]');
title({'Feedback steering command (front road-wheel angle)', ttl});
save_fig(figs(4), out_dir, [tag '_steer']);

figs(5) = figure('Visible', visible, 'Name', 'Summary', 'Color', 'w', ...
                 'Position', [100 100 1100 750]);
tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, {'Baseline path-following controller', ttl});
nexttile; plot(t, log.ey, 'LineWidth', 1.3); grid on;
xlabel('time [s]'); ylabel('e_y [m]'); title('Lateral offset');
nexttile; plot(t, rad2deg(log.epsi), 'LineWidth', 1.3); grid on;
xlabel('time [s]'); ylabel('e_\psi [deg]'); title('Heading error');
nexttile; plot(Xg, Yg, 'k--', 'LineWidth', 1.3); hold on;
plot(log.X, log.Y, 'LineWidth', 1.3); grid on;
xlabel('X [m]'); ylabel('Y [m]'); title('X-Y trace');
legend('reference path', 'vehicle CG', 'Location', 'southeast');
nexttile; plot(t, rad2deg(log.delta), 'LineWidth', 1.3); grid on;
xlabel('time [s]'); ylabel('\delta_f [deg]'); title('Steering command');
save_fig(figs(5), out_dir, [tag '_summary']);
end

function save_fig(fig, out_dir, name)
print(fig, fullfile(out_dir, [name '.png']), '-dpng', '-r130');
end
