function fig = plot_comparison(logs, names, out_dir, tag, ttl, visible)
%PLOT_COMPARISON  Overlay ey(t), epsi(t) and delta(t) of several runs.
%
%   fig = plot_comparison(logs, names, out_dir, tag, ttl, visible)
%
%   logs : cell array of log structs, names : legend entries,
%   saved as <out_dir>/<tag>.png

if nargin < 6, visible = 'on'; end
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

fig = figure('Visible', visible, 'Color', 'w', 'Name', tag, 'Position', [100 100 900 800]);
tl  = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, ttl);
ax1 = nexttile; hold(ax1, 'on'); grid(ax1, 'on'); ylabel(ax1, 'e_y [m]');      title(ax1, 'Lateral offset');
ax2 = nexttile; hold(ax2, 'on'); grid(ax2, 'on'); ylabel(ax2, 'e_\psi [deg]'); title(ax2, 'Heading error');
ax3 = nexttile; hold(ax3, 'on'); grid(ax3, 'on'); ylabel(ax3, '\delta_f [deg]'); xlabel(ax3, 'time [s]');
title(ax3, 'Steering command');

styles = {'-', '--', ':', '-.'};
for i = 1:numel(logs)
    L = logs{i}; s = styles{mod(i-1, numel(styles)) + 1};
    plot(ax1, L.t, L.ey,             s, 'LineWidth', 1.4);
    plot(ax2, L.t, rad2deg(L.epsi),  s, 'LineWidth', 1.4);
    plot(ax3, L.t, rad2deg(L.delta), s, 'LineWidth', 1.4);
end
legend(ax1, names, 'Location', 'best', 'Interpreter', 'none');
print(fig, fullfile(out_dir, [tag '.png']), '-dpng', '-r130');
end
