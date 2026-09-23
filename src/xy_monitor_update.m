function h = xy_monitor_update(t, X, Y, psi, y_e, pth)
%XY_MONITOR_UPDATE  Animated X-Y monitor figure driven by the Simulink model.
%
%   Called extrinsically from the "X-Y Monitor" subsystem every Ts_mon
%   seconds while the simulation runs.  At t = 0 the figure is
%   (re)initialised and the reference path Y = WH*tanh((X-Xoff)/eta) + WH is
%   drawn from pth = [WH, Xoff, eta]; every later call appends the CG
%   position to the vehicle trace, moves the ego-vehicle glyph and refreshes
%   the title (time, position, heading, lateral offset).
%
%   Ego vehicle glyph: an isosceles triangle centred on the CG whose nose
%   points along the yaw angle psi.  The axes are far from equal aspect
%   (X spans ~300 m, Y only a few metres), so a car outline drawn in metres
%   would collapse to a sliver; the triangle therefore has a fixed size on
%   screen (TRI_LEN x TRI_WID pixels) and is rotated by the heading *as it
%   appears on screen*, i.e. [cos psi, sin psi] mapped through the
%   pixel-per-metre scale of each axis, so that the nose follows the plotted
%   trace.  The true heading is printed in the title.
%
%   h = xy_monitor_update('figure')   returns the figure handle (e.g. to save it)
%   xy_monitor_update('close')        closes the monitor figure

persistent fig ax trace ego hRef xmax_ref

TRI_LEN = 28;                                 % [px] triangle length (nose -> base)
TRI_WID = 16;                                 % [px] triangle base width
EGO_COL = [0.85 0.33 0.1];

if ischar(t)                                  % queries
    h = [];
    switch lower(t)
        case 'figure'
            h = fig;
        case 'close'
            if ~isempty(fig) && ishandle(fig), close(fig); end
            fig = [];
    end
    return;
end
h = [];

% ---------------------------------------------------- (re)initialisation
if isempty(fig) || ~ishandle(fig) || t <= 0
    if isempty(fig) || ~ishandle(fig)
        fig = figure('Name', 'X-Y Monitor', 'NumberTitle', 'off', 'Color', 'w', ...
                     'Position', [80 80 1000 420]);
    else
        clf(fig);
    end
    ax = axes('Parent', fig); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
    xmax_ref = max(X, 0) + 320;                          % 15 s at 20 m/s, plus margin
    Xg   = linspace(min(X, 0) - 10, xmax_ref, 1200);
    hRef = plot(ax, Xg, path_reference(Xg, pth), 'k--', 'LineWidth', 1.5);
    trace = animatedline(ax, 'Color', [0 0.447 0.741], 'LineWidth', 2);
    ego   = patch('Parent', ax, 'XData', X + [0 0 0], 'YData', Y + [0 0 0], ...
                  'FaceColor', EGO_COL, 'EdgeColor', 'k', 'LineWidth', 1);
    xlabel(ax, 'X [m]'); ylabel(ax, 'Y [m]');
    yr = [min(hRef.YData), max(hRef.YData)];
    ylim(ax, [yr(1) - 0.5, yr(2) + 0.5]);
    xlim(ax, [Xg(1), Xg(end)]);
    legend(ax, [hRef, trace, ego], {'reference path', 'vehicle CG trace', 'ego vehicle'}, ...
           'Location', 'southeast');
end

% --------------------------------------- extend the axes if the car leaves
if X > xmax_ref - 20
    xmax_ref = xmax_ref + 100;
    Xg = linspace(ax.XLim(1), xmax_ref, 1200);
    set(hRef, 'XData', Xg, 'YData', path_reference(Xg, pth));
    xlim(ax, [Xg(1), Xg(end)]);
end
yl = ylim(ax);
if Y < yl(1) + 0.1 || Y > yl(2) - 0.1
    ylim(ax, [min(yl(1), Y - 0.5), max(yl(2), Y + 0.5)]);
end

% ------------------------------------------------------------- update
addpoints(trace, X, Y);
[xv, yv] = ego_triangle(ax, X, Y, psi, TRI_LEN, TRI_WID);
set(ego, 'XData', xv, 'YData', yv);
title(ax, sprintf(['t = %6.2f s     X = %7.2f m   Y = %6.3f m   ' ...
                   '\\psi = %+6.2f^\\circ     e_y = %+.3f m'], t, X, Y, rad2deg(psi), y_e));
drawnow limitrate;
end

% =========================================================================
function [xv, yv] = ego_triangle(ax, X, Y, psi, len_px, wid_px)
%EGO_TRIANGLE  Vertices (data units) of the ego-vehicle triangle: CG at
%   (X, Y), nose along psi.  Built in screen pixels and mapped back through
%   the current axis scaling so the glyph keeps its shape and size on the
%   non-equal-aspect axes, whatever the axis limits are at the moment.
pos = getpixelposition(ax);                   % [left bottom width height] in px
sx  = pos(3) / diff(ax.XLim);                 % px per metre along X
sy  = pos(4) / diff(ax.YLim);                 % px per metre along Y
hx  = cos(psi) * sx;  hy = sin(psi) * sy;     % heading direction as drawn on screen
nrm = hypot(hx, hy);  hx = hx / nrm;  hy = hy / nrm;
lx  = [ 0.6*len_px, -0.4*len_px, -0.4*len_px];   % local frame [px]: nose, base-left, base-right
ly  = [ 0,           0.5*wid_px, -0.5*wid_px];
xv  = X + (hx*lx - hy*ly) / sx;               % rotate in px, convert back to metres
yv  = Y + (hy*lx + hx*ly) / sy;
end
