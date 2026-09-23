function [Yp, psi_p, kappa, dYdX, d2YdX2] = path_reference(Xp, p)
%PATH_REFERENCE  Geometry of the S-shaped lane-change path.
%
%   [Yp, psi_p, kappa, dYdX, d2YdX2] = path_reference(Xp, p)
%
%   Path:
%       Y(X) = WH * tanh((X - Xoff)/eta) + WH ,   p = [WH, Xoff, eta]
%
%   Yp      : path Y-coordinate at Xp
%   psi_p   : path heading (tangent angle)           = atan(dY/dX)
%   kappa   : path curvature (left turn positive)    = Y'' / (1+Y'^2)^(3/2)
%   dYdX, d2YdX2 : first / second derivatives (used by closest-point search)
%
% Code-generation compatible (used inside Simulink MATLAB Function blocks).

WH = p(1); Xoff = p(2); eta = p(3);

u     = (Xp - Xoff) / eta;
th    = tanh(u);
sech2 = 1 - th.^2;                    % sech^2(u)

Yp     = WH * th + WH;
dYdX   = (WH / eta) * sech2;
d2YdX2 = -(2 * WH / eta^2) * sech2 .* th;

psi_p = atan(dYdX);
kappa = d2YdX2 ./ (1 + dYdX.^2).^1.5;
end
