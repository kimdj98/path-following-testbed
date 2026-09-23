function Xp = path_closest_point(X, Y, p, Xp0)
%PATH_CLOSEST_POINT  X-coordinate of the path point closest to (X, Y).
%
%   Xp = path_closest_point(X, Y, p, Xp0)
%
%   Minimises  D(Xp) = (Xp - X)^2 + (Y(Xp) - Y)^2  with a few damped Newton
%   iterations on  g(Xp) = dD/dXp / 2 = (Xp - X) + (Y(Xp) - Y) * Y'(Xp) = 0,
%   starting from Xp0 (the vehicle X is an excellent initial guess because
%   the path slope is small, |Y'| <= WH/eta = 0.05).
%
% Code-generation compatible (fixed iteration count, scalar math).

Xp = Xp0;
for k = 1:8
    [Yp, ~, ~, dY, d2Y] = path_reference(Xp, p);
    g  = (Xp - X) + (Yp - Y) * dY;
    dg = 1 + dY^2 + (Yp - Y) * d2Y;   % > 0 whenever (X,Y) is near the path
    if dg < 0.5
        dg = 0.5;                     % guard against loss of convexity far away
    end
    Xp = Xp - g / dg;
end
end
