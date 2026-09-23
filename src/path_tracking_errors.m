function [ey, ey_dot, epsi, epsi_dot, aux] = path_tracking_errors(X, Y, psi, Vx, Vy, r, p)
%PATH_TRACKING_ERRORS  "Path Tracking Errors" block.
%
%   [ey, ey_dot, epsi, epsi_dot, aux] = path_tracking_errors(X, Y, psi, Vx, Vy, r, p)
%
%   Inputs  : vehicle CG global position (X, Y), yaw angle psi, body-frame
%             velocities Vx (longitudinal), Vy (lateral) and yaw rate r,
%             path parameter vector p = [WH, Xoff, eta].
%   Outputs : the path-following error state used by the controller,
%
%       ey       lateral offset of the CG from the path   (left of path: +)
%       ey_dot   its time derivative     = Vx*sin(epsi) + Vy*cos(epsi)
%       epsi     heading error           = psi - psi_path
%       epsi_dot its time derivative     = r - psi_dot_des,
%                psi_dot_des = kappa * s_dot  (s_dot: speed along the path)
%
%       aux = [Xp, Yp, psi_p, kappa, psi_dot_des]  (closest path point etc.)
%
% Sign convention matches the control-oriented model in error_model_linear:
% positive steering angle -> positive yaw rate -> vehicle turns left -> ey grows.
%
% Code-generation compatible (used inside a Simulink MATLAB Function block).

% closest point on the path (projection of the CG onto the path)
Xp = path_closest_point(X, Y, p, X);
[Yp, psi_p, kappa] = path_reference(Xp, p);

% lateral offset: component of (CG - path point) along the path normal
% n = [-sin(psi_p); cos(psi_p)]  (points to the left of the path)
ey = -(X - Xp) * sin(psi_p) + (Y - Yp) * cos(psi_p);

% heading error, wrapped to (-pi, pi]
d    = psi - psi_p;
epsi = atan2(sin(d), cos(d));

% error rates (exact kinematics of the Frenet frame)
ey_dot      = Vx * sin(epsi) + Vy * cos(epsi);
s_dot       = (Vx * cos(epsi) - Vy * sin(epsi)) / (1 - kappa * ey);
psi_dot_des = kappa * s_dot;
epsi_dot    = r - psi_dot_des;

aux = [Xp, Yp, psi_p, kappa, psi_dot_des];
end
