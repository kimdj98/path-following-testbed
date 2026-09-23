function [dz, delta, e, aux, info] = closed_loop_rhs(z, veh_vec, pth_vec, Vx, K, delta_max)
%CLOSED_LOOP_RHS  Right-hand side of the complete closed loop (one evaluation).
%
%   z = [X; Y; psi; Vy; r]  (global position, yaw, lateral speed, yaw rate)
%   K : 1x4 state-feedback gain (delta = -K e), or a function handle
%       delta = K(e, aux) for a user-defined control law (see simulate_closed_loop)
%
%   Mirrors the Simulink diagram: Path Tracking Errors -> State Feedback
%   Controller -> vehicle body (here: vehicle_body_3dof re-implementation
%   used for fast design iterations) -> Vehicle Global Coordinate.

X = z(1); Y = z(2); psi = z(3); Vy = z(4); r = z(5);

% "Path Tracking Errors" block
[ey, ey_dot, epsi, epsi_dot, aux] = path_tracking_errors(X, Y, psi, Vx, Vy, r, pth_vec);
e = [ey; ey_dot; epsi; epsi_dot];

% "State Feedback Controller" block (with road-wheel angle saturation)
if isa(K, 'function_handle')
    delta = K(e, aux);
else
    delta = -K * e;
end
delta = min(max(delta, -delta_max), delta_max);

% vehicle lateral / yaw dynamics
[dVy, dr, info] = vehicle_body_3dof(delta, Vx, Vy, r, veh_vec);

% "Vehicle Global Coordinate" block
[dX, dY] = global_coordinate(Vx, Vy, psi);

dz = [dX; dY; r; dVy; dr];
end
