function log = simulate_closed_loop(veh, pth, ctrl, sim_opt)
%SIMULATE_CLOSED_LOOP  Fixed-step RK4 simulation of the path-following loop.
%
%   log = simulate_closed_loop(veh, pth, ctrl, sim_opt)
%
%   Pure-MATLAB design/analysis tool: same block functions as the Simulink
%   model except that the plant is the single-track re-implementation in
%   vehicle_body_3dof (instead of the Vehicle Dynamics Blockset block).
%   Used for fast controller design sweeps and as an independent
%   cross-check of the Simulink results.
%
%   ctrl.K    1x4 gain, control law delta = -K e  (from design_state_feedback)
%   ctrl.law  (optional, used instead of ctrl.K) function handle
%             delta = law(e, aux) with e = [ey; ey_dot; epsi; epsi_dot] and
%             aux = [Xp, Yp, psi_p, kappa, psi_dot_des] (see path_tracking_errors).
%             The +-delta_max saturation is applied in both cases.
%
%   log fields (column vectors over time): t, X, Y, psi, Vy, r,
%   ey, ey_dot, epsi, epsi_dot, delta, Xp, Yp, psi_p, kappa, psi_dot_des,
%   Fyf, Fyr, alpha_f, alpha_r, ay.

veh_vec = veh_to_vec(veh);
pth_vec = path_to_vec(pth);
Vx      = veh.Vx;
if isfield(ctrl, 'law')
    K = ctrl.law;
else
    K = ctrl.K;
end
dmax    = veh.delta_max;

dt = sim_opt.dt;
t  = (0:dt:sim_opt.Tend)';
N  = numel(t);

Z    = zeros(N, 5);
E    = zeros(N, 4);
D    = zeros(N, 1);
AUX  = zeros(N, 5);
INFO = zeros(N, 5);

f = @(z) closed_loop_rhs(z, veh_vec, pth_vec, Vx, K, dmax);

z = sim_opt.z0(:);
for k = 1:N
    [k1, delta, e, aux, info] = f(z);
    Z(k,:) = z'; E(k,:) = e'; D(k) = delta; AUX(k,:) = aux; INFO(k,:) = info;
    if k == N, break; end
    k2 = f(z + 0.5*dt*k1);
    k3 = f(z + 0.5*dt*k2);
    k4 = f(z +     dt*k3);
    z  = z + dt/6 * (k1 + 2*k2 + 2*k3 + k4);
end

log.t   = t;
log.X   = Z(:,1); log.Y = Z(:,2); log.psi = Z(:,3); log.Vy = Z(:,4); log.r = Z(:,5);
log.ey  = E(:,1); log.ey_dot = E(:,2); log.epsi = E(:,3); log.epsi_dot = E(:,4);
log.delta = D;
log.Xp = AUX(:,1); log.Yp = AUX(:,2); log.psi_p = AUX(:,3); log.kappa = AUX(:,4);
log.psi_dot_des = AUX(:,5);
log.Fyf = INFO(:,1); log.Fyr = INFO(:,2); log.alpha_f = INFO(:,3); log.alpha_r = INFO(:,4);
log.ay  = INFO(:,5);
end
