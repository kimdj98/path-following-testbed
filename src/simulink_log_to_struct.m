function log = simulink_log_to_struct(out)
%SIMULINK_LOG_TO_STRUCT  Convert the model's logged signals to a log struct.
%
%   log = simulink_log_to_struct(out)   with out = sim('path_following_baseline')
%
%   The model records its signals with signal logging (Dataset out.logsout,
%   signal names = line names in the diagram).  The result has the same
%   field layout as simulate_closed_loop so that performance_metrics and
%   plot_results work on both.  If the Info bus of the Vehicle Body block is
%   logged, its inertial position is returned in log.X_block / log.Y_block
%   (used to cross-check the Vehicle Global Coordinate block).

ds  = out.get('logsout');                       % Simulink.SimulationData.Dataset
sig = @(name) ds.get(name).Values;              % timeseries (or struct for buses)
col = @(name) reshape(sig(name).Data, [], 1);

log.t        = reshape(sig('y_e').Time, [], 1);
log.X        = col('X_glob');
log.Y        = col('Y_glob');
log.psi      = col('psi');
log.Vy       = col('ydot');
log.r        = col('r');
log.ey       = col('y_e');
log.ey_dot   = col('y_e_dot');
log.epsi     = col('psi_e');
log.epsi_dot = col('psi_e_dot');
log.delta    = col('WhlAngF');

% [Xp, Yp, psi_p, kappa, psi_dot_des]: a 1x5 vector signal is logged as
% 1x5xN, a column/1-D signal as Nx5 -> normalise to N x 5
P = squeeze(sig('path_info').Data);
if size(P, 1) ~= numel(log.t), P = P.'; end
log.Xp = P(:,1); log.Yp = P(:,2); log.psi_p = P(:,3);
log.kappa = P(:,4); log.psi_dot_des = P(:,5);

try
    info = sig('Info');                         % bus -> struct of timeseries
    log.X_block = reshape(info.InertFrm.Cg.Disp.X.Data, [], 1);
    log.Y_block = reshape(info.InertFrm.Cg.Disp.Y.Data, [], 1);
catch
    % Info bus not logged / different layout: cross-check is skipped
end
end
