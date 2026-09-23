function M = performance_metrics(log)
%PERFORMANCE_METRICS  Tracking accuracy and control effort of one run.
%
%   M = performance_metrics(log)   (log from simulate_closed_loop or
%                                  simulink_log_to_struct)
%
%   M.ey_max, M.ey_rms          [m]        lateral offset
%   M.epsi_max, M.epsi_rms      [deg]      heading error
%   M.delta_max, M.delta_rms    [deg]      steering command
%   M.steer_energy              [rad^2 s]  integral of delta^2 dt
%   M.ey_final, M.epsi_final    values at the end of the run

t  = log.t;
M.ey_max       = max(abs(log.ey));
M.ey_rms       = sqrt(mean(log.ey.^2));
M.epsi_max     = rad2deg(max(abs(log.epsi)));
M.epsi_rms     = rad2deg(sqrt(mean(log.epsi.^2)));
M.delta_max    = rad2deg(max(abs(log.delta)));
M.delta_rms    = rad2deg(sqrt(mean(log.delta.^2)));
M.steer_energy = trapz(t, log.delta.^2);
M.ey_final     = log.ey(end);
M.epsi_final   = rad2deg(log.epsi(end));
end
