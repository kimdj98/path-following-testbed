function print_metrics_table(names, metrics, fid)
%PRINT_METRICS_TABLE  Pretty-print a comparison table of performance metrics.
%   print_metrics_table(names, metrics)        -> to the command window
%   print_metrics_table(names, metrics, fid)   -> to an open file handle
if nargin < 3, fid = 1; end
fprintf(fid, '%-34s %10s %10s %12s %11s %11s %11s %13s\n', 'design', ...
    'max|ey|[m]', 'rms ey[m]', 'max|epsi|[d]', 'rms epsi[d]', ...
    'max|del|[d]', 'rms del[d]', 'int del^2 dt');
for i = 1:numel(metrics)
    M = metrics{i};
    fprintf(fid, '%-34s %10.4f %10.4f %12.4f %11.4f %11.4f %11.4f %13.3e\n', ...
        names{i}, M.ey_max, M.ey_rms, M.epsi_max, M.epsi_rms, ...
        M.delta_max, M.delta_rms, M.steer_energy);
end
end
