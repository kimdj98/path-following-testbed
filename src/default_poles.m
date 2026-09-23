function p = default_poles()
%DEFAULT_POLES  Closed-loop eigenvalues chosen for the baseline design.
%
%   Dominant complex pair  -2.5 +- 2.5i  (wn = 3.5 rad/s, zeta = 0.71,
%   ~1.5 s settling of the lateral offset) plus two faster real poles
%   (-6, -8) for the heading / yaw dynamics.  See run_baseline for the
%   sweep that motivated this choice.
p = [-2.5+2.5i, -2.5-2.5i, -6, -8];
end
