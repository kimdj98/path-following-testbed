function blk = vdb_block_params(veh, mode)
%VDB_BLOCK_PARAMS  Mask parameters for the "Vehicle Body 3DOF Single Track" block.
%
%   blk = vdb_block_params(veh, mode)      mode: 'match_design' | 'raw'
%                                          (default: veh.block_mode)
%
%   The Vehicle Dynamics Blockset block computes the axle lateral force as
%
%       Fy_axle = Cy * alpha * (Fz_axle / Fznom) * mu ,
%
%   i.e. Cy_f / Cy_r are cornering stiffnesses AT THE NOMINAL LOAD Fznom, not
%   at the actual axle loads.  With the block defaults (Fznom = 5000 N) and
%   Cy = 50000 N/rad typed in, the effective stiffnesses at the real loads
%   (Fz_f ~ 8.4 kN, Fz_r ~ 7.3 kN) are ~84 kN/rad and ~73 kN/rad and the
%   vehicle becomes almost neutral-steering (steady-state yaw-rate gain about
%   30 % above the design model).  Verified numerically in tests/run_tests.m.
%
%   'match_design' : aerodynamic forces off (no load transfer, the axle loads
%                    stay at the static values Fz_f = m g lr/L, Fz_r = m g lf/L)
%                    and Cy_f = Cf*Fznom/Fz_f, Cy_r = Cr*Fznom/Fz_r, so that
%                    the EFFECTIVE per-axle stiffnesses are exactly Cf and Cr.
%   'raw'          : Cy_f = Cf, Cy_r = Cr, block defaults otherwise
%                    (Fznom = 5000 N, Cd = 0.3, Cl = 0.1, Cpm = 0.1).
%
%   blk.Cf_eff / blk.Cr_eff report the effective stiffnesses at static load.

if nargin < 2 || isempty(mode), mode = veh.block_mode; end
L = veh.lf + veh.lr;

blk.mode = lower(mode);
blk.Fz_f = veh.m * veh.g * veh.lr / L;      % static front axle load [N]
blk.Fz_r = veh.m * veh.g * veh.lf / L;      % static rear  axle load [N]

switch blk.mode
    case 'match_design'
        blk.Fznom = 0.5 * veh.m * veh.g;
        blk.Cy_f  = veh.Cf * blk.Fznom / blk.Fz_f;
        blk.Cy_r  = veh.Cr * blk.Fznom / blk.Fz_r;
        blk.Cd = 0;   blk.Cl = 0;   blk.Cpm = 0;
    case 'raw'
        blk.Fznom = 5000;
        blk.Cy_f  = veh.Cf;
        blk.Cy_r  = veh.Cr;
        blk.Cd = 0.3; blk.Cl = 0.1; blk.Cpm = 0.1;
    otherwise
        error('vdb_block_params:mode', 'Unknown block mode "%s".', mode);
end

blk.Cf_eff = blk.Cy_f * blk.Fz_f / blk.Fznom;
blk.Cr_eff = blk.Cy_r * blk.Fz_r / blk.Fznom;
end
