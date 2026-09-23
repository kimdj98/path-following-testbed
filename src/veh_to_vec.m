function v = veh_to_vec(veh)
%VEH_TO_VEC  Pack vehicle parameters into a plain vector.
%   Used by the block-level functions (vehicle_body_3dof) so that the very
%   same code runs in MATLAB scripts and inside Simulink MATLAB Function
%   blocks (which receive the vector through a Constant block).
%
%   v = [lf, lr, m, Izz, Cf, Cr]
v = [veh.lf, veh.lr, veh.m, veh.Izz, veh.Cf, veh.Cr];
end
