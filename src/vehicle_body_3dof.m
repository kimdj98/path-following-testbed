function [dVy, dr, info] = vehicle_body_3dof(delta, Vx, Vy, r, v)
%VEHICLE_BODY_3DOF  Single-track (bicycle) lateral / yaw dynamics.
%
%   [dVy, dr, info] = vehicle_body_3dof(delta, Vx, Vy, r, v)
%
%   Re-implementation of the lateral/yaw part of the Simulink
%   "Vehicle Body 3DOF Single Track" block (Vehicle Dynamics Blockset) with
%   the option "Longitudinal velocity: external", i.e. the longitudinal speed
%   Vx is an input held constant by the environment and only the lateral
%   velocity Vy and yaw rate r are integrated.
%
%     alpha_f = delta - atan((Vy + lf*r)/Vx)        front slip angle
%     alpha_r =       - atan((Vy - lr*r)/Vx)        rear  slip angle
%     Fyf = Cf*alpha_f ,  Fyr = Cr*alpha_r          linear tyres
%     m  *(dVy + Vx*r) = Fyf*cos(delta) + Fyr
%     Izz* dr          = lf*Fyf*cos(delta) - lr*Fyr
%
%   v = [lf, lr, m, Izz, Cf, Cr]   (see veh_to_vec)
%   info = [Fyf, Fyr, alpha_f, alpha_r, ay]   with ay = dVy + Vx*r
%
% Code-generation compatible (used inside a Simulink MATLAB Function block).

lf = v(1); lr = v(2); m = v(3); Izz = v(4); Cf = v(5); Cr = v(6);

alpha_f = delta - atan2(Vy + lf * r, Vx);
alpha_r =       - atan2(Vy - lr * r, Vx);

Fyf = Cf * alpha_f;
Fyr = Cr * alpha_r;

dVy = (Fyf * cos(delta) + Fyr) / m - Vx * r;
dr  = (lf * Fyf * cos(delta) - lr * Fyr) / Izz;

ay   = dVy + Vx * r;
info = [Fyf, Fyr, alpha_f, alpha_r, ay];
end
