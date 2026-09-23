function [dX, dY] = global_coordinate(Vx, Vy, psi)
%GLOBAL_COORDINATE  "Vehicle Global Coordinate" block (kinematics).
%
%   [dX, dY] = global_coordinate(Vx, Vy, psi)
%
%   Rotates the body-frame velocity into the inertial frame; integrating
%   dX, dY gives the CG position (X, Y).
%
% Code-generation compatible (used inside a Simulink MATLAB Function block).

dX = Vx * cos(psi) - Vy * sin(psi);
dY = Vx * sin(psi) + Vy * cos(psi);
end
