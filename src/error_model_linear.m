function [A, B, Bd, C, D] = error_model_linear(veh)
%ERROR_MODEL_LINEAR  Control-oriented path-tracking error model.
%
%   [A, B, Bd, C, D] = error_model_linear(veh)
%
%   State  x = [ey; ey_dot; epsi; epsi_dot],  input u = delta (front steer),
%   disturbance d = psi_dot_des = Vx*kappa (desired yaw rate of the path):
%
%       x_dot = A x + B u + Bd d ,      y = C x + D u = [ey; epsi]   (D = 0)
%
%   e.g.  sys = ss(A, B, C, D)  is the 2-output, 1-input plant for design.
%
%   Derived from the linear single-track model (small angles, Vx const.)
%
%       m  (Vy_dot + Vx r) = Cf*(delta - (Vy+lf r)/Vx) + Cr*(-(Vy-lr r)/Vx)
%       Izz r_dot          = lf*Cf*(delta - (Vy+lf r)/Vx) - lr*Cr*(-(Vy-lr r)/Vx)
%
%   together with the error kinematics  ey_dot = Vy + Vx*epsi,
%   epsi_dot = r - psi_dot_des  (Rajamani, "Vehicle Dynamics and Control",
%   Ch. 2-3, written here for per-axle cornering stiffnesses).
%
%   veh : struct with fields lf, lr, m, Izz, Cf, Cr, Vx  (see load_params)

lf = veh.lf; lr = veh.lr; m = veh.m; Izz = veh.Izz;
Cf = veh.Cf; Cr = veh.Cr; Vx = veh.Vx;

A = [ 0,                1,                     0,                 0;
      0, -(Cf + Cr)/(m*Vx),          (Cf + Cr)/m,   (-Cf*lf + Cr*lr)/(m*Vx);
      0,                0,                     0,                 1;
      0, -(Cf*lf - Cr*lr)/(Izz*Vx), (Cf*lf - Cr*lr)/Izz, -(Cf*lf^2 + Cr*lr^2)/(Izz*Vx) ];

B  = [ 0;  Cf/m;  0;  Cf*lf/Izz ];

Bd = [ 0;
       (-Cf*lf + Cr*lr)/(m*Vx) - Vx;
       0;
      -(Cf*lf^2 + Cr*lr^2)/(Izz*Vx) ];

C  = [ 1 0 0 0;
       0 0 1 0 ];

D  = [ 0;
       0 ];
end
