function ctrl = design_state_feedback(A, B, method, spec)
%DESIGN_STATE_FEEDBACK  Full-state feedback gain  u = -K x.
%
%   ctrl = design_state_feedback(A, B, 'place', struct('poles', p))
%       eigenvalue assignment: places eig(A - B*K) at p  (Control System
%       Toolbox PLACE; ACKER is used as fallback).
%
%   ctrl = design_state_feedback(A, B, 'lqr', struct('Q', Q, 'R', R))
%       linear-quadratic regulator minimising  int x'Qx + u'Ru dt.
%
%   ctrl.K       1x4 gain,  ctrl.poles  closed-loop eigenvalues,
%   ctrl.method  'place' | 'lqr',  ctrl.spec  the design specification.

switch lower(method)
    case 'place'
        try
            K = place(A, B, spec.poles);
        catch
            K = acker(A, B, spec.poles);
        end
    case 'lqr'
        K = lqr(A, B, spec.Q, spec.R);
    otherwise
        error('design_state_feedback:method', 'Unknown method "%s".', method);
end

ctrl.method = lower(method);
ctrl.K      = K;
ctrl.poles  = eig(A - B*K);
ctrl.spec   = spec;
end
