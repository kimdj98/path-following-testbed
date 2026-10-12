function K = default_gain(veh)
%DEFAULT_GAIN  Baseline state-feedback gain: eigenvalue assignment at default_poles().
%
%   Called by the Simulink model's InitFcn at the start of every run
%   (K = default_gain(veh)), so an edit of default_poles.m is picked up the
%   next time the model is run.  Scripts that simulate the model with a
%   different gain pass it with Simulink.SimulationInput/setVariable, which
%   takes precedence over the InitFcn (see run_baseline, my_design_template).
[A, B] = error_model_linear(veh);
ctrl = design_state_feedback(A, B, 'place', struct('poles', default_poles()));
K = ctrl.K;
end
