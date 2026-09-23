function ctrl = init_model_workspace(design_file, block_mode, scenario)
%INIT_MODEL_WORKSPACE  Put everything the Simulink model needs in the base workspace.
%
%   ctrl = init_model_workspace()                        design from results/baseline_design.mat
%   ctrl = init_model_workspace(design_file)             (if it exists, else default pole placement)
%   ctrl = init_model_workspace(design_file, block_mode) block_mode: 'match_design' | 'raw'
%                                                        (see vdb_block_params)
%   ctrl = init_model_workspace(design_file, block_mode, scenario)
%                                                        scenario: see load_params; if omitted,
%                                                        the base-workspace variable 'scenario'
%                                                        is used when it exists, else 'baseline'
%
%   Variables created: veh, pth, pth_vec, Vx, delta_max, K, ctrl, sim_opt, blk.
%   Called by the model's PreLoadFcn callback and by run_baseline / tests.

src  = fileparts(mfilename('fullpath'));
proj = fileparts(src);
if nargin < 1 || isempty(design_file)
    design_file = fullfile(proj, 'results', 'baseline_design.mat');
end
if nargin < 3 || isempty(scenario)
    scenario = 'baseline';
    if evalin('base', 'exist(''scenario'', ''var'')')
        scenario = evalin('base', 'scenario');
    end
end

[veh, pth, sim_opt] = load_params(scenario);
if nargin >= 2 && ~isempty(block_mode), veh.block_mode = block_mode; end
blk = vdb_block_params(veh);

if exist(design_file, 'file')
    S = load(design_file, 'ctrl');
    ctrl = S.ctrl;
else
    [A, B] = error_model_linear(veh);
    ctrl = design_state_feedback(A, B, 'place', struct('poles', default_poles()));
end

assignin('base', 'veh',       veh);
assignin('base', 'blk',       blk);
assignin('base', 'pth',       pth);
assignin('base', 'pth_vec',   path_to_vec(pth));
assignin('base', 'Vx',        veh.Vx);
assignin('base', 'delta_max', veh.delta_max);
assignin('base', 'K',         ctrl.K);
assignin('base', 'ctrl',      ctrl);
assignin('base', 'sim_opt',   sim_opt);

% X-Y Monitor subsystem: update period of the live figure and on/off switch
% (the switch is left alone when the caller has already defined it, e.g.
% run_baseline sets xy_monitor_on = 0 for batch runs)
assignin('base', 'Ts_mon', 0.05);
if ~evalin('base', 'exist(''xy_monitor_on'', ''var'')')
    assignin('base', 'xy_monitor_on', 1);
end
end
