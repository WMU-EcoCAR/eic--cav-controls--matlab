function mdl = build_cav_model(controller)
%BUILD_CAV_MODEL Build the ACC and lane centering model for one controller.
%   mdl = BUILD_CAV_MODEL('pid') builds cav_pid.slx and
%   mdl = BUILD_CAV_MODEL('mpc') builds cav_mpc.slx, from scratch, next to
%   this file. Both share the plant, the road and the lead vehicle, so the
%   two controllers are compared on the same car. The models read their
%   parameters from cav_params.m through InitFcn, so they also run when
%   opened by hand.
%
%   Ego vehicle   Vehicle Dynamics Blockset bicycle model, force input
%   Lead vehicle  a speed profile versus time, integrated along the lane
%   PID  ACC      gap PID (time-gap spacing) -> speed PID -> axle force
%   PID  lateral  steering PID on the lookahead error
%   MPC  ACC      Adaptive Cruise Control System block -> acceleration -> force
%   MPC  lateral  Lane Keeping Assist System block with previewed curvature

if nargin < 1
    controller = 'pid';
end
controller = validatestring(controller, {'pid', 'mpc'});
mdl = ['cav_' controller];
here = fileparts(mfilename('fullpath'));
if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
slxFile = fullfile(here, [mdl '.slx']);
if isfile(slxFile)
    delete(slxFile);
end
new_system(mdl);

set_param(mdl, 'InitFcn', ['P = cav_params(); road = make_road(P.road.segments, P.road.ds, P.road.transition); ' ...
    'mpcverbosity off;']);
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'ode4', ...
    'FixedStep', 'P.dt', 'StopTime', 'P.Tsim', ...
    'AutoInsertRateTranBlk', 'on', ...
    'ReturnWorkspaceOutputs', 'on', 'SignalLogging', 'on');

addLeadVehicle(mdl);
addEgoVehicle(mdl);
addPerception(mdl);
switch controller
    case 'pid'
        [latBlocks, accBlocks, steerSrc, forceSrc, extraLogs] = addPidControllers(mdl);
    case 'mpc'
        [latBlocks, accBlocks, steerSrc, forceSrc, extraLogs] = addMpcControllers(mdl);
end
add_line(mdl, steerSrc, 'Vehicle Body/1');
add_line(mdl, forceSrc, 'Front Share/1');
add_line(mdl, forceSrc, 'Rear Share/1');
addLogging(mdl, [extraLogs; {'steer', steerSrc; 'Fx', forceSrc}]);
nameSignals(mdl, {steerSrc, 'steer'; forceSrc, 'Fx'});

groupIntoSubsystem(mdl, 'Lead Vehicle', {'Clock', 'Lead Speed Profile', 'Lead Position'});
groupIntoSubsystem(mdl, 'Ego Vehicle', {'Vehicle Body', 'Position', 'Front Share', 'Rear Share'});
groupIntoSubsystem(mdl, 'Lane Centering', [{'Lane Geometry'} latBlocks]);
groupIntoSubsystem(mdl, 'ACC', [{'Spacing Policy'} accBlocks]);

Simulink.BlockDiagram.arrangeSystem(mdl);
save_system(mdl, slxFile);
end

function addLeadVehicle(mdl)
add_block('simulink/Sources/Clock', [mdl '/Clock']);
add_block('simulink/Lookup Tables/1-D Lookup Table', [mdl '/Lead Speed Profile'], ...
    'Table', 'P.lead.v', 'BreakpointsForDimension1', 'P.lead.t', ...
    'ExtrapMethod', 'Clip');
add_block('simulink/Continuous/Integrator', [mdl '/Lead Position'], ...
    'InitialCondition', 'P.lead.s0');
add_line(mdl, 'Clock/1', 'Lead Speed Profile/1');
add_line(mdl, 'Lead Speed Profile/1', 'Lead Position/1');
end

function addEgoVehicle(mdl)
blk = [mdl '/Vehicle Body'];
add_block('vehdynlibeom/Bicycle Model - Force Input', blk);
set_param(blk, 'm', 'P.ego.m', 'Izz', 'P.ego.Izz', 'a', 'P.ego.a', ...
    'b', 'P.ego.b', 'Cy_f', 'P.ego.Cyf', 'Cy_r', 'P.ego.Cyr', ...
    'xdot_o', 'P.ego.v0');
add_block('simulink/Signal Routing/Bus Selector', [mdl '/Position'], ...
    'OutputSignals', 'InertFrm.Cg.Disp.X,InertFrm.Cg.Disp.Y');
add_line(mdl, 'Vehicle Body/1', 'Position/1');
add_block('simulink/Math Operations/Gain', [mdl '/Front Share'], ...
    'Gain', 'P.ego.frontSplit');
add_block('simulink/Math Operations/Gain', [mdl '/Rear Share'], ...
    'Gain', '1 - P.ego.frontSplit');
add_line(mdl, 'Front Share/1', 'Vehicle Body/2');
add_line(mdl, 'Rear Share/1', 'Vehicle Body/3');
end

function addPerception(mdl)
% Ideal lane and radar measurements, shared by both controllers
addMatlabFunction(mdl, 'Lane Geometry', laneGeometryCode(), {'road', 'P'}, 'P.Ts');
add_line(mdl, 'Position/1', 'Lane Geometry/1');
add_line(mdl, 'Position/2', 'Lane Geometry/2');
add_line(mdl, 'Vehicle Body/4', 'Lane Geometry/3');
add_line(mdl, 'Vehicle Body/2', 'Lane Geometry/4');
addMatlabFunction(mdl, 'Spacing Policy', spacingCode(), {'P'}, 'P.Ts');
add_line(mdl, 'Lead Position/1', 'Spacing Policy/1');
add_line(mdl, 'Lane Geometry/1', 'Spacing Policy/2');
add_line(mdl, 'Vehicle Body/2', 'Spacing Policy/3');
end

function [latBlocks, accBlocks, steerSrc, forceSrc, logs] = addPidControllers(mdl)
add_block('simulink/Math Operations/Gain', [mdl '/Steer Error'], 'Gain', '-1');
add_block('simulink/Discrete/Discrete PID Controller', [mdl '/Steering PID'], ...
    'Controller', 'PID', 'P', 'P.lat.P', 'I', 'P.lat.I', 'D', 'P.lat.D', ...
    'N', 'P.lat.N', 'FilterMethod', 'Backward Euler', ...
    'SampleTime', 'P.Ts', 'LimitOutput', 'on', ...
    'UpperSaturationLimit', 'P.lat.maxSteer', ...
    'LowerSaturationLimit', '-P.lat.maxSteer', 'AntiWindupMode', 'clamping');
add_line(mdl, 'Lane Geometry/4', 'Steer Error/1');
add_line(mdl, 'Steer Error/1', 'Steering PID/1');

add_block('simulink/Discrete/Discrete PID Controller', [mdl '/Gap PID'], ...
    'Controller', 'PD', 'P', 'P.acc.gap.P', 'D', 'P.acc.gap.D', ...
    'N', 'P.acc.gap.N', 'FilterMethod', 'Backward Euler', ...
    'SampleTime', 'P.Ts', 'LimitOutput', 'on', ...
    'UpperSaturationLimit', 'P.acc.gap.dvMax', ...
    'LowerSaturationLimit', '-P.acc.gap.dvMax');
addMatlabFunction(mdl, 'ACC Arbitration', arbitrationCode(), {'P'}, 'P.Ts');
add_block('simulink/Math Operations/Sum', [mdl '/Speed Error'], 'Inputs', '+-');
add_block('simulink/Discrete/Discrete PID Controller', [mdl '/Speed PID'], ...
    'Controller', 'PI', 'P', 'P.acc.speed.P', 'I', 'P.acc.speed.I', ...
    'SampleTime', 'P.Ts', 'LimitOutput', 'on', ...
    'UpperSaturationLimit', 'P.acc.FxMax', ...
    'LowerSaturationLimit', 'P.acc.FxMin', 'AntiWindupMode', 'clamping', ...
    'InitialConditionForIntegrator', '150');
add_line(mdl, 'Spacing Policy/2', 'Gap PID/1');
add_line(mdl, 'Lead Speed Profile/1', 'ACC Arbitration/1');
add_line(mdl, 'Gap PID/1', 'ACC Arbitration/2');
add_line(mdl, 'Spacing Policy/1', 'ACC Arbitration/3');
add_line(mdl, 'ACC Arbitration/1', 'Speed Error/1');
add_line(mdl, 'Vehicle Body/2', 'Speed Error/2');
add_line(mdl, 'Speed Error/1', 'Speed PID/1');

latBlocks = {'Steer Error', 'Steering PID'};
accBlocks = {'Gap PID', 'ACC Arbitration', 'Speed Error', 'Speed PID'};
steerSrc = 'Steering PID/1';
forceSrc = 'Speed PID/1';
logs = {'vRef', 'ACC Arbitration/1'; 'mode', 'ACC Arbitration/2'};
end

function [latBlocks, accBlocks, steerSrc, forceSrc, logs] = addMpcControllers(mdl)
% Lane keeping: MPC on lateral deviation and relative yaw, with the road
% curvature previewed over the prediction horizon
addMatlabFunction(mdl, 'Curvature Preview', curvaturePreviewCode(), {'road', 'P'}, 'P.mpc.Ts');
% The MPC blocks take only literal values, so the parameters are written
% in at build time: rebuild after changing them in cav_params.m.
P = cav_params();
num = @(x) mat2str(x, 6);
add_block('mpclib/Automated Driving/Lane Keeping Assist System', [mdl '/Lane Keeping MPC'], ...
    'VehicleMass', num(P.ego.m), 'VehicleYawInertia', num(P.ego.Izz), ...
    'LengthToFront', num(P.ego.a), 'LengthToRear', num(P.ego.b), ...
    'FrontTireStiffness', num(P.ego.Cyf), 'RearTireStiffness', num(P.ego.Cyr), ...
    'InitialLongVel', num(P.ego.v0), ...
    'MinSteering', num(-P.lat.maxSteer), 'MaxSteering', num(P.lat.maxSteer), ...
    'Ts', num(P.mpc.Ts), 'PredictionHorizon', num(P.mpc.lat.horizon), ...
    'ControllerBehavior', num(P.mpc.lat.behavior));
add_line(mdl, 'Lane Geometry/1', 'Curvature Preview/1');
add_line(mdl, 'Vehicle Body/2', 'Curvature Preview/2');
add_line(mdl, 'Curvature Preview/1', 'Lane Keeping MPC/1');
add_line(mdl, 'Vehicle Body/2', 'Lane Keeping MPC/2');
add_line(mdl, 'Lane Geometry/2', 'Lane Keeping MPC/3');
add_line(mdl, 'Lane Geometry/3', 'Lane Keeping MPC/4');

% ACC: MPC on speed and spacing, same 6 m + 1.8 s spacing policy, then
% acceleration to axle force with an aerodynamic drag feedforward
addMatlabFunction(mdl, 'Radar', radarCode(), {'P'}, 'P.Ts');
add_block('simulink/Sources/Constant', [mdl '/Set Speed'], 'Value', 'P.acc.vSet');
add_block('simulink/Sources/Constant', [mdl '/Time Gap'], 'Value', 'P.mpc.acc.minTimeGap');
add_block('mpclib/Automated Driving/Adaptive Cruise Control System', [mdl '/ACC MPC'], ...
    'EgoModel', sprintf('tf(1, [%g 1 0])', P.mpc.acc.lag), ...
    'InitialEgoVelocity', num(P.ego.v0), 'DefaultSpacing', num(P.acc.d0), ...
    'MinAcceleration', num(P.mpc.acc.aMin), 'MaxAcceleration', num(P.mpc.acc.aMax), ...
    'Ts', num(P.mpc.Ts), 'PredictionHorizon', num(P.mpc.acc.horizon), ...
    'ControllerBehavior', num(P.mpc.acc.behavior));
addMatlabFunction(mdl, 'Force Command', forceCode(), {'P'}, 'P.Ts');
add_line(mdl, 'Spacing Policy/1', 'Radar/1');
add_line(mdl, 'Lead Speed Profile/1', 'Radar/2');
add_line(mdl, 'Vehicle Body/2', 'Radar/3');
add_line(mdl, 'Set Speed/1', 'ACC MPC/1');
add_line(mdl, 'Time Gap/1', 'ACC MPC/2');
add_line(mdl, 'Vehicle Body/2', 'ACC MPC/3');
add_line(mdl, 'Radar/1', 'ACC MPC/4');
add_line(mdl, 'Radar/2', 'ACC MPC/5');
add_line(mdl, 'ACC MPC/1', 'Force Command/1');
add_line(mdl, 'Vehicle Body/2', 'Force Command/2');

latBlocks = {'Curvature Preview', 'Lane Keeping MPC'};
accBlocks = {'Radar', 'Set Speed', 'Time Gap', 'ACC MPC', 'Force Command'};
steerSrc = 'Lane Keeping MPC/1';
forceSrc = 'Force Command/1';
logs = {'aCmd', 'ACC MPC/1'};
end

function addLogging(mdl, extra)
% Each row: signal name, source port
sig = [{
    'vEgo',   'Vehicle Body/2'
    'psi',    'Vehicle Body/4'
    'X',      'Position/1'
    'Y',      'Position/2'
    'sEgo',   'Lane Geometry/1'
    'ey',     'Lane Geometry/2'
    'epsi',   'Lane Geometry/3'
    'ePrev',  'Lane Geometry/4'
    'sLead',  'Lead Position/1'
    'vLead',  'Lead Speed Profile/1'
    'gap',    'Spacing Policy/1'
    'gapErr', 'Spacing Policy/2'
    }; extra];
for i = 1:size(sig, 1)
    blk = ['log_' sig{i, 1}];
    add_block('simulink/Sinks/To Workspace', [mdl '/' blk], ...
        'VariableName', sig{i, 1}, 'SaveFormat', 'Timeseries', ...
        'SampleTime', 'P.Ts');
    add_line(mdl, sig{i, 2}, [blk '/1'], 'autorouting', 'on');
end
end

function nameSignals(mdl, extra)
% Subsystem ports take the names of the signals that cross them
sig = [{
    'Lead Position/1',      'sLead'
    'Lead Speed Profile/1', 'vLead'
    'Vehicle Body/2',       'vEgo'
    'Vehicle Body/4',       'psi'
    }; extra];
for i = 1:size(sig, 1)
    parts = split(sig{i, 1}, '/');
    ports = get_param([mdl '/' parts{1}], 'PortHandles');
    line = get_param(ports.Outport(str2double(parts{2})), 'Line');
    set_param(line, 'Name', sig{i, 2});
end
end

function addMatlabFunction(mdl, name, code, params, sampleTime)
blk = [mdl '/' name];
add_block('simulink/User-Defined Functions/MATLAB Function', blk);
chart = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', blk);
chart.Script = code;
chart.ChartUpdate = 'DISCRETE';
chart.SampleTime = sampleTime;
for i = 1:numel(params)
    data = chart.find('-isa', 'Stateflow.Data', 'Name', params{i});
    data.Scope = 'Parameter';
    data.Tunable = false;
end
end

function groupIntoSubsystem(mdl, name, names)
before = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'SubSystem');
handles = cellfun(@(n) get_param([mdl '/' n], 'Handle'), names);
Simulink.BlockDiagram.createSubsystem(handles);
after = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'SubSystem');
created = setdiff(after, before);
sub = created{1};
set_param(sub, 'Name', name);
sub = [mdl '/' name];
% A signal that branches to several blocks inside gets a port named InN;
% give it the name of the signal instead
ports = get_param(sub, 'PortHandles');
inports = find_system(sub, 'SearchDepth', 1, 'BlockType', 'Inport');
for i = 1:numel(inports)
    if ~isempty(regexp(get_param(inports{i}, 'Name'), '^In\d+$', 'once'))
        n = str2double(get_param(inports{i}, 'Port'));
        signal = get_param(get_param(ports.Inport(n), 'Line'), 'Name');
        if ~isempty(signal)
            set_param(inports{i}, 'Name', signal);
        end
    end
end
Simulink.BlockDiagram.arrangeSystem(sub);
end

function code = laneGeometryCode()
code = strjoin({
'function [s, ey, epsi, ePrev] = lane_geometry(X, Y, psi, v, road, P)'
'%Arc length, lateral offset (left positive), heading error, and the'
'%lookahead error the steering PID drives to 0: the offset of a point L'
'%ahead along the heading, measured from the lane tangent at the car. It'
'%is zero only when the car is centered and aligned, so on a curve the'
'%integral term supplies the steady steering.'
'persistent idx'
'n = numel(road.x);'
'if isempty(idx)'
'    idx = nearest_index(road, X, Y, 1, n);'
'else'
'    idx = nearest_index(road, X, Y, max(1, idx - 40), min(n, idx + 200));'
'end'
'%Interpolate along the arc between centerline points, so heading and'
'%offset change smoothly instead of in steps of ds * curvature.'
'dx = X - road.x(idx);'
'dy = Y - road.y(idx);'
'along = cos(road.th(idx)) * dx + sin(road.th(idx)) * dy;'
'th = road.th(idx) + road.k(idx) * along;'
'ey = -sin(road.th(idx)) * dx + cos(road.th(idx)) * dy - road.k(idx) * along^2 / 2;'
's = road.s(idx) + along;'
'epsi = atan2(sin(psi - th), cos(psi - th));'
'L = P.lat.L0 + P.lat.Tla * max(v, 0);'
'ePrev = ey + L * sin(epsi);'
'end'
''
'function best = nearest_index(road, X, Y, lo, hi)'
'best = lo;'
'bestD = inf;'
'for k = lo:hi'
'    d = (road.x(k) - X)^2 + (road.y(k) - Y)^2;'
'    if d < bestD'
'        bestD = d;'
'        best = k;'
'    end'
'end'
'end'
}, newline);
end

function code = spacingCode()
code = strjoin({
'function [gap, gapErr] = spacing_policy(sLead, sEgo, vEgo, P)'
'%Bumper-to-bumper gap and its error from the constant time-gap policy.'
'gap = sLead - sEgo - (P.ego.length + P.lead.length) / 2;'
'gapErr = gap - (P.acc.d0 + P.acc.Th * vEgo);'
'end'
}, newline);
end

function code = arbitrationCode()
code = strjoin({
'function [vRef, mode] = acc_arbitration(vLead, dv, gap, P)'
'%Speed command: the set speed, or the lead speed plus the gap correction,'
'%whichever is lower. mode is 1 for speed control and 2 for gap control.'
'vRef = P.acc.vSet;'
'mode = 1;'
'if gap < P.acc.range'
'    vGap = max(vLead + dv, 0);'
'    if vGap < vRef'
'        vRef = vGap;'
'        mode = 2;'
'    end'
'end'
'end'
}, newline);
end

function code = curvaturePreviewCode()
code = strjoin({
'function kPreview = curvature_preview(s, v, road, P)'
'%Road curvature at each step of the MPC prediction horizon, assuming the'
'%car holds its current speed.'
'p = P.mpc.lat.horizon;'
'kPreview = zeros(1, p);'
'n = numel(road.k);'
'for j = 1:p'
'    i = round((s + max(v, 0) * P.mpc.Ts * j) / road.ds) + 1;'
'    kPreview(j) = road.k(min(max(i, 1), n));'
'end'
'end'
}, newline);
end

function code = radarCode()
code = strjoin({
'function [dRel, vRel] = radar(gap, vLead, vEgo, P)'
'%Relative distance and speed of the lead, or a clear road beyond range.'
'if gap < P.acc.range'
'    dRel = gap;'
'    vRel = vLead - vEgo;'
'else'
'    dRel = 1000;'
'    vRel = 0;'
'end'
'end'
}, newline);
end

function code = forceCode()
code = strjoin({
'function Fx = force_command(aCmd, v, P)'
'%Axle force for the commanded acceleration, plus aerodynamic drag.'
'Fx = P.ego.m * aCmd + P.ego.cDrag * v^2;'
'Fx = min(max(Fx, P.acc.FxMin), P.acc.FxMax);'
'end'
}, newline);
end
