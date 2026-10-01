function P = cav_params()
%CAV_PARAMS Parameters for the ACC and lane centering simulation.
%   P = CAV_PARAMS() returns every tunable number in one struct: the ego
%   vehicle, the road, the lead vehicle's speed profile, and the gains of
%   the three PID controllers. Change values here, not in the model.

% Timing
P.Tsim = 60;      % simulation length [s]
P.dt = 0.01;      % solver step for the vehicle dynamics [s]
P.Ts = 0.02;      % controller sample time, 50 Hz [s]

% Ego vehicle: a mid-size sedan on the Vehicle Dynamics Blockset bicycle model
P.ego.m = 2000;      % mass [kg]
P.ego.Izz = 4000;    % yaw inertia [kg m^2]
P.ego.a = 1.4;       % CG to front axle [m]
P.ego.b = 1.6;       % CG to rear axle [m]
% Cornering stiffness per tire (the block multiplies by 2 tires per axle),
% about 15 times the tire load per radian. The block's defaults (12e3,
% 11e3) let this car slide at 8 deg of body slip.
P.ego.Cyf = 80e3;    % front tire cornering stiffness [N/rad]
P.ego.Cyr = 75e3;    % rear tire cornering stiffness [N/rad]
P.ego.v0 = 20;       % initial speed [m/s]
P.ego.length = 4.8;  % body length, for the bumper-to-bumper gap [m]
P.ego.width = 1.9;   % body width, for the lane departure check [m]
P.ego.frontSplit = 0.6;  % share of drive and brake force on the front axle
% Drag force = cDrag * v^2; the block's air density (101325 Pa, 273 K) times
% 0.5 * Cd 0.3 * frontal area 2 m^2. The MPC force command feeds it forward.
P.ego.cDrag = 0.39;

% Road: straight, left curve, straight, right curve, straight
% Each row is [length (m), radius (m); 0 radius means straight]. Positive
% radius turns left.
P.road.segments = [200 0; 393 250; 200 0; 314 -200; 300 0];
P.road.ds = 0.5;          % centerline point spacing [m]
P.road.transition = 60;   % clothoid length where curvature ramps [m]
P.road.laneWidth = 3.7;   % [m]

% Lead vehicle: speed profile versus time, linear between breakpoints
% It starts beyond radar range, so the ego first cruises at the set speed,
% then closes in, follows, rides out a hard brake and the recovery.
P.lead.s0 = 130;                             % starts 130 m ahead [m]
P.lead.t = [0 22 25 35 40 60];               % [s]
P.lead.v = [20 20 11 11 20 20];              % [m/s], brakes at 3 m/s^2
P.lead.length = 4.8;

% ACC: constant time-gap spacing policy feeding a speed loop
P.acc.vSet = 25;        % driver set speed [m/s]
P.acc.d0 = 6;           % standstill gap [m]
P.acc.Th = 1.8;         % time gap [s]
P.acc.range = 120;      % radar range; beyond it the lead is ignored [m]
P.acc.gap.P = 0.25;     % gap PID: m/s of speed command per m of gap error
P.acc.gap.I = 0;
P.acc.gap.D = 0.15;
P.acc.gap.N = 5;       % derivative filter [rad/s]
P.acc.gap.dvMax = 8;    % speed correction limits [m/s]
P.acc.speed.P = 900;    % speed PID: N of force per m/s of speed error
P.acc.speed.I = 120;
P.acc.speed.D = 0;
P.acc.FxMax = 0.35 * P.ego.m * 9.81;   % drive force limit [N]
P.acc.FxMin = -0.6 * P.ego.m * 9.81;   % brake force limit [N]

% Lane centering: PID on the lateral offset of a lookahead point
P.lat.L0 = 6;           % lookahead at standstill [m]
P.lat.Tla = 0.6;        % lookahead time, so lookahead = L0 + Tla*v [m]
P.lat.P = 0.12;         % rad of steer per m of lookahead error
P.lat.I = 0.06;
P.lat.D = 0.005;
P.lat.N = 20;           % derivative filter [rad/s]
P.lat.maxSteer = deg2rad(30);   % road wheel angle limit [rad]

% MPC alternative: Model Predictive Control Toolbox ACC and lane keeping
% blocks, with the same set speed, standstill gap and steering limit as the
% PID controllers. The MPC treats the spacing policy as a hard minimum, not
% a target, so its time gap is set lower; it then follows at about 1.85 s,
% close to the PID's 1.8 s target.
P.mpc.Ts = 0.1;             % MPC sample time [s]
P.mpc.acc.horizon = 30;     % prediction horizon, 3 s [steps]
P.mpc.acc.minTimeGap = 1.5; % minimum time gap the MPC must keep [s]
P.mpc.acc.lag = 0.15;       % ego acceleration lag in the MPC's model [s]; 0.3
                            % rides the gap constraint into 4 m/s^2 brake spikes
P.mpc.acc.aMin = -4;        % comfort and ISO 15622-style limits [m/s^2]
P.mpc.acc.aMax = 2;
P.mpc.acc.behavior = 0.5;   % 0 smooth ... 1 aggressive
P.mpc.lat.horizon = 20;     % prediction horizon and curvature preview, 2 s [steps]
P.mpc.lat.behavior = 0.5;
end
