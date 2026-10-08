%[text] # Add Sensors to RoadRunner Scenario Using Simulink
%[text] ## Set Up RoadRunner Scenario — Simulink Interface
%[text] Configure your RoadRunner installation and project folder properties. Open the RoadRunner app.
rrApp=roadrunnerSetup;
%[text] ## Open Scene and Scenario
%[text] Open the scenario and create a `ScenarioSimulation` object to connect Simulink with the RoadRunner scenario.
openScenario(rrApp,"EIC")
rrSim = createSimulation(rrApp); %[output:2c4ad2b6]
%[text] Specify the step size for the RoadRunner scenario simulation. The Simulink model also uses the same step size. Units are in seconds.
%%
simStepSize = 0.1;
set(rrSim,"StepSize",simStepSize);
set(rrSim,"Logging","on");
%%
%[text] ## Inspect Simulink Model and Simulate Scenario
load("busDefinitionsForRRSim.mat")
modelName = 'SensorModel';
open_system(modelName)
%[text] To visualize the scenario and sensor detections, use the Bird's-Eye-Scope app. On the Simulink toolstrip, under **Review Results**, click **Bird's-Eye Scope**.
%[text] Start the scenario simulation. The model visualizes the 3D point cloud using the Point Cloud Viewer block during the simulation. In Bird's-Eye Scope, you can visualize the 2D point cloud, object detections and lane detections alongside ground truth values.
%%
%[text] ## Plot ego vehicle position
rrLog = get(rrSim,"SimulationLog"); %[output:435c2daa]
poseActor1 = rrLog.get('Pose','ActorID',1);
positionActor1_x = arrayfun(@(x) x.Pose(1,4),poseActor1)
positionActor1_y = arrayfun(@(x) x.Pose(2,4),poseActor1)
%%
%set(rrSim,"SimulationCommand","Start")
%[text] 
%[text] 
%[text] 
%[text] 
%[text] 
%[text] *Copyright 2022 The MathWorks, Inc.*

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"inline","rightPanelPercent":40}
%---
%[output:2c4ad2b6]
%   data: {"dataType":"text","outputData":{"text":"Connection status: 1\nConnected to RoadRunner Scenario server on localhost:54322, with client id {db47938b-2756-402d-bd7f-338131e0a4df}\n","truncated":false}}
%---
%[output:435c2daa]
%   data: {"dataType":"error","outputData":{"errorType":"runtime","text":"Error using <a href=\"matlab:matlab.lang.internal.introspective.errorDocCallback('Simulink.ScenarioSimulationMcos\/get')\" style=\"font-weight:bold\">Simulink.ScenarioSimulationMcos\/get<\/a>\nError in Scenario Server: Scenario does not exist\n\nError in Simulink.ScenarioSimulation\/get"}}
%---
