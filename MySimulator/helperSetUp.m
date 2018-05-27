% Set up Script for the Lane Keeping Assist (LKA) Example
%
% This script initializes the LKA example model. It loads necessary control
% constants and sets up the buses required for the referenced model
%
%   This is a helper script for example purposes and may be removed or
%   modified in the future.

%   Copyright 2017 The MathWorks, Inc.

%% General Model Parameters
Ts = 0.1;               % Simulation sample time                (s)

%% Ego Car Parameters
% Dynamics modeling parameters
data_vehicle;

%% Controller parameter
PredictionHorizon = 30; % Number of steps for preview    (N/A)

%% Bus Creation
% Create the bus of actors from the scenario reader
modelName = 'MoveMove';
wasModelLoaded = bdIsLoaded(modelName);
if ~wasModelLoaded
    load_system(modelName)
end
% load the bus for scenario reader
blk=find_system(modelName,'System','helperScenarioReader');
s = get_param(blk{1},'PortHandles');
get(s.Outport(1),'SignalHierarchy');

%% Create scenario and road specifications
[scenario,roadCenters,laneSpecification] = createDoubleCurveScenario;

% You can use Driving Scenario Designer to explore the scenario
% drivingScenarioDesigner(scenario)

%% Generate data for Simulink simulation  
[driverPath,x0_ego,y0_ego,v0_ego,yaw0_ego,simStopTime] = ...
    createDriverPath(scenario,6);

scenario = createSimulinkScenarioData(...
        roadCenters, laneSpecification, simStopTime,...
        'lkaScenarioSimulink.mat');

costmap = creatMyCostMap();

fprintf("setup called\n");
