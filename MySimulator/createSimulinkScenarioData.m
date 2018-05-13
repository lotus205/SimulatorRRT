function createSimulinkScenarioData(...
            roadCenters, laneSpecification, simStopTime,...
            scenarioReaderMATFileName)
% Create data for scenario reader block
% New scenario will not include ego vehicle because ego will be
% controlled by the simulation

% Create scenario container with one large sample step
scenarioStopTime = simStopTime*1.1;
scenario = drivingScenario('StopTime', 20);
%     'SampleTime', simStopTime,...
road(scenario, roadCenters, 'Lanes', laneSpecification);

% Add dummy vehicle as at least one car is required for the
% scenario reader block
dummyActor = vehicle(scenario, ...
    'ClassID', 1);
speed = 1;
distance = speed * scenarioStopTime;
startPosition = [-1000 0 0];
endPosition = startPosition + [distance 0 0 ];
waypoints = [startPosition; endPosition];
trajectory(dummyActor, waypoints, speed);
% % Add the ego car
% egoCar = vehicle(scenario, ...
%     'ClassID', 1, ...
%     'Position', [1.6 -1.8 0]);
% waypoints = [1.6 -1.8 0;
%     18.7 -2.3 0;
%     36.9 1.6 0;
%     50.4 8.7 0];
% speed = 1;
% trajectory(egoCar, waypoints, speed);

% Add the non-ego actors
bicycle = actor(scenario, ...
    'ClassID', 3, ...
    'Length', 1.7, ...
    'Width', 0.45, ...
    'Height', 1.7, ...
    'Position', [21.1 2 0]);
waypoints = [21.1 2 0;
    11 1.2 0;
    1.3 1.6 0];
speed = 3;
trajectory(bicycle, waypoints, speed);

pedestrian = actor(scenario, ...
    'ClassID', 4, ...
    'Length', 0.24, ...
    'Width', 0.45, ...
    'Height', 1.7, ...
    'Position', [15.8 2.7 0], ...
    'RCSPattern', [-8 -8;-8 -8]);
waypoints = [15.8 2.7 0;
    16 -1.5 0];
speed = 1.5;
trajectory(pedestrian, waypoints, speed);

truck = vehicle(scenario, ...
    'ClassID', 2, ...
    'Length', 8.2, ...
    'Width', 2.5, ...
    'Height', 3.5, ...
    'Position', [119.3 25.2 0]);
waypoints = [119.3 25.2 0;
    101.7 26.9 0;
    85.7 26.4 0;
    71.4 22.7 0;
    55.1 15.9 0;
    55.1 15.9 0;
    33.3 4.6 0;
    23.5 2.5 0];
speed = 8;
trajectory(truck, waypoints, speed);

%% Save the Scenario to a File format used by scenario reader
vehiclePoses = record(scenario); %#ok<NASGU>

ActorProfile = scenario.Actors;

% Obtain lane marking vertices and faces
[LaneMarkingVertices, LaneMarkingFaces] = laneMarkingVertices(scenario); %#ok<ASGLU>

% Obtain road boundaries from the scenario and convert them from cell to
% struct for saving
roads = roadBoundaries(scenario);
RoadBoundaries = cell2struct(roads, 'RoadBoundaries',1); %#ok<NASGU>

% Obtain the road network suitable for the scenario reader
RoadNetwork = driving.scenario.internal.getRoadNetwork(scenario);  %#ok<NASGU>

save(scenarioReaderMATFileName,...
    'LaneMarkingFaces', ...
    'LaneMarkingVertices',  ...
    'RoadBoundaries', ...
    'RoadNetwork', ...
    'vehiclePoses', ...
    'ActorProfile');

 plot(scenario,'Waypoints','on');