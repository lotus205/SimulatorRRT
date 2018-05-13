function createSimulinkScenarioData(...
            roadCenters, laneSpecification, simStopTime,...
            scenarioReaderMATFileName)
% Create data for scenario reader block
% New scenario will not include ego vehicle because ego will be
% controlled by the simulation

% Create scenario container with one large sample step
scenarioStopTime = simStopTime*1.1;
scenario = drivingScenario(...
    'StopTime', scenarioStopTime);
%     'SampleTime', simStopTime,...
road(scenario, roadCenters, 'Lanes', laneSpecification);

% Add dummy vehicle as at least one car is required for the
% scenario reader block
dummyActor = vehicle(scenario, ...
    'ClassID', 1);
speed = 10;
distance = speed * scenarioStopTime;
startPosition = [0 0 0];
endPosition = startPosition + [distance 0 0 ];
waypoints = [startPosition; endPosition];
trajectory(dummyActor, waypoints, speed);

% Add the non-ego actors
bicycle = actor(scenario, ...
    'ClassID', 3, ...
    'Length', 3.7, ...
    'Width', 0.45, ...
    'Height', 1.7, ...
    'Position', [0 3.8 0]);
speed = 5;
distance = speed * scenarioStopTime;
startPosition = [0 3.8 0];
endPosition = startPosition + [distance 0 0 ];
waypoints = [startPosition; endPosition];
trajectory(bicycle, waypoints, speed);

pedestrian = actor(scenario, ...
    'ClassID', 4, ...
    'Length', 0.14, ...
    'Width', 0.45, ...
    'Height', 1.7, ...
    'Position', [0 -3.8 0], ...
    'RCSPattern', [-8 -8;-8 -8]);
speed = 3;
distance = speed * scenarioStopTime;
startPosition = [0 -3.8 0];
endPosition = startPosition + [distance 0 0 ];
waypoints = [startPosition; endPosition];
trajectory(pedestrian, waypoints, speed);

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
