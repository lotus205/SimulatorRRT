
% createDrivingScenario Returns the drivingScenario defined in the Designer

% Generated by MATLAB(R) 9.4 and Automated Driving System Toolbox 1.2.
% Generated on: 14-May-2018 00:17:57

% Construct a drivingScenario object.
scenario = drivingScenario('StopTime', 20);

% Add all road segments
roadCenters = [0.770000000000003 0.0999999999999996 0;
    20.87 0 0;
    37.07 4.3 0;
    44.37 8.3 0;
    81.4 23.9 0;
    121 23.2 0];
laneSpecification = lanespec(2);
road(scenario, roadCenters, 'Lanes', laneSpecification);

% Add the ego car
egoCar = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [1.6 -1.8 0]);
waypoints = [1.6 -1.8 0;
    18.7 -2.3 0;
    35.1 1.3 0;
    54.5 12.4 0;
    78 21.4 0;
    92 23.6 0;
    117.7 21.9 0];
speed = 10;
trajectory(egoCar, waypoints, speed);

% Add the non-ego actors
bicycle = actor(scenario, ...
    'ClassID', 3, ...
    'Length', 1.7, ...
    'Width', 0.45, ...
    'Height', 1.7, ...
    'Position', [96.2 33 0]);
waypoints = [96.2 33 0;
    85 33 0];
speed = 5;
trajectory(bicycle, waypoints, speed);

vehiclePoses = record(scenario);
  plot(scenario,'Waypoints','on');