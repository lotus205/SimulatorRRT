function [scenario,roadCenters,laneSpecification] = createDoubleCurveScenario(plotScenario) 
% Create scenario and road specifications

if nargin < 1
    plotScenario = false;
end


% Construct a drivingScenario object.
scenario = drivingScenario;

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
    36.9 1.6 0;
    50.4 8.7 0];
speed = 1;
trajectory(egoCar, waypoints, speed);

if plotScenario
     plot(scenario,'Waypoints','on');
end