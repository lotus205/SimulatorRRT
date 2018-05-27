function controlOutput = myMotionPlanning(costmapStruct, vehicleDimsStruct, ...
    currentPose, nextGoal, startPose, ...
    minIterations, connectionDistance, minTurningRadius, currentTime)

%helperSLMotionPlanning plan a feasible path.

% Copyright 2017-2018 The MathWorks, Inc.

persistent motionPlanner nextGoalPose controlSequence vehiclePose path

if isempty(motionPlanner)
    % Initialize vehicleDimensions object
    vehicleDims = vehicleDimensions( ...
        vehicleDimsStruct.Length, ...
        vehicleDimsStruct.Width, ...
        vehicleDimsStruct.Height, ...
        'Wheelbase',         vehicleDimsStruct.Wheelbase, ...
        'RearOverhang',      vehicleDimsStruct.RearOverhang, ...
        'WorldUnits',        char(vehicleDimsStruct.WorldUnits));
    
    % Initialize vehicleCostmap object
    costmap = vehicleCostmap(costmapStruct.Costs, ...
        'FreeThreshold',     costmapStruct.FreeThreshold, ...
        'OccupiedThreshold', costmapStruct.OccupiedThreshold, ...
        'MapLocation',       costmapStruct.MapExtent([1, 3]), ...
        'CellSize',          costmapStruct.CellSize, ...
        'VehicleDimensions', vehicleDims, ...
        'InflationRadius',   costmapStruct.InflationRadius);
    
    % Initialize pathPlannerRRT object
    motionPlanner = myPathPlannerRRT(costmap, ...
        'MinIterations',     minIterations, ...
        'ConnectionDistance',connectionDistance, ...
        'ConnectionMethod', 'Customize');
    
    nextGoalPose  = nextGoal;
    vehiclePose   = startPose;
    fprintf('My RRT init\n');
    [path, controlSequence] = plan(motionPlanner, vehiclePose, nextGoalPose);
    assignin('base', 'motionPlanner', motionPlanner);
    assignin('base', 'path', path);
    assignin('base', 'controlSequence', controlSequence);
end

% Plan a new path if there is a new goal pose
if ~isequal(nextGoalPose, nextGoal)
    nextGoalPose  = nextGoal;
    vehiclePose   = startPose;
    [path, controlSequence] = plan(motionPlanner, vehiclePose, nextGoalPose);  
    assignin('base', 'motionPlanner', motionPlanner);
    assignin('base', 'path', path);
    assignin('base', 'controlSequence', controlSequence);
end

% Output reference path as a bus signal

if(currentTime < size(controlSequence, 1) * 0.3)
    index = ceil(currentTime / 0.3);
    if(index == 0)
        index = 1;
    end
    controlOutput = controlSequence(index, :);
else
    controlOutput = [0 0];
    plot(motionPlanner,'Tree','on')
    set_param(gcs, 'SimulationCommand', 'stop');
end
