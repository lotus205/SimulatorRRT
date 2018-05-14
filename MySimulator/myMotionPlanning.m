function controlOutput = myMotionPlanning(costmapStruct, vehicleDimsStruct, ...
    currentPose, nextGoal, startPose, ...
    minIterations, connectionDistance, minTurningRadius)

%helperSLMotionPlanning plan a feasible path.

% Copyright 2017-2018 The MathWorks, Inc.

persistent motionPlanner nextGoalPose controlSequence vehiclePose

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
        'ConnectionDistance',connectionDistance);
    
    nextGoalPose  = nextGoal;
    vehiclePose   = startPose;
    controlSequence = plan(motionPlanner, vehiclePose, nextGoalPose);
end

% Plan a new path if there is a new goal pose
if ~isequal(nextGoalPose, nextGoal)
    nextGoalPose  = nextGoal;
    vehiclePose   = currentPose;
    controlSequence = plan(motionPlanner, vehiclePose, nextGoalPose);  
end

% Output reference path as a bus signal
controlOutput                    = struct;
controlOutput.steeringAng        = 1;
controlOutput.force              = 1;
