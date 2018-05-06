function [nextGoal, motionPlannerConfig, speedProfilerConfig, stopMotion] = ...
    helperSLBehaviorPlanning(costmapStruct, vehicleDimsStruct, routePlan, ...
    currentPose, currentSpeed, isGoalReached, startPose, startSpeed)

%helperSLBehaviorPlanning generate navigation tasks.
%   This helper function converts structs costmapStruct and vehicleDimsStruct,
%   created in Mask Initialization, back to objects as vehicleCostmap object 
%   and vehicleDimensions object, respectivley. It also generates a sequence 
%   of navigation tasks by providing a new destination and configurations  
%   for the Motion Planner block and the Trajectory Generator block.
%
%   See also vehicleCostmap and vehicleDimensions

% Copyright 2017-2018 The MathWorks, Inc.

persistent behavioralPlanner costmap
persistent nextGoalPose plannerConfig speedConfig
persistent pose speed

if isempty(behavioralPlanner)
    % Initialize vehicle dimensions object
    vehicleDims = vehicleDimensions( ...
        vehicleDimsStruct.Length, ...
        vehicleDimsStruct.Width, ...
        vehicleDimsStruct.Height, ...
        'Wheelbase',        vehicleDimsStruct.Wheelbase, ...
        'RearOverhang',     vehicleDimsStruct.RearOverhang, ...
        'WorldUnits',       char(vehicleDimsStruct.WorldUnits));
    
    % Initialize costmap object
    costmap = vehicleCostmap(costmapStruct.Costs, ...
        'FreeThreshold',     costmapStruct.FreeThreshold, ...
        'OccupiedThreshold', costmapStruct.OccupiedThreshold, ...
        'MapLocation',       costmapStruct.MapExtent([1, 3]), ...
        'CellSize',          costmapStruct.CellSize, ...
        'VehicleDimensions', vehicleDims, ...
        'InflationRadius',   costmapStruct.InflationRadius);
    
    % Initialize pose and speed
    pose  = startPose;
    speed = startSpeed;
    
    % Initialize planner
    steerLimit = 35;
    behavioralPlanner = HelperBehavioralPlanner(costmap, ...
        struct2table(routePlan), vehicleDims, steerLimit);
    [nextGoalPose, plannerConfig, speedConfig] = requestManeuver( ...
        behavioralPlanner, pose, speed);
end

pose  = currentPose;
speed = currentSpeed;

disTol = 0.5;
% Plan a new path when reaching the goal
if isGoalReached && ~reachedDestination(behavioralPlanner) && ...
        norm(pose(1:2) - nextGoalPose(1:2)) < disTol
    [nextGoalPose, plannerConfig, speedConfig] = requestManeuver( ...
        behavioralPlanner, pose, speed);
end

% Stop motion when reaching the destination
stopMotion = isGoalReached && reachedDestination(behavioralPlanner) && ...
    norm(pose(1:2) - nextGoalPose(1:2)) < disTol;

nextGoal            = nextGoalPose;
motionPlannerConfig = plannerConfig;
speedProfilerConfig = speedConfig;

end
