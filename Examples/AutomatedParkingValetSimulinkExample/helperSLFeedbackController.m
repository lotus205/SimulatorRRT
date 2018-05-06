function [speed, steering, reachGoal] = helperSLFeedbackController( ...
    pose, vehicleDimsStruct, refPath, refSpeed, startSteer, startSpeed)

%helperSLFeedbackController compute control commands to drive the vehicle.

% Copyright 2017-2018 The MathWorks, Inc.

persistent controller vehicleDims

if isempty(vehicleDims)
    vehicleDims = vehicleDimensions( ...
        vehicleDimsStruct.Length, ...
        vehicleDimsStruct.Width, ...
        vehicleDimsStruct.Height, ...
        'Wheelbase',    vehicleDimsStruct.Wheelbase, ...
        'RearOverhang', vehicleDimsStruct.RearOverhang, ...
        'WorldUnits',   char(vehicleDimsStruct.WorldUnits));
end

% Initialize feedback controller object
if isempty(controller)
    controller = HelperFeedbackController(refPath, refSpeed, vehicleDims);
    
    controller.SteeringAngle   = startSteer;
    controller.FrontWheelSpeed = startSpeed; 
else
    controller.setProfile(refPath, refSpeed);
end

if ~controller.ReachGoal
    pose(3)           = mod(pose(3), 360);
    [speed, steering] = updateControlCommands(controller, pose);
    reachGoal         = controller.ReachGoal;
else
    speed             = controller.FrontWheelSpeed;
    steering          = controller.SteeringAngle;
    reachGoal         = controller.ReachGoal;
    
    % Reset flag for the next path segment
    resetReachGoal(controller);  
end

end