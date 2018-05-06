function splineData = helperSLGenerateSmoothPath(refPath, plannerConfig, costmapStruct)

%helperSLGenerateSmoothPath generate a smooth path by fitting a cubic spline 

% Copyright 2017-2018 The MathWorks, Inc.

persistent splineFitter

% Create a driving.Path object from reference path bus signal
resolutionFactor = 5;
numInterpolationSteps = resolutionFactor * max(3, ...
    ceil(plannerConfig.ConnectionDistance/costmapStruct.CellSize));

connMech  = driving.planning.DubinsConnectionMechanism(...
    plannerConfig.MinTurningRadius, numInterpolationSteps, ...
    plannerConfig.ConnectionDistance);

pathObj = driving.Path.create(refPath.KeyPoses, connMech, refPath.Cost);

% Initialize spline fitting object
if isempty(splineFitter)
    splineFitter = HelperCubicSplineFit(pathObj);
else
    splineFitter.setPoints(pathObj);
end

% Set as constant to avoid variable-size signal in splineData bus
numPoints = 1000;

splineData = fit(splineFitter, numPoints);

end