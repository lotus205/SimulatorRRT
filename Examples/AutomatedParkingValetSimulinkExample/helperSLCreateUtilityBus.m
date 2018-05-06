% helperSLCreateUtilityBus define Simulink buses used in the model.

% Copyright 2017-2018 The MathWorks, Inc.

% Spline
elemsSpline(1)                = Simulink.BusElement;
elemsSpline(1).Name           = 'ts';
elemsSpline(1).Dimensions     = [1000, 1];
elemsSpline(1).DimensionsMode = 'Fixed';
elemsSpline(1).DataType       = 'double';
elemsSpline(1).SampleTime     = -1;
elemsSpline(1).Complexity     = 'real';

elemsSpline(2)                = Simulink.BusElement;
elemsSpline(2).Name           = 'x';
elemsSpline(2).Dimensions     = [1000, 1];
elemsSpline(2).DimensionsMode = 'Fixed';
elemsSpline(2).DataType       = 'double';
elemsSpline(2).SampleTime     = -1;
elemsSpline(2).Complexity     = 'real';

elemsSpline(3)                = Simulink.BusElement;
elemsSpline(3).Name           = 'y';
elemsSpline(3).Dimensions     = [1000, 1];
elemsSpline(3).DimensionsMode = 'Fixed';
elemsSpline(3).DataType       = 'double';
elemsSpline(3).SampleTime     = -1;
elemsSpline(3).Complexity     = 'real';

elemsSpline(4)                = Simulink.BusElement;
elemsSpline(4).Name           = 'dx';
elemsSpline(4).Dimensions     = [1000, 1];
elemsSpline(4).DimensionsMode = 'Fixed';
elemsSpline(4).DataType       = 'double';
elemsSpline(4).SampleTime     = -1;
elemsSpline(4).Complexity     = 'real';

elemsSpline(5)                = Simulink.BusElement;
elemsSpline(5).Name           = 'dy';
elemsSpline(5).Dimensions     = [1000, 1];
elemsSpline(5).DimensionsMode = 'Fixed';
elemsSpline(5).DataType       = 'double';
elemsSpline(5).SampleTime     = -1;
elemsSpline(5).Complexity     = 'real';

elemsSpline(6)                = Simulink.BusElement;
elemsSpline(6).Name           = 'kappa';
elemsSpline(6).Dimensions     = [1000, 1];
elemsSpline(6).DimensionsMode = 'Fixed';
elemsSpline(6).DataType       = 'double';
elemsSpline(6).SampleTime     = -1;
elemsSpline(6).Complexity     = 'real';

splineDataBus                 = Simulink.Bus;
splineDataBus.Elements        = elemsSpline;

% Path
elemsPath(1)                  = Simulink.BusElement;
elemsPath(1).Name             = 'KeyPoses';
elemsPath(1).Dimensions       = [100, 3];
elemsPath(1).DimensionsMode   = 'Variable';
elemsPath(1).DataType         = 'double';
elemsPath(1).SampleTime       = -1;
elemsPath(1).Complexity       = 'real';

elemsPath(2)                  = Simulink.BusElement;
elemsPath(2).Name             = 'Cost';
elemsPath(2).Dimensions       = 1;
elemsPath(2).DimensionsMode   = 'Fixed';
elemsPath(2).DataType         = 'double';
elemsPath(2).SampleTime       = -1;
elemsPath(2).Complexity       = 'real';

elemsPath(3)                  = Simulink.BusElement;
elemsPath(3).Name             = 'ConnectionMethod';
elemsPath(3).Dimensions       = [1, 11];
elemsPath(3).DimensionsMode   = 'Variable';
elemsPath(3).DataType         = 'uint8';
elemsPath(3).SampleTime       = -1;
elemsPath(3).Complexity       = 'real';

pathBus                       = Simulink.Bus;
pathBus.Elements              = elemsPath;

% Speed configuration
elemsSpeedConfig(1)                  = Simulink.BusElement;
elemsSpeedConfig(1).Name             = 'StartSpeed';
elemsSpeedConfig(1).Dimensions       = 1;
elemsSpeedConfig(1).DimensionsMode   = 'Fixed';
elemsSpeedConfig(1).DataType         = 'double';
elemsSpeedConfig(1).SampleTime       = -1;
elemsSpeedConfig(1).Complexity       = 'real';

elemsSpeedConfig(2)                  = Simulink.BusElement;
elemsSpeedConfig(2).Name             = 'EndSpeed';
elemsSpeedConfig(2).Dimensions       = 1;
elemsSpeedConfig(2).DimensionsMode   = 'Fixed';
elemsSpeedConfig(2).DataType         = 'double';
elemsSpeedConfig(2).SampleTime       = -1;
elemsSpeedConfig(2).Complexity       = 'real';

elemsSpeedConfig(3)                  = Simulink.BusElement;
elemsSpeedConfig(3).Name             = 'MaxSpeed';
elemsSpeedConfig(3).Dimensions       = 1;
elemsSpeedConfig(3).DimensionsMode   = 'Fixed';
elemsSpeedConfig(3).DataType         = 'double';
elemsSpeedConfig(3).SampleTime       = -1;
elemsSpeedConfig(3).Complexity       = 'real';

speedConfigBus                       = Simulink.Bus;
speedConfigBus.Elements              = elemsSpeedConfig;

% Planner configuration
elemsPlannerConfig(1)                = Simulink.BusElement;
elemsPlannerConfig(1).Name           = 'ConnectionDistance';
elemsPlannerConfig(1).Dimensions     = 1;
elemsPlannerConfig(1).DimensionsMode = 'Fixed';
elemsPlannerConfig(1).DataType       = 'double';
elemsPlannerConfig(1).SampleTime     = -1;
elemsPlannerConfig(1).Complexity     = 'real';

elemsPlannerConfig(2)                = Simulink.BusElement;
elemsPlannerConfig(2).Name           = 'MinIterations';
elemsPlannerConfig(2).Dimensions     = 1;
elemsPlannerConfig(2).DimensionsMode = 'Fixed';
elemsPlannerConfig(2).DataType       = 'double';
elemsPlannerConfig(2).SampleTime     = -1;
elemsPlannerConfig(2).Complexity     = 'real';

elemsPlannerConfig(3)                = Simulink.BusElement;
elemsPlannerConfig(3).Name           = 'GoalTolerance';
elemsPlannerConfig(3).Dimensions     = [1 3];
elemsPlannerConfig(3).DimensionsMode = 'Fixed';
elemsPlannerConfig(3).DataType       = 'double';
elemsPlannerConfig(3).SampleTime     = -1;
elemsPlannerConfig(3).Complexity     = 'real';

elemsPlannerConfig(4)                = Simulink.BusElement;
elemsPlannerConfig(4).Name           = 'MinTurningRadius';
elemsPlannerConfig(4).Dimensions     = 1;
elemsPlannerConfig(4).DimensionsMode = 'Fixed';
elemsPlannerConfig(4).DataType       = 'double';
elemsPlannerConfig(4).SampleTime     = -1;
elemsPlannerConfig(4).Complexity     = 'real';

plannerConfigBus                     = Simulink.Bus;
plannerConfigBus.Elements            = elemsPlannerConfig;


