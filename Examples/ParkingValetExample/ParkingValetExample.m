%% Automated Parking Valet
% This example shows how to construct an automated parking valet system. In
% this example, you learn about tools and techniques in support of path
% planning, trajectory generation, and vehicle control. For a Simulink(R)
% version of this example, see
% <docid:driving_examples.mw_b8684291-90c7-415b-97db-725062fe2674 Automated
% Parking Valet in Simulink>.
%
% Copyright 2017-2018 The MathWorks, Inc.

%% Overview
% Automatically parking a car that is left in front of a parking lot is a
% challenging problem. The vehicle's automated systems are expected to take
% over control and steer the vehicle to an available parking spot. Such a
% function makes use of multiple on-board sensors. For example:
%
% * Front and side cameras for detecting lane markings, road signs (stop
%   signs, exit markings, etc.), other vehicles, and pedestrians
% * Lidar and ultrasound sensors for detecting obstacles and accurate
%   distance measurements
% * Ultrasound sensors for obstacle detection
% * IMU and wheel encoders for dead reckoning   
%
% On-board sensors are used to perceive the environment around the vehicle.
% The perceived environment includes an understanding of road markings to
% interpret road rules and infer drivable regions, recognition of
% obstacles, and detection of available parking spots.
%
% As the vehicle sensors perceive the world, the vehicle must plan a path
% through the environment towards a free parking spot and execute a
% sequence of control actions needed to drive to it. While doing so, it
% must respond to dynamic changes in the environment, such as pedestrians
% crossing its path, and readjust its plan.
%
% This example implements a subset of features required to implement such a
% system. It focuses on planning a feasible path through the environment,
% and executing the actions needed to traverse the path. Map creation and
% dynamic obstacle avoidance are excluded from this example.

%% Environment Model
% The environment model represents a map of the environment. For a parking
% valet system, this map includes available and occupied parking spots,
% road markings, and obstacles such as pedestrians or other vehicles.
% Occupancy maps are a common representation for this form of environment
% model. Such a map is typically built using Simultaneous Localization and
% Mapping (SLAM) by integrating observations from lidar and camera sensors.
% This example concentrates on a simpler scenario, where a map is already
% provided, for example, by a vehicle-to-infrastructure (V2X) system or a
% camera overlooking the entire parking space. It uses a static map of a
% parking lot and assumes that the self-localization of the vehicle is
% accurate.

%%
% The parking lot example used in this example is composed of three
% occupancy grid layers.
% 
% * Stationary obstacles: This layer contains stationary obstacles like
%   walls, barriers, and bounds of the parking lot.
% * Road markings: This layer contains occupancy information pertaining to
%   road markings, including road markings for parking spaces.
% * Parked cars: This layer contains information about which parking spots
%   are already occupied.
%
% Each map layer contains different kinds of obstacles that represent
% different levels of danger for a car navigating through it. With this
% structure, each layer can be handled, updated, and maintained
% independently. 
%
% Load and display the three map layers. In each layer, dark cells
% represent occupied cells, and light cells represent free cells.
mapLayers = loadParkingLotMapLayers;
plotMapLayers(mapLayers)

%%
% For simplicity, combine the three layers into a single costmap.
costmap = combineMapLayers(mapLayers);

figure,
plot(costmap, 'Inflation', 'off')
legend off

%%
% The combined |costmap| is a |<matlab:doc('vehicleCostmap')
% vehicleCostmap>| object, which represents the vehicle environment as a
% 2-D occupancy grid. Each grid in the cell has values between 0 and 1,
% representing the cost of navigating through the cell. Obstacles have a
% higher cost, while free space has a lower cost. A cell is considered an
% obstacle if its cost is higher than the |OccupiedThreshold| property, and
% free if its cost is lower than the |FreeThreshold| property.

%%
% The |costmap| covers the entire 75m-by-50m parking lot area, divided into
% 0.5m-by-0.5m square cells.

costmap.MapExtent % [x, y, width, height] in meters

costmap.CellSize    % cell size in meters

%%
% Create a |<matlab:doc('vehicleDimensions') vehicleDimensions>| object for
% storing the dimensions of the vehicle that will park automatically. Also
% define the maximum steering angle of the vehicle. This value determines
% the limits on the turning radius during motion planning and control.
vehicleDims      = vehicleDimensions;
maxSteeringAngle = 35; % in degrees

%%
% Update the |VehicleDimensions| property of the costmap with the
% dimensions of the vehicle to park. This setting adjusts the extent of
% inflation in the map around obstacles to correspond to the size of the
% vehicle being parked, ensuring that collision-free paths can be found
% through the parking lot.
costmap.VehicleDimensions = vehicleDims;

%%
% Define the starting pose of the vehicle. The pose is obtained through
% localization, which is left out of this example for simplicity. The
% vehicle pose is specified as $[x,y,\theta]$, in world coordinates, with
% $(x,y)$ representing the position of the center of the vehicle's rear
% axle in
% <matlab:helpview(fullfile(docroot,'driving','helptargets.map'),'drivingCoordinates');
% world coordinate system> and $\theta$ representing the orientation of the
% vehicle with respect to world X axis.

currentPose = [4 12 0]; % [x, y, theta]

%% Behavioral Layer
% Planning involves organizing all pertinent information into hierarchical
% layers. Each successive layer is responsible for a more fine-grained
% task. The behavioral layer [1] sits at the top of this stack, and is
% responsible for activating and managing the different parts of the
% mission by supplying a sequence of navigation tasks. The behavioral layer
% assembles information from all relevant parts of the system, including:
%
% * Localization: The behavioral layer inspects the localization module for
%   an estimate of the current location of the vehicle.
% * Environment model: Perception and sensor fusion systems report a map of
%   the environment around the vehicle. 
% * Determining a parking spot: The behavioral layer analyzes the map to
%   determine the closest available parking spot.
% * Finding a global route: A routing module calculates a global route
%   through the road network obtained either from a mapping service or from
%   V2X infrastructure. Decomposing the global route as a series of road
%   links allows the trajectory for each link to be planned differently.
%   For example, the final parking maneuver requires a different speed
%   profile than the approach to the parking spot. In a more general
%   setting, this becomes crucial for navigating through streets that
%   involve different speed limits, numbers of lanes, and road signs.
%
% Rather than rely on vehicle sensors to build a map of the environment,
% this example uses a map that comes from a smart parking lot via
% vehicle-to-infrastructure (V2X) communication. For simplicity, assume
% that the map is in the form of an occupancy grid, with road links and
% locations of available parking spots provided by V2X.

%%
% The |<matlab:edit('HelperBehavioralPlanner') HelperBehavioralPlanner>|
% class mimics an interface of a behavioral planning layer. The
% |<matlab:edit('HelperBehavioralPlanner') HelperBehavioralPlanner>| is
% created using the map and the global route plan. This example uses a
% static global route plan stored in a MATLAB table, but typically a
% routing algorithm provided by the local parking infrastructure or a
% mapping service determines this plan. The global route plan is described
% as a sequence of lane segments to traverse to reach a parking spot.

%%
% Load the MAT-file containing a route plan stored in a table. The table
% has three variables : |StartPose|, |EndPose|, and |Attributes|.
% |StartPose| and |EndPose| specify the start and end poses of the segment,
% expressed as $(x,y,\theta)$. |Attributes| specifies properties of the
% segment like the speed limit.
data = load('routePlan.mat');
routePlan = data.routePlan %#ok<NOPTS>

%%
% Plot a vehicle at the current pose, and along each goal in the route
% plan.

% Plot vehicle at current pose
hold on
helperPlotVehicle(currentPose, vehicleDims, 'DisplayName', 'Current Pose')
legend

for n = 1 : height(routePlan)
    % Extract the goal waypoint
    vehiclePose = routePlan{n, 'EndPose'};
    
    % Plot the pose
    legendEntry = sprintf('Goal %i', n);
    helperPlotVehicle(vehiclePose, vehicleDims, 'DisplayName', legendEntry);
end
hold off

%%
% Create the behavioral planner helper object. The |requestManeuver| method
% is used to request a stream of navigation tasks from the behavioral
% planner until the destination is reached. 
behavioralPlanner = HelperBehavioralPlanner(costmap, routePlan, vehicleDims, maxSteeringAngle);

%%
% The vehicle navigates each path segment using these steps:
%
% # *Motion Planning:* Plan a feasible path through the environment map
% using the optimal rapidly exploring random tree (RRT*) algorithm
% (|<matlab:doc('pathPlannerRRT') pathPlannerRRT>|).
% # *Path Smoothing:* Smooth the reference path by fitting splines
% (|<matlab:edit('HelperCubicSplineFit') HelperCubicSplineFit>|).
% # *Trajectory Generation:* Convert the smoothed path into a trajectory by
% generating a speed profile (|<matlab:edit('HelperSpeedProfileGenerator') HelperSpeedProfileGenerator>|).
% # *Vehicle Control:* Use a feedback controller to execute the trajectory
% (|<matlab:edit('HelperFeedbackController') HelperFeedbackController>|).
% # Check if the vehicle reached the destination (|<matlab:edit('HelperBehavioralPlanner/reachedDestination') reachedDestination>|).
% 
% The rest of this example describes these steps in detail, before 
% assembling them into a complete solution.


%% Motion Planning
% Given a global route, motion planning can be used to plan a path through
% the environment to reach each intermediate waypoint, until the vehicle
% reaches the final destination. The planned path for each link
% must be feasible and collision-free. A feasible path is one that can be
% realized by the vehicle given the motion and dynamic constraints imposed
% on it. A parking valet system involves low velocities and low
% accelerations. This allows us to safely ignore dynamic constraints
% arising from inertial effects.
%
% Create a |<matlab:doc('pathPlannerRRT') pathPlannerRRT>| object
% configures a path planner using an optimal rapidly exploring random tree
% (RRT*) approach. The RRT family of planning algorithms find a path by
% constructing a tree of connected, collision-free vehicle poses. Poses are
% connected using Dubins or Reeds-Shepp steering, ensuring that the
% generated path is kinematically feasible.

motionPlanner = pathPlannerRRT(costmap, 'MinIterations', 1000, ...
    'ConnectionDistance', 10, 'MinTurningRadius', 20);

%%
% Plan a path from the current pose to the first goal by using the |plan|
% function. The returned |<matlab:doc('driving.Path') driving.Path>| is a
% feasible and collision-free reference path.
goalPose = routePlan{1, 'EndPose'};
refPath = plan(motionPlanner, currentPose, goalPose);

%%
% The reference path consists of a sequence of key poses, expressed as
% $[x,y,\theta]$. Inspect the poses and visualize the planned path.
refPath.KeyPoses

plot(motionPlanner)

%%
% In addition to the planned reference path, notice the red areas on the
% plot. These areas represent areas of the costmap where the origin of the
% vehicle (center of the rear axle) must not cross in order to avoid
% hitting any obstacles. The |<matlab:doc('pathPlannerRRT') pathPlannerRRT>| finds paths that avoid
% obstacles by checking to ensure that vehicle poses generated do not lie
% on these areas.

%% Path Smoothing and Trajectory Generation
% The reference path generated by the path planner is composed either of
% Dubins or Reeds-Shepp segments. The curvature at the junctions of two
% such segments is not continuous and can result in abrupt changes to the
% steering angle. To avoid such unnatural motion and to ensure passenger
% comfort, the path needs to be continuously differentiable and therefore
% smooth [2]. One approach to smoothing a path involves fitting a
% parametric cubic spline. Spline fitting enables you to generate a smooth
% path that a controller can execute.
%
% Create a |<matlab:edit('HelperCubicSplineFit') HelperCubicSplineFit>| object to fit a parametric cubic spline
% that passes through all the waypoints in the reference path. The spline
% approximately matches the starting and ending directions with the
% starting and ending heading angle of the vehicle. The
% |<matlab:edit('HelperCubicSplineFit') HelperCubicSplineFit>| object uses the |<matlab:doc('spline') spline>|
% function for curve fitting.

% Create an object to smooth the path using spline fitting
splineFitter = HelperCubicSplineFit(refPath);

% Fit spline
numPoints = 1000;   % Number of spline points along the path
splineData = fit(splineFitter, numPoints);

% Plot the smoothed path
hold on
hSmoothPath = plot(splineData.x, splineData.y, 'r', 'LineWidth', 2, ...
    'DisplayName', 'Smoothed Path');
hold off

%%
% Next, convert the generated smooth path to a trajectory that can be
% executed using a speed profile. Compute a speed profile for each path
% as a sequence of three phases : accelerating to a set maximum speed,
% maintaining the maximum speed and decelerating to a terminal speed. The
% |<matlab:edit('HelperSpeedProfileGenerator')
% HelperSpeedProfileGenerator>| class generates such a speed profile.
% Currently, only forward motion is supported.
% 
% Create a |<matlab:edit('HelperSpeedProfileGenerator') HelperSpeedProfileGenerator>| object to generate the speed profile based
% on the smoothed path.
speedProfileGenerator = HelperSpeedProfileGenerator(splineData);

%%
% Specify initial, maximum, and terminal speeds so that the vehicle starts
% stationary, accelerates to a speed of 5 meters/second, and comes to a
% stop.
maxSpeed = 5; % in meters/second
speedProfileGenerator.StartSpeed = 0;
speedProfileGenerator.EndSpeed   = 0;
speedProfileGenerator.MaxSpeed   = maxSpeed;

%%
% Generate a speed profile according to the above specifications.
refSpeeds = generate(speedProfileGenerator);

%%
% |refSpeeds| contains reference speeds for each point along the smoothed
% path. Plot the generated speed profile.
plot(speedProfileGenerator.RefPathLength, refSpeeds, 'LineWidth', 2, ...
    'DisplayName', 'Speed Profile')
hold on
xlim([0 speedProfileGenerator.RefPathLength(end)]);
ylim([0 maxSpeed + 2])
line([0;speedProfileGenerator.RefPathLength(end)], [maxSpeed;maxSpeed], ...
    'Color', 'r', 'DisplayName', 'Max Speed')
xlabel('Path Length (m)');
ylabel('Speed (m/s)');
legend
hold off
title('Generated speed profile')

%% Vehicle Control and Simulation
% The reference speeds, together with the smoothed path, comprise a
% feasible trajectory that the vehicle can follow. A feedback controller is
% used to follow this trajectory. The controller selects actuator inputs to
% carry out the planned trajectory. The controller corrects errors in
% tracking the trajectory that arise from tire slippage and other sources
% of noise, such as inaccuracies in localization.
%
% Since this scenario involves slow speeds, you can simplify the controller
% to take into account only a kinematic model. Commonly used controllers
% for this scenario include pure pursuit, front wheel position feedback,
% and rear wheel position feedback. Pure pursuit is not ideally suited to
% tracking parking maneuvers that could include big changes to path
% curvature. Front wheel position feedback does not support driving in
% reverse, a situation that is common in parking lots. Therefore, this
% example uses a controller based on rear wheel position feedback [3]. The
% rear wheel position feedback controller tries to stabilize the rear
% wheels' path. The |<matlab:edit('HelperFeedbackController')
% HelperFeedbackController>| implements a rear wheel controller and
% generates steering and speed commands to control the vehicle.
%
% The feedback controller requires a simulator that can execute the desired
% controller commands using a suitable vehicle model. The
% |<matlab:edit('HelperVehicleSimulator') HelperVehicleSimulator>| class
% simulates such a vehicle using the following kinematic bicycle model:
% 
% $$\dot x_r = v_r*\cos(\theta) $$
%
% $$\dot y_r = v_r*\sin(\theta) $$
%
% $$\dot \theta = \frac{v_r}{l}*\tan(\delta) $$
% 
% In the above equations, $(x_r,y_r,\theta)$ represents the vehicle pose in
% world coordinates. $v_r$, $l$, and $\delta$ represent the rear wheel
% speed, wheel base and steering angle respectively. The position and speed
% of the front wheel can be obtained by:
%
% $$ x_f = x_r + l cos(\theta)$$
%
% $$ y_f = y_r + l sin(\theta)$$
%
% $$ v_f = \frac{v_r} {cos(\delta)}$$
%

% Create the vehicle simulator
vehicleSim = HelperVehicleSimulator(costmap, vehicleDims);

% Set the vehicle pose and configure the simulator to show the trajectory
vehicleSim.setVehiclePose(currentPose);
vehicleSim.showTrajectory(true);

%%
% Create a |<matlab:edit('HelperFeedbackController') HelperFeedbackController>| object to compute speed and steering
% commands to drive the vehicle. The feedback controller minimizes the
% distance between the rear wheel and the reference path. It also minimizes
% the difference between vehicle heading and the tangent at the nearest
% point on the path to the rear wheel.

controller = HelperFeedbackController(splineData, refSpeeds, vehicleDims);

% Set initial steering angle and speed of the controller
controller.SteeringAngle    = 0;
controller.FrontWheelSpeed  = 0;

%%
% Use the |<matlab:edit('HelperFixedRate') HelperFixedRate>| object to
% ensure fixed-rate execution of the feedback controller. Use a control
% rate of 20 Hz.
controlRate = HelperFixedRate(20); % in hertz

%%
% Until the goal is reached, do the following: 
% 
% * Compute steering commands (speed and steering angle) required to track
% the planned trajectory from the feedback controller.
% * Feed the steering commands to the simulator.
% * Record the returned vehicle pose to feed into the controller in the
% next iteration.

while ~controller.ReachGoal
    % Get current pose of the vehicle
    currentPose = getVehiclePose(vehicleSim);
        
    % Compute the controller outputs, which become inputs to the vehicle's
    % control system
    [speed, steeringAngle] = updateControlCommands(controller, currentPose);
    
    % Simulate the vehicle using the controller outputs
    drive(vehicleSim, speed, steeringAngle);
    
    % Wait for fixed-rate execution
    waitfor(controlRate);
end

% Stop the vehicle
stopSpeed           = 0;
stopSteeringAngle   = 0;
drive(vehicleSim, stopSpeed, stopSteeringAngle);

%%
% This completes the first leg of the route plan and demonstrates each step
% of the process. The next sections run the simulator for the entire route
% plan, which takes the vehicle close to the parking spot, and finally
% executes a parking maneuver to place the vehicle into the parking spot.

%% Execute a Complete Plan
% Now combine all the previous steps in the planning process and run the
% simulation for the complete route plan. This process involves
% incorporating the behavioral planner.

% Set the vehicle pose back to the initial starting point
currentPose = [4 12 0]; % [x, y, theta]
vehicleSim.setVehiclePose(currentPose);

% Set current speed and steering angle
currentSpeed = 0; % meters/second
currentSteer = 0; % degrees

while ~reachedDestination(behavioralPlanner)
    
    % Request next maneuver from behavioral layer
    [nextGoal, plannerConfig, speedConfig] = requestManeuver(behavioralPlanner, ...
        currentPose, currentSpeed);
    
    % Configure the motion planner
    configurePlanner(motionPlanner, plannerConfig);
    
    % Plan a reference path using RRT* planner to the next goal pose
    refPath = plan(motionPlanner, currentPose, nextGoal);
    
    % Check if the path is valid. If the planner fails to compute a path,
    % or the path is not collision-free because of updates to the map, the
    % system needs to re-plan. This scenario uses a static map, so the path
    % will always be collision-free.
    isReplanNeeded = ~checkPathValidity(refPath, costmap);
    if isReplanNeeded
        warning('Unable to find a valid path. Attempting to re-plan.')
        
        % Request behavioral planner to re-plan
        replanNeeded(behavioralPlanner);
        continue;
    end
    
    % Smooth the path using splines
    splineFitter.setPoints(refPath);
    splineData = fit(splineFitter, numPoints);
    
    % Generate a trajectory using the speed profile generator
    speedProfileGenerator.setReferencePath(splineData);
    configureSpeedProfileGenerator(speedProfileGenerator, speedConfig);
    refSpeeds = generate(speedProfileGenerator);
    
    % Reset controller
    controller.resetReachGoal();
    
    % Update controller with the reference path and speed
    controller.setProfile(splineData, refSpeeds);
    
    % Update steering angle and speed of the controller
    controller.SteeringAngle    = currentSteer;
    controller.FrontWheelSpeed  = currentSpeed;

    % Execute control loop
    while ~controller.ReachGoal
        % Get current pose of the vehicle
        currentPose = getVehiclePose(vehicleSim);
        
        % Compute commands predicted by the controller
        [speed, steeringAngle] = updateControlCommands(controller, currentPose);
        
        % Simulate the vehicle behavior using the controller outputs
        drive(vehicleSim, speed, steeringAngle); % use bicycle kinematic model to simulate behavior
        
        % Wait for fixed-rate execution
        waitfor(controlRate);
    end
    
    % Get vehicle speed and steering angle reported by the simulator
    currentSpeed = getVehicleSpeed(vehicleSim);
    currentSteer = getVehicleSteer(vehicleSim);
end

%% Parking Maneuver
% Now that the vehicle is near the parking spot, a specialized parking
% maneuver is used to park the vehicle in the final parking spot. The
% |<matlab:edit('helperComputeParkManeuver') helperComputeParkManeuver>| function computes a parking maneuver using
% the following strategy:
%
% # Attempt a simple computation by finding a Dubins path to the parking
% spot destination over a range of possible turning radii.
% # If a Dubins path cannot be found, use the
% |<matlab:doc('pathPlannerRRT') pathPlannerRRT>| to compute a path to the
% parking spot.
%
% Both approaches use a reduced inflation radius in the environment map,
% allowing the vehicle to navigate into a narrow parking spot. This
% maneuver is typically accompanied with ultrasound sensors or laser
% scanners continuously checking for obstacles.

% Define desired pose for the parking spot, returned by the V2X system
parkPose = [36 44 90];

% Compute the required parking maneuver
refPath = helperComputeParkManeuver(costmap, currentPose, parkPose, vehicleDims, maxSteeringAngle);

% Plot the resulting parking maneuver
figure
helperPlotParkingManeuver(costmap, refPath, currentPose, parkPose)

%%

%%
% Once the maneuver is found, repeat the previous process to determine a
% complete plan : smooth the path, generate a speed profile and follow the
% trajectory using the feedback controller.

% Smooth the path using splines
splineFitter.setPoints(refPath);
splineData = fit(splineFitter, numPoints);

% Set up the speed profile generator to stop at the end of the trajectory,
% with a speed limit of 5 mph
speedProfileGenerator.setReferencePath(splineData);
speedProfileGenerator.StartSpeed = currentSpeed;
speedProfileGenerator.MaxSpeed   = 2.2352; % 5 mph
speedProfileGenerator.EndSpeed   = 0;

% Generate speed profile
refSpeeds = generate(speedProfileGenerator);

% Reset controller
controller.resetReachGoal();

% Update controller with the reference path and speed
controller.setProfile(splineData, refSpeeds);

while ~controller.ReachGoal
    % Get current pose of the vehicle
    currentPose = getVehiclePose(vehicleSim);
    
    % Compute commands predicted by the controller
    [speed, steeringAngle] = updateControlCommands(controller, currentPose);
    
    % Simulate the vehicle using the controller outputs
    drive(vehicleSim, speed, steeringAngle);
    
    % Wait for fixed-rate execution
    waitfor(controlRate);
end

% Take snapshot for publishing demo
snapnow

% Delete the simulator
delete(vehicleSim);

%%

%%
% This example showed how to:
% 
% # Plan a feasible path in a semi-structured environment like a parking
% lot using an RRT* path planning algorithm.
% # Smooth the path using splines.
% # Generate a velocity profile and execute it using a feedback controller.

%% References
% [1] Buehler, Martin, Karl Iagnemma, and Sanjiv Singh. _The DARPA Urban
%     Challenge: Autonomous Vehicles in City Traffic_ (1st ed.).
%     Springer Publishing Company, Incorporated, 2009.
%
% [2] Lepetic, Marko, Gregor Klancar, Igor Skrjanc, Drago Matko, Bostjan
%     Potocnik, Time optimal path planning considering acceleration
%     limits. _Robotics and Autonomous Systems_, Volume 45, Issues 3-4,
%     2003, pp. 199-210.
%
% [3] Samson, Claude. Path Following And Time-Varying Feedback
%     Stabilization of a Wheeled Mobile Robot. _Second International
%     Conference on Automation, Robotics and Computer Vision_, Volume
%     13, 1992.

%% Supporting Functions
%%%
% *loadParkingLotMapLayers*
% Load environment map layers for parking lot
function mapLayers = loadParkingLotMapLayers()
%loadParkingLotMapLayers
%   Load occupancy maps corresponding to 3 layers - obstacles, road
%   markings, and used spots.

mapLayers.StationaryObstacles = imread('stationary.bmp');
mapLayers.RoadMarkings        = imread('road_markings.bmp');
mapLayers.ParkedCars          = imread('parked_cars.bmp');
end

%%%
% *plotMapLayers*
% Plot struct containing map layers
function plotMapLayers(mapLayers)
%plotMapLayers
%   Plot the multiple map layers on a figure window.

figure
cellOfMaps = cellfun(@imcomplement, struct2cell(mapLayers), 'UniformOutput', false);
montage( cellOfMaps, 'Size', [1 numel(cellOfMaps)], 'Border', [5 5], 'ThumbnailSize', [300 nan] )
title('Map Layers - stationary obstacles, road markings and parked cars')
end

%%%
% *combineMapLayers*
% Combine map layers into a single costmap
function costmap = combineMapLayers(mapLayers)
%combineMapLayers
%   Combine map layers struct into a single vehicleCostmap.

combinedMap = mapLayers.StationaryObstacles + mapLayers.RoadMarkings + ...
    mapLayers.ParkedCars;
combinedMap = im2single(combinedMap);

res = 0.5; % meters
costmap = vehicleCostmap(combinedMap, 'CellSize', res);
end

%%%
% *configurePlanner*
% Configure path planner with specified settings
function configurePlanner(pathPlanner, config)
%configurePlanner
% Configure the path planner object, pathPlanner, with settings specified
% in struct config.

fieldNames = fields(config);
for n = 1 : numel(fieldNames)
    pathPlanner.(fieldNames{n}) = config.(fieldNames{n});
end
end

%%%
% *configureSpeedProfileGenerator*
% Configure speed profile generator with specified settings
function configureSpeedProfileGenerator(speedProfiler, config)
%configureSpeedProfileGenerator
% Configure speed profiler, speedProfiler, with settings specified in
% struct config.

fieldNames = fields(config);
for n = 1 : numel(fieldNames)
    speedProfiler.(fieldNames{n}) = config.(fieldNames{n});
end
end

%%%
% *helperPlotParkingManeuver*
% Display the generated parking maneuver on a costmap
function helperPlotParkingManeuver(costmap, refPath, currentPose, parkPose)
%helperPlotParkingManeuver
% Plot the generated parking maneuver on a costmap.

% Plot the costmap, without inflated areas
plot(costmap, 'Inflation', 'off')

% Plot reference parking maneuver on the costmap
hold on
plot(refPath, 'DisplayName', 'Parking Maneuver')

title('Parking Maneuver')

% Zoom into parking maneuver by setting axes limits
lo = min([currentPose(1:2); parkPose(1:2)]);
hi = max([currentPose(1:2); parkPose(1:2)]);

buffer = 4; % meters

xlim([lo(1)-buffer hi(1)+buffer])
ylim([lo(2)-buffer hi(2)+buffer])
end