%% Automated Parking Valet in Simulink 
% This example shows how to construct an automated parking valet system in
% Simulink(R) with Automated Driving System Toolbox(TM). It closely follows the 
% <matlab:web(fullfile(docroot,'driving','examples','automated-parking-valet.html')) Automated Parking Valet> 
% MATLAB(R) example.
%
% Copyright 2017-2018 The MathWorks, Inc.

%% Introduction
% Automatically parking a car that is left in front of a parking lot is a
% challenging problem. The vehicle's automated systems are expected to take
% over and steer the vehicle to an available parking spot. 
% This example focuses on planning a feasible path through the environment,
% generating a trajectory from this path, and using a feasible controller to 
% execute the trajectory. Map creation and dynamic obstacle avoidance are 
% excluded from this example. In addition, the specialized parking maneuver in the 
% <matlab:web(fullfile(docroot,'driving','examples','automated-parking-valet.html')) Automated Parking Valet> 
% MATLAB(R) example that parks the vehicle in the final parking spot is not
% implemented here.

%% 
% Before simulation, the |<matlab:edit('helperSLCreateCostmap') helperSLCreateCostmap>|
% function, called within the PreLoadFcn callback function of the model,
% creates a static map of the parking lot. The map contains information 
% about stationary obstacles, road markings, and parked cars and is 
% represented as a |<matlab:doc('vehicleCostmap') vehicleCostmap>| object.
%
% To use the |<matlab:doc('vehicleCostmap') vehicleCostmap>| object in 
% Simulink(R), the |<matlab:edit('helperSLCreateUtilityStruct') helperSLCreateUtilityStruct>|
% function converts the |<matlab:doc('vehicleCostmap') vehicleCostmap>|
% into a struct array in the block's <matlab:web(fullfile(docroot,'simulink/ug/initialize-mask.html')) Mask Initialization>.
costmap = helperSLCreateCostmap();
figure,
plot(costmap, 'Inflation', 'off')
legend off

%% 
% The global route plan is described as a sequence of lane segments to 
% traverse to reach a parking spot. Before simulation, the 
% <matlab:web(fullfile(docroot,'simulink/ug/model-callbacks.html#')) |PreLoadFcn|>
% callback function of the model loads a route plan, which is stored as a 
% table. The table specifies the start and end poses of the segment, as 
% well as properties of the segment, such as the speed limit.
data      = load('routePlan.mat');
routePlan = data.routePlan %#ok<NOPTS>

hold on
currentPose = [4 12 0]; % [x, y, theta]
vehicleDims = costmap.VehicleDimensions;
helperPlotVehicle(currentPose, vehicleDims, 'DisplayName', 'Current Pose')
legend('Location', 'northwest');

for n = 1 : height(routePlan)
    % Extract the goal waypoint
    vehiclePose = routePlan{n, 'EndPose'};
    
    % Plot the pose
    legendEntry = sprintf('Goal %i', n);
    helperPlotVehicle(vehiclePose, vehicleDims, 'DisplayName', legendEntry);
end
hold off

%%
% The inputs and outputs of many blocks in this example are
% <matlab:web(fullfile(docroot,'simulink','slref','simulink.bus-class.html'))
% Simulink Buses>. In the <matlab:web(fullfile(docroot,'simulink/ug/model-callbacks.html#')) |PreLoadFcn|> 
% callback function of the model, the |<matlab:edit('helperSLCreateUtilityBus') helperSLCreateUtilityBus>|
% function creates these buses.

%% 
open_system('AutomatedParkingValet')
set_param('AutomatedParkingValet','SimulationCommand','Update')
%%
% Planning is a hierarchical process, with each successive layer responsible 
% for a more fine-grained task. The behavior layer [1] sits at the top of 
% this stack. The *Behavior Planner* block triggers a sequence of navigation  
% tasks based on the global route plan by providing an intermediate goal and 
% configuration for the *Path Planner* and *Trajectory Generator* blocks. 
% Each path segment is navigated using these steps:
%
% # *Motion Planner*: Plan a feasible path through the environment map
% using the optimal rapidly-exploring random tree (RRT*) algorithm
% (|<matlab:doc('pathPlannerRRT') pathPlannerRRT>|).
% # *Trajectory Generator*: Smooth the reference path by fitting splines [2]  
% and convert the smoothed path into a trajectory by generating a speed profile. 
% # *Vehicle Controller*: Use a feedback controller [3] based on rear-wheel
% position to execute the trajectory. The feedback controller minimizes the
% distance between the rear wheel and the reference path. It also minimizes
% the difference between the vehicle heading and the tangent at the nearest
% point on the path to the rear wheel.
% # Check if the vehicle has reached the final pose of the segment.
%
% To demonstrate the performance, the feedback controller is applied to the 
% *Vehicle Model* block, which contains a <matlab:web(fullfile(docroot,'vdynblks','ref','vehiclebody3dof.html')) Vehicle Body 3DOF Single Track>
% block shared between Automated Driving System Toolbox(TM) and Vehicle 
% Dynamics Blockset(TM). Compared with the kinematic bicycle model used in the
% <matlab:web(fullfile(docroot,'driving','examples','automated-parking-valet.html')) Automated Parking Valet>
% MATLAB(R) example, this model is more accurate because it considers the
% inertial effects, such as tire force and aerodynamic drag.
%%

%% Simulation Results
% The *Visualization* block shows how the vehicle tracks the reference
% path. It also displays vehicle speed and steering command in a scope. 
% The following images are the simulation results for this example:
%%
sim('AutomatedParkingValet')
snapnow
close 'Automated Parking Valet'
%%
% Simulation stops at about 34 seconds when the vehicle reaches the destination.
% The final maneuver required to park the vehicle in the spot is omitted 
% from this example and can be found in the 
% <matlab:web(fullfile(docroot,'driving','examples','automated-parking-valet.html')) Automated Parking Valet> 
% MATLAB(R) example.
%%
scope = 'AutomatedParkingValet/Visualization/Commands';
open_system(scope);
snapnow
close_system(scope);
%%
bdclose all;

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