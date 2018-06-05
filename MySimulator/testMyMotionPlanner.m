clear

load testMotionPlanner

% Initialize pathPlannerRRT object
motionPlanner = myPathPlannerRRT(Costmap, ...
    'MinIterations',     1000, ...
    'ConnectionDistance',3, ...
    'ConnectionMethod', 'Customize', ...
    'GoalBias', 0.1, ...
    'MaxIterations', 3000);

nextGoalPose = [40 5 pi/5 0 0 0];
vehiclePose   = [2.5 0 0 0 0 0];
fprintf('My RRT init\n');
[path, controlSequence, treeDiagraph] = plan(motionPlanner, vehicle Pose, nextGoalPose);

plot(motionPlanner,'Tree','on')
