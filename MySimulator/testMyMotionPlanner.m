clear

load testMotionPlanner

% Initialize pathPlannerRRT object
motionPlanner = myPathPlannerRRT(Costmap, ...
    'MinIterations',     1000, ...
    'ConnectionDistance',3, ...
    'ConnectionMethod', 'Customize', ...
    'GoalBias', 0.1, ...
    'MaxIterations', 10000);

nextGoalPose = [40 7 0 0 0 0];
vehiclePose   = [1 0 0 0 0 0];
fprintf('My RRT init\n');
[path, controlSequence, treeDiagraph] = plan(motionPlanner, vehiclePose, nextGoalPose);

plot(motionPlanner,'Tree','on')
