clear

load testMotionPlanner

% Initialize pathPlannerRRT object
motionPlanner = myPathPlannerRRT(costmap, ...
    'MinIterations',     1000, ...
    'ConnectionDistance',3, ...
    'ConnectionMethod', 'Customize');

nextGoalPose  = nextGoal;
vehiclePose   = startPose;
fprintf('My RRT init\n');
[path, controlSequence] = plan(motionPlanner, vehiclePose, nextGoalPose);

plot(motionPlanner,'Tree','on')
