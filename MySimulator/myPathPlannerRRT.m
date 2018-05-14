%pathPlannerRRT Configure an RRT* path planner.
%   pathPlannerRRT configures a path planner based on the Optimal Rapidly
%   Exploring Random Tree (RRT*) algorithm. An RRT planner explores the
%   environment around the vehicle by constructing a tree of random
%   collision-free poses from a specified start pose towards a desired goal
%   region. Once the pathPlannerRRT is configured, use the plan method to
%   plan a path.
%
%   planner = pathPlannerRRT(costmap) returns a pathPlannerRRT object for
%   planning. costmap is a vehicleCostmap object specifying the environment
%   around the vehicle as a costmap.
%
%   planner = pathPlannerRRT(...,Name,Value) specifies additional
%   name-value pair arguments as described below:
%
%
%   'GoalTolerance'         Acceptable tolerance around goal pose specified
%                           as [xTol, yTol, thetaTol]. xTol and yTol are
%                           specified in world units. thetaTol is specified
%                           in degrees.
%
%                           Default: [0.5, 0.5, 5]
%
%   'GoalBias'              Probability of selecting the goal pose at each
%                           iteration as opposed to a random pose. Larger
%                           values accelerate reaching the goal at the risk
%                           of failing to circumnavigate obstacles.
%
%                           Default: 0.1
%
%   'ConnectionMethod'      Method used to calculate connection between
%                           consecutive poses, specified as one of the
%                           strings 'Dubins' or 'Reeds-Shepp'. These
%                           methods compute a kinematically feasible path
%                           between poses as a sequence of primitive
%                           motions.
%
%       -------------------------------------------------------------------
%       Connection Method  | Description
%       -------------------|-----------------------------------------------
%       'Dubins'           | A sequence of 3 primitive motions, each of
%                          | of which can be:
%                          | - straight (forward)
%                          | - left turn at maximum steer (forward)
%                          | - right turn at maximum steer (forward)
%       -------------------|-----------------------------------------------
%       'Reeds-Shepp'      | A sequence of up to 5 primitive motions, each
%                          | of which can be:
%                          | - straight (forward and reverse)
%                          | - left turn at maximum steer (forward and
%                          |   reverse)
%                          | - right turn at maximum steer (forward and
%                          |   reverse)
%       -------------------------------------------------------------------
%
%                           Default: 'Dubins'
%
%   'ConnectionDistance'    Maximum distance between two connected poses.
%                           Distance is computed along the path. Larger
%                           values result in longer path segments between
%                           poses.
%
%                           Default: 5
%
%   'MinTurningRadius'      Minimum turning radius of vehicle specified in
%                           world units. This corresponds to the radius of
%                           the turning circle at maximum steer. A larger
%                           value limits the maximum steering angle used by
%                           the planner.
%
%                           Default: 4 (this is computed using a wheelbase
%                           of 2.8 meters and a maximum steering angle of
%                           35 degrees.)
%
%   'MinIterations'         Minimum number of planner iterations. This is
%                           the minimum number of iterations of exploration
%                           the planner undertakes before returning a path.
%                           
%
%                           Default: 100
%
%   'MaxIterations'         Maximum number of planner iterations. This is
%                           the maximum number of iterations of exploration
%                           the planner undertakes in search of a path. If
%                           no path is found at the end of MaxIterations
%                           number of iterations, the plan method will
%                           return a path with no poses. 
%
%                           Default: 1e4
%
%   'ApproximateSearch'     Flag indicating whether approximate nearest
%                           neighbor search is used. Setting this to true
%                           uses a faster, but approximate search
%                           algorithm. Setting this to false uses an exact,
%                           but slower search algorithm at the cost of
%                           increased computation time.
%
%                           Default: true
%
%
%   pathPlannerRRT properties:
%   Costmap                 - Costmap used for collision checking (read-only)
%   GoalTolerance           - Acceptable tolerance around goal pose
%   GoalBias                - Probability of selecting goal pose
%   ConnectionMethod        - Method used to connect poses
%   ConnectionDistance      - Maximum distance between two connected poses
%   MinTurningRadius        - Minimum turning radius of vehicle
%   MinIterations           - Minimum number of iterations of exploration
%   MaxIterations           - Maximum number of iterations of exploration
%   ApproximateSearch       - Flag indicating use of approximate search
%
%   pathPlannerRRT methods:
%   plan                    - plan a path between start and goal poses
%   plot                    - plot a planned path on a map.
%
%
%   Notes
%   -----
%   - The pathPlannerRRT checks for collisions by verifying that the
%     center of the vehicle is not within an inflated grid cell in the 
%     vehicleCostmap. If the InflationRadius of the vehicleCostmap is set
%     to a value less than the radius of the smallest circle completely
%     encircling the vehicle (default), the planned path may contain
%     collisions.
%   - Updating any of the properties of the planner will clear the planned
%     path from the pathPlannerRRT. Subsequent calls to plot will only show
%     the map.
%   - In order to improve performance, the pathPlannerRRT uses an 
%     approximate nearest neighbor search that only searches through
%     sqrt(N) nodes, where N is the number of nodes to search. This can
%     result in less optimal paths. Set 'ApproximateSearch' to false to use
%     exact nearest neighbor search.
%   - 'Dubins' and 'Reeds-Shepp' connection methods are kinematically
%     feasible and ignore inertial effects. This makes the planner suitable
%     for low velocity environments where the inertial effects of wheel
%     forces are small.
%
%
%   Example: Plan a path towards parking spot
%   -----------------------------------------
%   % Load a costmap for a parking lot
%   data = load('parkingLotCostmap.mat');
%   parkMap = data.parkingLotCostmap;
%
%   % Define a start and goal pose as [x,y,theta]
%   startPose = [4, 4, 90];   % [meters, meters, degrees]
%   goalPose  = [30, 13, 0];
%
%   % Create an RRT planner
%   planner = pathPlannerRRT(parkMap);
%
%   % Plan a route from start to goal pose
%   refPath = plan(planner, startPose, goalPose);
%
%   % Plot planned route
%   plot(planner)
%
%   % Compute all poses along path (as seen in plot)
%   allPoses = connectingPoses(refPath);
%
%   See also vehicleCostmap, checkPathValidity.

% Copyright 2017-2018 The MathWorks, Inc.

%   References
%   ----------
%   [1] S. Karaman, E. Frazzoli, "Optimal kinodynamic motion planning using
%       incremental sampling-based methods," IEEE Conference on Decision
%       and Control (CDC), pp. 7681-7687, 2010.
%
%   [2] A. Shkel, V. Lumelsky, "Classification of the Dubins set," Robotics
%       and Autonomous Systems, vol. 34, issue 4, pp. 179-202, 2001.
%
%   [3] J.A. Reeds, L.A. Shepp, "Optimal paths for a car that goes both
%       forwards and backwards," Pacific Journal of Mathematics,
%       145(2):3670393, 1990.

classdef myPathPlannerRRT < driving.planning.PathPlanner
    
    properties (SetAccess = private)
        %Costmap Costmap used for collision checking.
        %   Costmap is an object of type vehicleCostmap and represents the
        %   map around the vehicle to be used for collision checking during
        %   planning.
        Costmap
    end
    
    properties
        %GoalTolerance Tolerance around goal pose.
        %   Acceptable tolerance around goal pose specified as a 3-element
        %   vector [xTol, yTol, thetaTol]. xTol and yTol are specified in
        %   world units. thetaTol is specified in degrees. A goal is
        %   reached if the planner finds a path to a pose within [+/-xTol,
        %   +/-yTol, +/-thetaTol] of the goalPose.
        %
        %   Default: [0.5, 0.5, 5]
        GoalTolerance = [0.5, 0.5, 5, 2, 2, 3];
        
        %GoalBias Probability of selecting goal pose.
        %   Probability with which to select goal pose for tree expansion.
        %   Increasing this value biases exploration towards goal at the
        %   cost of possibly getting stuck at an obstacle.
        %
        %   Default: 0.1
        GoalBias = 0.1;
        
        %ConnectionMethod Method used to connect poses.
        %   Method used to connect two nodes (poses) specified as one of
        %   the strings 'Dubins' or 'Reeds-Shepp'.
        %
        %   ---------------------------------------------------------------
        %   Connection Method  | Description
        %   -------------------|-------------------------------------------
        %                      |
        %   'Dubins'           | Connects two poses by computing a
        %                      | kinematically feasible path between them
        %                      | as a sequence of 3 primitive motions, each
        %                      | of which can be:
        %                      | - Straight (forward)
        %                      | - Left turn at maximum steer (forward)
        %                      | - Right turn at maximum steer (forward)
        %                      |
        %   -------------------|-------------------------------------------
        %   'Reeds-Shepp'      | Connects two poses by computing a
        %                      | kinematically feasible path between them
        %                      | as a sequence of up to 5 primitive
        %                      | motions, each of which can be:
        %                      | - Straight (forward and reverse)
        %                      | - Left turn at maximum steer (forward and
        %                      |   reverse)
        %                      | - Right turn at maximum steer (forward and
        %                      |   reverse)
        %                      |
        %   ---------------------------------------------------------------
        %
        %   Default: 'Dubins'
        ConnectionMethod = 'Dubins';
        
        %ConnectionDistance Maximum distance between two connected nodes.
        %   Maximum distance between two connected nodes. Distance is
        %   computed along the path.
        %
        %   Default: 5
        ConnectionDistance = 5;
        
        %MinTurningRadius Minimum turning radius of vehicle.
        %   Minimum turning radius of vehicle specified in world units.
        %   This corresponds to the radius of the turning circle at maximum
        %   steer. A larger value limits the maximum steering angle used by
        %   the planner.
        %
        %   Default: 4
        MinTurningRadius = 4;
        
        %MinIterations Minimum number of iterations of exploration.
        %   Minimum number of iterations of exploration the planner
        %   undertakes before returning a path.
        %
        %   Default: 100
        MinIterations = 100
        
        %MaxIterations Maximum number of iterations of exploration.
        %   Maximum number of iterations of exploration the planner
        %   undertakes in search of a path. If no path is found at the end
        %   of MaxIterations number of iterations, the plan method will
        %   return a path with no poses.
        %
        %   Default: 1e4
        MaxIterations = 1e4;
        
        %ApproximateSearch Flag indicating use of approximate search.
        %   Flag indicating whether approximate nearest neighbor search is
        %   used. True or false.
        %
        %   Default: true
        ApproximateSearch = true;
    end
    
    properties (Access = protected)
        Name    = mfilename;
        
        PoseDim = 6;
    end
    
    properties (Access = private, Hidden)
        Version = ver('driving');
        
        Sampler
        
        ConnectionMechanism
        
        StartPose
        
        GoalPose
        
        GoalNodes       = [];
        
        BestGoalNode    = [];
        
        BestCost     = inf;
        
        Path
    end
    
    properties (Access = ?tpathPlannerRRT, Hidden)
        Tree
    end
    
    properties (Access = private, Dependent, Transient)
        %NumConnectionSteps
        %   Number of steps along connection.
        NumConnectionSteps
    end
    
    %----------------------------------------------------------------------
    % Interface
    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        function this = myPathPlannerRRT(varargin)
            
            plannerDims = 6;
            this = this@driving.planning.PathPlanner(mfilename, plannerDims);
            
            inputs = this.parseInputs(varargin{:});
            
            this.Costmap             = copy(inputs.Costmap);
            this.GoalTolerance       = inputs.GoalTolerance;
            this.GoalBias            = inputs.GoalBias;
            this.ConnectionMethod    = inputs.ConnectionMethod;
            this.ConnectionDistance  = inputs.ConnectionDistance;
            this.MinTurningRadius    = inputs.MinTurningRadius;
            this.MinIterations       = inputs.MinIterations;
            this.MaxIterations       = inputs.MaxIterations;
            this.ApproximateSearch   = inputs.ApproximateSearch;
            
            this.configure();
        end
        
        %------------------------------------------------------------------
        function varargout = plan(this, startPose, goalPose)
            %plan Plan a path using RRT*.
            %   refPath = plan(planner, startPose, goalPose) plans a path
            %   from startPose towards goalPose using RRT*.
            %
            %   [refPath,tree] = plan(...) additionally returns the
            %   exploration tree.
            %
            %
            %   Inputs
            %   ------
            %   startPose   Initial vehicle pose specified as [x,y,theta].
            %               Specify x, y in world units and theta in
            %               degrees.
            %
            %
            %   goalPose    Goal vehicle pose specified as [x,y,theta].   
            %               Specify x, y in world units and theta in
            %               degrees.
            %
            %   Outputs
            %   -------
            %   refPath     Planned vehicle path returned as a
            %               driving.Path object containing reference poses
            %               along the found path. If planning is not
            %               successful, the path has no reference poses.
            %               Use the checkPathValidity function to check if
            %               the path is still valid in case the costmap is
            %               updated.
            %
            %   tree        Explored tree returned as a digraph object
            %               with nodes representing explored vehicle poses
            %               and edges representing distance between
            %               connected nodes.
            %
            %
            %   Example : Plan a path through a maze
            %   ------------------------------------
            %   % Load a costmap of a maze
            %   data = load('mazeMap.mat');
            %   mazeMap = data.mazeMap;
            %
            %   % Define a start and a goal
            %   startPose = [3 2 0];
            %   goalPose  = [45 35 135];
            %
            %   % Create an RRT planner
            %   planner = pathPlannerRRT(mazeMap, 'MinTurningRadius', 1);
            %
            %   % Plan a path to the goal
            %   refPath = plan(planner, startPose, goalPose)
            %
            %   % Check if a path was found.
            %   pathFound = ~isempty(refPath.KeyPoses)
            %
            %   % Plot the planner
            %   figure, plot(planner)
            %
            %
            %   See also checkPathValidity, digraph.
            
            nargoutchk(0,2);
            
            this.validatePoses(startPose, goalPose);
            
            startPose(3) = driving.planning.angleUtilities.convertAndWrapTo2Pi(startPose(3));
            goalPose(3)  = driving.planning.angleUtilities.convertAndWrapTo2Pi(goalPose(3));
            
            this.StartPose = startPose;
            this.GoalPose  = goalPose;
            
            this.planPath();
            
            varargout{1} = this.Path;
            
            if nargout>1
                varargout{2} = this.Tree.toDigraph();
            end
        end
        
        %------------------------------------------------------------------
        function plot(this, varargin)
            %plot Plot a planned path.
            %
            %   plot(planner) plots the path planned by the pathPlannerRRT
            %   planner.
            %
            %   plot(...,Name,Value) specifies additional name-value pair
            %   arguments as described below:
            %
            %   'Parent'    Handle to an axes on which to display the path.
            %
            %   'Tree'      A string 'on' or 'off' to turn on or off the
            %               display of poses explored by RRT*.
            %
            %               Default: 'off'
            %
            %   'Vehicle'   A string 'on' or 'off' to turn on or off the
            %               display of the vehicle along the found path.
            %
            %               Default: 'on'
            %
            %   Notes
            %   -----
            %   If a path has not been found, a path has not been planned
            %   or properties of the planner have changed, only the costmap
            %   will be displayed.
            %
            %   See also vehicleCostmap.
            
            params = this.parsePlotInputs(varargin{:});
            
            % Prepare axes handle according to NextPlot
            hAx = newplot(params.Parent);
            
            % Display costmap
            plot(this.Costmap, 'Parent', hAx, 'Inflation', 'on');
            
            
            % Record current hold state
            if ishold(hAx)
                oldState = 'on';
            else
                oldState = 'off';
            end
            
            % Turn hold on
            hold(hAx, 'on')
            
            % Restore hold state on completion
            restoreHoldState = onCleanup(@()hold(hAx, oldState));
            
            % Change resolution factor for faster processing of plot
            resolutionFactor = 5;
            this.ConnectionMechanism.NumSteps = ceil(this.NumConnectionSteps / resolutionFactor);
            
            % Restore resolution factor on completion
            restoreResolution = onCleanup(@()restoreNumSteps(this));
            
            function restoreNumSteps(this)
                this.ConnectionMechanism.NumSteps = this.NumConnectionSteps;
            end
            
            % Define colors
            colors.Tree  = [0.9290    0.6940    0.1250];
            colors.Path  = [     0    0.4470    0.7410];
            colors.Goal  = [0.8500    0.3250    0.0980];%[0.4660    0.6740    0.1880];%[0.6350    0.0780    0.1840];
            
            
            % Plot exploration tree
            if strcmp(params.Tree, 'on')
                plotTree(this, hAx, colors.Tree);
            end
            
            % Plot planned path
            plotPath(this, hAx, colors.Path, params.Vehicle);
            
            % Plot start and goal poses
            plotEndPoints(this, hAx, colors.Goal);
            
            % Turn on legend if there is a path
            if ~isempty(this.GoalPose)
                legend;
            end
        end
    end
    
    
    %----------------------------------------------------------------------
    % Accessors
    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        function set.Costmap(this, costmap)
            
            this.validateCostmap(costmap);
            this.Costmap = costmap;
        end
        
        %------------------------------------------------------------------
        function set.GoalTolerance(this, gtol)
            
            this.validateGoalTolerance(gtol);
            this.GoalTolerance = gtol;
        end
        
        %------------------------------------------------------------------
        function set.GoalBias(this, gbias)
            
            validateattributes(gbias, {'single','double'}, ...
                {'scalar','>=', 0, '<', 1}, mfilename, 'GoalBias');
            this.GoalBias = gbias;
        end
        
        %------------------------------------------------------------------
        function set.ConnectionMethod(this, cmethod)
            
            this.ConnectionMethod = ...
                driving.planning.validation.checkConnectionMethod( ...
                cmethod, mfilename);
        end
        
        %------------------------------------------------------------------
        function set.ConnectionDistance(this, cdist)
            
            this.validateConnectionDistance(cdist);
            this.ConnectionDistance = double(cdist);
        end
        
        %------------------------------------------------------------------
        function set.MinTurningRadius(this, radius)
            
            validateattributes(radius, {'single','double'}, ...
                {'scalar', 'positive', 'finite'}, mfilename, ...
                'MinTurningRadius');
            this.MinTurningRadius = double(radius);
        end
        
        %------------------------------------------------------------------
        function set.MinIterations(this, minIter)
            
            validateattributes(minIter, {'numeric'}, ...
                {'scalar', 'positive', 'finite', 'integer'}, mfilename, ...
                'MinIterations');
            
            this.checkIterations(minIter, this.MaxIterations); %#ok<MCSUP>
            
            this.MinIterations = double(minIter);
        end
        
        %------------------------------------------------------------------
        function set.MaxIterations(this, maxIter)
            
            validateattributes(maxIter, {'numeric'}, ...
                {'scalar', 'positive', 'finite', 'integer'}, mfilename, ...
                'MaxIterations');
            
            this.checkIterations(this.MinIterations, maxIter); %#ok<MCSUP>
            
            this.MaxIterations = double(maxIter);
        end
        
        %------------------------------------------------------------------
        function set.ApproximateSearch(this, approxSearch)
            
            validateattributes(approxSearch, {'numeric','logical'}, ...
                {'scalar', 'binary'}, mfilename, 'ApproximateSearch');
            this.ApproximateSearch = logical(approxSearch);
        end
        
        %------------------------------------------------------------------
        function numSteps = get.NumConnectionSteps(this)
            
            % Number of connection steps is used to control the resolution
            % at which collision checking is performed against the map. We
            % use a resolution factor of 5, i.e. an interpolation step at a
            % distance of 0.2*cellSize.
            resolutionFactor = 5;
            
            if isfinite(this.ConnectionDistance)
                numSteps = resolutionFactor * max(3, ...
                    ceil(this.ConnectionDistance/this.Costmap.CellSize) );
            else
                numSteps = resolutionFactor * norm( size(this.Costmap.Costmap) );
            end
        end
    end
    
    
    %----------------------------------------------------------------------
    % Save/Load
    %----------------------------------------------------------------------
    methods (Static, Hidden)
        %------------------------------------------------------------------
        function this = loadobj(that)
            
            this = pathPlannerRRT(that.Costmap, ...
                'GoalTolerance',        that.GoalTolerance, ...
                'GoalBias',             that.GoalBias, ...
                'ConnectionMethod',     that.ConnectionMethod, ...
                'ConnectionDistance',   that.ConnectionDistance, ...
                'MinTurningRadius',     that.MinTurningRadius, ...
                'MinIterations',        that.MinIterations, ...
                'MaxIterations',        that.MaxIterations, ...
                'ApproximateSearch',    that.ApproximateSearch);
        end
    end
    
    methods (Hidden)
        %------------------------------------------------------------------
        function that = saveobj(this)
            
            that.Costmap            = this.Costmap;        % this is an object
            that.GoalTolerance      = this.GoalTolerance;
            that.GoalBias           = this.GoalBias;
            that.ConnectionMethod   = this.ConnectionMethod;
            that.ConnectionDistance = this.ConnectionDistance;
            that.MinTurningRadius   = this.MinTurningRadius;
            that.MinIterations      = this.MinIterations;
            that.MaxIterations      = this.MaxIterations;
            that.ApproximateSearch  = this.ApproximateSearch;
            that.Version            = this.Version;
        end
    end
    
    
    %----------------------------------------------------------------------
    % Planning
    %----------------------------------------------------------------------
    methods (Access = private)
        %------------------------------------------------------------------
        function configure(this)
            
            % Instantiate pose sampler
            precision = class(this.GoalTolerance);
            this.Sampler = myUniformPoseSampler(...
                this.Costmap.MapExtent, precision);
            
            % Instantiate RRT tree
            this.Tree = myRRTTree(this.PoseDim, precision);
        end
        
        %------------------------------------------------------------------
        function configureConnectionMechanism(this)
            
            switch this.ConnectionMethod
                case 'Dubins'
                    this.ConnectionMechanism = ...
                        driving.planning.DubinsConnectionMechanism( ...
                        this.MinTurningRadius, this.NumConnectionSteps, ...
                        this.ConnectionDistance);
                case 'Reeds-Shepp'
                    this.ConnectionMechanism = ...
                        driving.planning.ReedsSheppConnectionMechanism( ...
                        this.MinTurningRadius, this.NumConnectionSteps, ...
                        this.ConnectionDistance);
                case 'Customize'
                    this.ConnectionMechanism = ...
                        myConnectionMechanism( ...
                        this.MinTurningRadius, this.NumConnectionSteps, ...
                        this.ConnectionDistance);
            end
            
            this.Tree.configureConnectionMechanism(this.ConnectionMechanism);
        end
        
        %------------------------------------------------------------------
        function planPath(this)
            
            startPose = this.StartPose;
            
            % Reset internal tree and sampler
            this.Tree.reset();
            this.Sampler.reset();
            
            % Initialize path to empty
            this.Path = myPath.create();
            this.GoalNodes      = [];
            this.BestGoalNode   = [];
            this.BestCost       = inf;
            
            % Set up nearest neighbor search algorithm
            this.Tree.configureNeighborSearcher(this.ApproximateSearch);
            
            % Set up connection mechanism
            this.configureConnectionMechanism();
            
            % Set up sampling
            this.Sampler.configureCollisionChecker(this.Costmap);
            
            % Seed the root of the tree.
            this.Tree.addNode(startPose);
            
            maxIter = this.MaxIterations;
            
            for n = 1 : maxIter
                
                % Sample collision-free pose with goal biasing
                randPose = this.sampleCollisionFreeWithGoalBiasing();
                
                % Find nearest neighbor in tree
                [nearestPose, nearestId] = this.Tree.nearest(randPose);
                
                % Interpolate towards random pose
                newPose = this.interpolate(nearestPose, randPose);
                
                if isempty(newPose)
                    continue;
                end
                
                % Find near neighbors to new pose
                [nearPoses, nearIds] = this.Tree.near(newPose);
                
                % Add new pose to tree
                newId = this.Tree.addNode(newPose);
                
                % Find minimum cost path to new pose.
                this.findMinCostPath(nearPoses, nearIds, ...
                    nearestPose, nearestId, newPose, newId);
                
                % Rewire tree
%                 this.rewireTree(nearPoses, nearIds, newPose, newId);
                
                if this.inGoalRegion(newPose)
                    
                    % Add Id to goals
                    this.GoalNodes(end+1) = newId;
                    
                    planCost = this.Tree.costTo(newId);
                    
                    % Update best cost
                    if planCost < this.BestCost
                        this.BestGoalNode   = newId;
                        this.BestCost       = planCost;
                    end
                    
                    if this.terminateExploration(n)
                        break;
                    end
                end
            end
            
            if ~isempty(this.GoalNodes)
                
                % Find path through tree
                [path,cost] = this.Tree.shortestPathFromRoot(...
                    this.BestGoalNode, this.GoalNodes);
                
                pathPoses = this.Tree.Nodes(path,:);
                pathPoses(:,3) = rad2deg(pathPoses(:,3));
            else
                % No path was found
                pathPoses = zeros(0,3);
                cost      = 0;
            end
            
            % Construct driving.Path object to hold path
            this.Path = driving.Path.create(pathPoses, ...
                this.ConnectionMechanism, cost);
        end
    end
    
    methods (Access = private)
        %------------------------------------------------------------------
        function pose = sampleCollisionFreeWithGoalBiasing(this)
            
            if this.Sampler.sampleGoalBias() > this.GoalBias
                pose = this.Sampler.sampleCollisionFree();
            else
                pose = this.GoalPose;
            end
        end
        
        %------------------------------------------------------------------
        function pose = interpolate(this, from, towards)
            
            % Interpolate along connection mechanism
            posesInterp = this.ConnectionMechanism.interpolate(from, towards);
            
            throwError = false;
            free = checkFreeVehiclePoses(this.Costmap, posesInterp, throwError);
            
            % If any pose is outside the map or in collision, bail out.
            if any(~free)
                pose = [];
            else
                pose = posesInterp(end,:);
                pose = [pose, towards(4:6)];
            end
        end
        
        %------------------------------------------------------------------
        function findMinCostPath(this, nearPoses, nearIds, nearestPose, ...
                nearestId, newPose, newId)
            %findMinCostPath find the minimum cost path to newPose from
            %near neighbors nearPoses, and add this edge to the tree.
            
            % Use local variables to improve performance
            treeLocal = this.Tree;
            
            minCost = treeLocal.costTo(nearestId) + ...
                this.ConnectionMechanism.distance(nearestPose, newPose);
            
            minCostId = nearestId;
            numNear   = numel(nearIds);
            maxDist   = this.ConnectionDistance;
            
            distances = this.ConnectionMechanism.distance(nearPoses, newPose);
            
            % For each near neighbor
            for n = 1 : numNear
                nearPose = nearPoses(n,:);
                nearId   = nearIds(n);
                nearCost = treeLocal.costTo(nearId) + distances(n);
                
                % Note: The call to interpolate in this conditional is last
                % to be evaluated. We take advantage of short-circuiting
                % behavior of && to implement delayed collision-checking.
                
                if nearCost < minCost ...                                   % Cost is lower
                        && distances(n)<=maxDist ...                        % Distance is within connection distance
                        && ~isempty(this.interpolate(nearPose, newPose))    % Path is collision-free and in bounds
                    
                    minCost = nearCost;
                    minCostId = nearId;
                end
            end
            
            % Add edge for minimum cost path.
            treeLocal.addEdge(minCostId, newId);
        end
        
        %------------------------------------------------------------------
        function rewireTree(this, nearPoses, nearIds, newPose, newId)
            %rewireTree rewire tree so that cost to nearPose is more
            %optimal after newPose was added to the tree.
            
            % Use local variables to improve performance
            treeLocal = this.Tree;
            cellSize  = this.Costmap.CellSize;
            thetaTol  = deg2rad(this.GoalTolerance(3));
            
            newCost = treeLocal.costTo(newId);
            numNear = numel(nearIds);
            maxDist = this.ConnectionDistance;
            
            forwardDistances = this.ConnectionMechanism.distance(nearPoses, newPose);
            reverseDistances = this.ConnectionMechanism.distance(newPose, nearPoses);
            
            % For each neighobr
            for n = 1 : numNear
                nearPose = nearPoses(n,:);
                nearId   = nearIds(n);
                nearCost = treeLocal.costTo(nearId);
                
                newNearCost = newCost + reverseDistances(n);
                
                if newNearCost < nearCost ...                                   % Cost is lower
                        && forwardDistances(n) <= maxDist                       % Distance is within connection distance
                    
                    finalPose = this.interpolate(newPose, nearPose);
                    
                    reachedNearPose = ~isempty(finalPose) ...
                        && norm(finalPose(1:2)-nearPose(1:2)) ...
                        <= cellSize ...
                        && abs(driving.planning.angleUtilities.angdiff(...
                        finalPose(3), nearPose(3))) ...
                        <= thetaTol;
                    
                    % Collision checking already completed
                    if reachedNearPose
                        % Replace parent pose
                        treeLocal.replaceParent(nearId, newId);
                    end
                end
            end
        end
        
        %------------------------------------------------------------------
        function TF = inGoalRegion(this, pose)
            
            goalPose   = this.GoalPose;
            goalTol    = this.GoalTolerance;
            goalTol(3) = driving.planning.angleUtilities.convertAndWrapTo2Pi(...
                goalTol(3));
            
            TF = abs(pose(1)-goalPose(1)) <= goalTol(1) ...             % x in goal
                && abs(pose(2)-goalPose(2)) <= goalTol(2) ...           % && y in goal
                && abs(driving.planning.angleUtilities.angdiff(...
                pose(3),goalPose(3))) <= goalTol(3);                    % && theta in goal
        end
        
        %------------------------------------------------------------------
        function TF = terminateExploration(this, numIter)
            
            TF = numIter>= this.MinIterations;
        end
    end
    
    
    %----------------------------------------------------------------------
    % Input Parsing
    %----------------------------------------------------------------------
    methods (Access = private)
        %------------------------------------------------------------------
        function inputs = parseInputs(this, varargin)
            
            parser = inputParser;
            parser.FunctionName = mfilename;
            
            parser.addRequired('Costmap', @(cm)validateattributes(cm,{'vehicleCostmap'},{'nonempty'},mfilename));
            parser.addParameter('GoalTolerance', this.GoalTolerance);
            parser.addParameter('GoalBias', this.GoalBias);
            parser.addParameter('ConnectionMethod', this.ConnectionMethod);
            parser.addParameter('ConnectionDistance', this.ConnectionDistance);
            parser.addParameter('MinTurningRadius', this.MinTurningRadius);
            parser.addParameter('MinIterations', this.MinIterations);
            parser.addParameter('MaxIterations', this.MaxIterations);
            parser.addParameter('ApproximateSearch', this.ApproximateSearch);
            
            parser.parse(varargin{:});
            
            inputs = parser.Results;
        end
        
        %------------------------------------------------------------------
        function validatePoses(this, startPose, goalPose)
            
            this.validatePose(startPose, 'startPose');
            this.validatePose(goalPose, 'goalPose');
            
            % Here poses are still in degrees.
            poses = [startPose;goalPose];
            
            % Convert to radians.
            poses(:,3) = deg2rad(poses(:,3));
            
            % Check that poses are obstacle-free.
            throwError = false;
            free = this.Costmap.checkFreeVehiclePoses(poses, throwError);
            if any(~free)
                error(message('driving:pathPlannerRRT:posesInCollision'))
            end
        end
        
        %------------------------------------------------------------------
        function inputs = parsePlotInputs(this, varargin)
            
            parser = inputParser;
            parser.FunctionName = mfilename;
            
            parser.addParameter('Parent', [], @this.validateParent);
            parser.addParameter('Tree', 'off', @this.validateOnOff);
            parser.addParameter('Vehicle', 'on', @this.validateOnOff);
            
            parser.parse(varargin{:});
            
            inputs = parser.Results;
        end
        
        %------------------------------------------------------------------
        function tf = validateParent(~, parent)
            
            tf = true;
            if isempty(parent)
                return;
            end
            
            if ~ishghandle(parent) || ~strcmp(get(parent,'Type'), 'axes')
                error(message('driving:pathPlannerRRT:validParent'))
            end
        end
        
        %------------------------------------------------------------------
        function validateOnOff(~, onoff)
            
            % This will error if it's not an expected on/off value.
            matlab.lang.OnOffSwitchState(onoff);
        end
        
        %------------------------------------------------------------------
        function checkIterations(~, minIter, maxIter)
            
            validateattributes(minIter, {'numeric'}, ...
                {'scalar','<=',maxIter}, mfilename, 'MinIterations');
        end
    end
    
    
    %----------------------------------------------------------------------
    % Plotting
    %----------------------------------------------------------------------
    methods (Access = private)
        %------------------------------------------------------------------
        function plotTree(this, hAx, treeColor)
            
            treeNodes = this.Tree.Nodes;
            treeEdges = this.Tree.Edges;
            
            if isempty(treeNodes)
                return;
            end
            
            % Plot poses using scatter
            scatter(hAx, treeNodes(:,1), treeNodes(:,2), [], treeColor, ...
                'filled', 'Tag', 'rrtExploredNodes', ...
                'DisplayName', ...
                getString(message('driving:pathPlannerRRT:treeLegendString')));
            
            numSteps = this.ConnectionMechanism.NumSteps;
            numEdges = size(treeEdges,1);
            
            % Compute all interpolated poses
            poses = nan((numSteps+2)*numEdges, 3);
            idx = 1 : numSteps;
            for n = 1 : numEdges
                
                from = treeEdges(n,1);
                to   = treeEdges(n,2);
                
                % Add start of link
                poses(idx(1),:) = treeNodes(from,:);
                
                % Add points along link
                poses(idx+1,:) = this.ConnectionMechanism.interpolate(...
                    treeNodes(from,:), treeNodes(to,:) );
                
                idx = idx + numSteps + 2;
            end
            
            hLines = plot(hAx, poses(:,1), poses(:,2), 'Color', treeColor, ...
                'LineWidth', 1, 'Tag', 'rrtExploredPath');
            
            % Turn off legend display of lines
            hLines.Annotation.LegendInformation.IconDisplayStyle = 'off';
        end
        
        %------------------------------------------------------------------
        function plotPath(this, hAx, pathColor, vehicleOnOff)
            
            path = this.Path;
            if isempty(this.Path) || isempty(this.Path.KeyPoses)
                return;
            end
            
            vdims       = this.Costmap.VehicleDimensions;
            legendName  = getString(message('driving:pathPlannerRRT:pathLegendString'));
            
            plot(path, 'Parent', hAx, 'Color', pathColor, ...
                'Vehicle', vehicleOnOff, 'VehicleDimensions', vdims, ...
                'DisplayName', legendName, 'Tag', 'rrtSolved');
        end
        
        %------------------------------------------------------------------
        function plotEndPoints(this, hAx, goalColor)
            
            goalPose  = this.GoalPose;
            if ~isempty(goalPose)
                scatter(hAx, goalPose(:,1), goalPose(:,2), [], goalColor, ...
                    'filled', 'Tag', 'rrtGoal', 'DisplayName', ...
                    getString(message('driving:pathPlannerRRT:goalLegendString')));
            end
        end
    end
    
end