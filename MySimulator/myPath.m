%driving.Path Planned vehicle path.
%   driving.Path is returned by the plan method of the pathPlannerRRT. It
%   holds the planned path as a set of key poses. Use checkPathValidity to
%   check validity of the path against a vehicleCostmap. Use the
%   connectingPoses method to compute intermediate poses along the path.
%
%   driving.Path properties:
%   KeyPoses            - Key poses along path. (read-only)
%   NumSegments         - Number of segments along path. (read-only)
%
%   driving.Path methods:
%   connectingPoses     - Compute connecting poses along path.
%   plot                - Plot a path.
%
%
%   Example - Plan a path and check if it is valid
%   ----------------------------------------------
%   % Load a costmap of a maze
%   data = load('mazeMap.mat');
%   mazeMap = data.mazeMap;
%   vdims   = mazeMap.VehicleDimensions;
% 
%   % Define a start and a goal
%   startPose = [3 2 0];
%   goalPose  = [45 35 135];
% 
%   % Create an RRT planner
%   planner = pathPlannerRRT(mazeMap, 'MinTurningRadius', 1);
% 
%   % Plan a path to the goal
%   path = plan(planner, startPose, goalPose);
% 
%   % Check if the path is valid
%   disp('Is path valid (before adding obstacle):')
%   isPathValid = checkPathValidity(path, mazeMap)
% 
%   % Display path on map
%   figure
%   plot(mazeMap)
%   hold on
%   plot(path, 'VehicleDimensions', vdims)
%   title('Valid path')
% 
%   % Add an obstacle on the map blocking the path
%   [X,Y] = meshgrid(4:10,7:9);
%   setCosts(mazeMap, [X(:),Y(:)], 1)
% 
%   disp('Is path valid (after adding obstacle):')
%   isPathValid = checkPathValidity(path, mazeMap)
% 
%   % Display path on map
%   figure
%   plot(mazeMap)
%   hold on
%   plot(path, 'VehicleDimensions', vdims)
%   title('Invalid path')
%
%
%   See also checkPathValidity, pathPlannerRRT, vehicleCostmap.

% Copyright 2017-2018 The MathWorks, Inc.

classdef myPath
    
    properties (SetAccess = private)
        %KeyPoses
        %   Key poses along the path specified as an M-by-3 matrix of
        %   [x,y,theta] in world co-ordinates.
        KeyPoses               = zeros(0,3);
        
        %NumSegments
        %   Number of segments along the path, specified as a scalar. A
        %   path with M key poses has M-1 segments.
        NumSegments
    end
    
    properties (Hidden)
        %Cost
        %   Total cost along path, computed as the sum of accumulated path
        %   lengths between poses.
        Cost                = 0;
        
        %ConnectionMethod
        %   Method used to connect two poses specified as one of the
        %   strings 'Dubins' or 'Reeds-Shepp'.
        %
        %   Default: Dubins
        ConnectionMethod
    end
    
    properties (Access = private)
        %ConnectionMechanism
        %   A driving.planning.ConnectionMechanism object.
        ConnectionMechanism
    end
    
    %----------------------------------------------------------------------
    % API
    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        function poses = connectingPoses(this, varargin)
            %connectingPoses - compute connecting poses along path.
            %
            %   poses = connectingPoses(path) computes connecting poses
            %   along the path and returns an M-by-3 array of [x,y,theta]
            %   poses along the path.
            %
            %   poses = connectingPoses(path, n) computes connecting poses
            %   along the nth segment along the path. A path has
            %   size(path.KeyPoses,1)-1 segments.
            %
            %   poses = connectingPoses(___,Name,Value) additionally
            %   specifies name-value pair arguments as described below:
            %
            %   'NumSamples'    The number of connecting poses between two
            %                   successive key poses. 
            %
            %                   Default: 100
            %
            %   Example - Visualize first segment of a planned path
            %   ---------------------------------------------------
            %   % Load a costmap
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
            %   % Extract poses along the first segment of the path
            %   segmentPoses = connectingPoses(refPath, 1);
            %
            %   % Display the first segment of the path on the map
            %   plot(parkMap)
            %   hold on
            %   plot(segmentPoses(:,1),segmentPoses(:,2), 'LineWidth', 2)
            %
            %   See also pathPlannerRRT.
            
            % Check that key poses are not empty
            validateattributes(this.KeyPoses, {'numeric'}, {'nonempty'}, ...
                'interpolate', 'KeyPoses');
            
            % Parse inputs
            parser = inputParser;
            parser.FunctionName = 'connectingPoses';
            
            parser.addOptional('n', []);
            parser.addParameter('NumSamples', 100, ...
                @(numSamples)validateattributes(numSamples, {'numeric'}, ...
                {'integer', 'scalar', 'real', '>', 1}));
            
            parse(parser, varargin{:});
            
            params = parser.Results;
            
            segNum     = params.n;
            numSamples = params.NumSamples;
            
            % Set the number of connection steps
            connMech = copy(this.ConnectionMechanism);
            connMech.NumSteps = numSamples - 1;
            
            % Convert poses from degrees to radians
            wayPosesRadians = this.KeyPoses;
            wayPosesRadians(:,3) = deg2rad(wayPosesRadians(:,3));
            
            if isempty(segNum) && any(contains(parser.UsingDefaults, 'n'))
                % Compute connecting poses along the entire path
                
                numSegments = this.NumSegments;
                
                poses = wayPosesRadians(1, :);
                
                for n = 1 : numSegments
                    from = wayPosesRadians(n  ,:);
                    to   = wayPosesRadians(n+1,:);
                    poses = [poses; ...
                        connMech.interpolate(from, to)]; %#ok<AGROW>
                end
            else
                validateattributes(segNum, {'numeric'}, ...
                    {'real', 'positive', 'scalar', 'integer', '<', size(this.KeyPoses,1)}, ...
                    'connectingPoses', 'n');
                
                from = wayPosesRadians(segNum  , :);
                to   = wayPosesRadians(segNum+1, :);
                poses = [from; connMech.interpolate(from, to)];
            end
            
            % Convert back to degrees
            poses(:,3) = rad2deg( poses(:,3) );
        end
        
        %------------------------------------------------------------------
        function plot(this, varargin)
            %plot - Plot a path.
            %
            %   plot(path) plots the path.
            %
            %   plot(...,Name,Value) specifies additional name-value pair
            %   arguments as described below:
            %
            %   'Parent'            Handle to an axes on which to display
            %                       the path.
            %
            %   'Vehicle'           A string 'on' or 'off' to turn on or
            %                       off the display of the vehicle along
            %                       the path. 
            %               
            %                       Default: 'on'
            %
            %   'VehicleDimensions' A vehicleDimensions object specifying
            %                       dimensions of the vehicle.
            %
            %                       Default: vehicleDimensions()
            %
            %   'DisplayName'       Name of the entry to show in the
            %                       legend. If no name is specified, no
            %                       entry is shown.
            %
            %                       Default: ''
            %
            %   'Color'             Color of the path, specified as an RGB
            %                       triplet or a color name. An RGB triplet
            %                       is a 3-element row vector whose
            %                       elements specify the intensities of the
            %                       red, green and blue components of the
            %                       color. The intensities must be in the
            %                       range [0,1].
            %
            %                       Default: Selected according to default
            %                                color order.
            %
            %   'Tag'               Tag used to identify path, specified as
            %                       a string.
            %
            %                       Default: ''
            %
            %
            %   Example - Display multiple vehicle paths
            %   ----------------------------------------
            %   % Create a costmap
            %   costmap = vehicleCostmap(zeros(100));
            %   setCosts(costmap, [70 15], 1);
            %
            %   % Configure a path planner
            %   planner = pathPlannerRRT(costmap);
            %
            %   % Plan two paths through the costmap
            %   path1 = plan(planner, [10 10  0], [40 40 90]);
            %   path2 = plan(planner, [40 40 90], [60 60  0]);
            %   
            %   % Display paths on the costmap
            %   plot(costmap)
            %   hold on
            %   plot(path1, 'Color', 'r', 'DisplayName', 'path1')
            %   plot(path2, 'Color', 'g', 'DisplayName', 'path2')
            %   legend on
            %
            %   See also vehicleCostmap/plot.
            
            p = inputParser;
            p.FunctionName = 'plot';
            p.addParameter('Parent', [], @this.validateParent);
            p.addParameter('Vehicle', 'on', @this.validateOnOff);
            p.addParameter('VehicleDimensions', vehicleDimensions, @this.validateVehicleDimensions);
            p.addParameter('DisplayName', '', @this.validateDisplayName);
            p.addParameter('Color', '', @this.validateColor);
            p.addParameter('Tag', '', @this.validateTag);
            
            parse(p, varargin{:});
            
            parent  = p.Results.Parent;
            vehicle = p.Results.Vehicle;
            vdims   = p.Results.VehicleDimensions;
            legname = p.Results.DisplayName;
            color   = p.Results.Color;
            tag     = p.Results.Tag;
            
            % Do the right thing
            hAx = newplot(parent);
            
            % Check hold state
            if ishold(hAx)
                oldState = 'on';
            else
                oldState = 'off';
            end
            
            % Turn on hold
            hold(hAx, 'on')
            restoreHoldState = onCleanup(@()hold(hAx, oldState));
            
            % Capture the color to use for plot.
            if isempty(color)
                pathColor = hAx.ColorOrder( hAx.ColorOrderIndex,: );
            else
                pathColor = color;
            end
            
            poses = this.KeyPoses;
            
            if isempty(poses)
                return;
            end
            
            hScatter = scatter(hAx, poses(:,1), poses(:,2), [], pathColor, ...
                'filled');
            
            if isempty(legname)
                turnOffLegend(hScatter);
            else
                hScatter.DisplayName = legname;
            end
            
            if ~isempty(tag)
                hScatter.Tag = strcat(tag, 'Nodes');
            end
            
            % Compute interpolated poses
            numSteps = this.ConnectionMechanism.NumSteps;
            numLinks = size(poses,1)-1;
            segmentPoses = nan(numSteps*numLinks + 1, 3);
            
            % Add start pose
            segmentPoses(1, :) = poses(1, :);
            
            % Extract and add segments
            idx = 2 : (numSteps+1);
            for n = 1 : numLinks
                % Extract segment
                segmentPoses(idx,:) = this.extractSegment(n);
                
                % Update indices
                idx = idx + numSteps;
            end
            
            hLines = plot(hAx, segmentPoses(:,1), segmentPoses(:,2), ...
                'LineWidth', 2, 'Color', pathColor);
            turnOffLegend(hLines);
            
            if ~isempty(tag)
                hLines.Tag = strcat(tag, 'Path');
            end
            
            if strcmp(vehicle,'on')
                % Use polyshape to draw a car at each pose
                
                ro = vdims.RearOverhang;
                fo = vdims.FrontOverhang;
                wb = vdims.Wheelbase;
                hw  = vdims.Width/2;
                carShapes = polyshape( ...
                    [-ro; wb+fo; wb+fo; -ro],...
                    [-hw;   -hw;    hw;  hw]);
                
                numPoses = size(poses,1);
                carShapes = repelem(carShapes, numPoses, 1);
                
                % Create a function handle to translate and rotate
                transform = @(shape, x, y, theta)rotate(...
                    translate(shape, [x,y]), theta, [x,y] );
                
                % Translate and rotate each car to it's respective way
                % point.
                carShapes = arrayfun(transform, carShapes, ...
                    poses(:,1), poses(:,2), poses(:,3));
                
                % Finally plot the cars
                hShapes = plot(hAx, carShapes, 'FaceColor', pathColor);
                turnOffLegend(hShapes);
            end
        end
    end
    
    
    %----------------------------------------------------------------------
    % Accessors
    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        function numSegments = get.NumSegments(this)
            
            numSegments = max(size(this.KeyPoses, 1) - 1, 0);
        end
        
        %------------------------------------------------------------------
        function connMethod = get.ConnectionMethod(this)
            
            if isa(this.ConnectionMechanism, 'driving.planning.ReedsSheppConnectionMechanism')
                connMethod = 'ReedsShepp';
            else % this is also the default
                connMethod = 'Dubins';
            end
        end
    end
    
    
    %----------------------------------------------------------------------
    % Hidden methods
    %----------------------------------------------------------------------
    methods(Hidden)
        %------------------------------------------------------------------
        function segmentPoses = extractSegment(this, n, inRadians)
            %extractSegment - extract a segment of the path.
            %   segmentPoses = extractSegment(path, n) extracts the nth
            %   segment of the path.
            
            validateattributes(n, {'numeric'}, ...
                {'real','scalar','positive','integer','<', size(this.KeyPoses,1)}, ...
                'segment', 'n');
            
            from    = this.KeyPoses(n  ,:);
            from(3) = deg2rad(from(3));
            
            to      = this.KeyPoses(n+1,:);
            to(3)   = deg2rad(to(3));
            
            segmentPoses = this.ConnectionMechanism.interpolate(from, to);
            
            if nargin==2 || ~inRadians
                segmentPoses(:,3) = rad2deg(segmentPoses(:,3));
            end
        end
    end
    
    
    %----------------------------------------------------------------------
    % Construction
    %----------------------------------------------------------------------
    methods (Hidden, Static)
        %------------------------------------------------------------------
        function this = create(varargin)
            this = myPath(varargin{:});
        end
    end
    
    methods (Access = protected)
        %------------------------------------------------------------------
        function this = myPath(poses, connMech, totalCost)
            
            if nargin == 0
                return;
            end
            
            this.KeyPoses               = this.checkPoses(poses);
            this.ConnectionMechanism = connMech;
            
            if nargin==3
                this.Cost           = this.checkCost(totalCost);
            end
        end
    end
    
    
    %----------------------------------------------------------------------
    % Input checking
    %----------------------------------------------------------------------
    methods(Access = private)
        %------------------------------------------------------------------
        function poses = checkPoses(~, poses)
            validateattributes(poses, {'single', 'double'}, ...
                {'ncols', 3, 'finite'}, mfilename, 'KeyPoses');
            
            % Wrap in [0,2pi].
            poses(:,3) = rad2deg(...
                driving.planning.angleUtilities.convertAndWrapTo2Pi( poses(:,3) ));
        end
        
        %------------------------------------------------------------------
        function cost = checkCost(~, cost)
            
            validateattributes(cost, {'single','double'}, ...
                {'scalar','nonnegative','finite'}, mfilename, 'Cost');
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
        function validateVehicleDimensions(~, vdim)
            
            validateattributes(vdim, {'vehicleDimensions'}, {'scalar'}, ...
                'plot', 'VehicleDimensions');
        end
        
        %------------------------------------------------------------------
        function validateDisplayName(~, name)
            
            validateattributes(name, {'char','string'}, {'scalartext'}, ...
                'plot', 'DisplayName');
        end
        
        %------------------------------------------------------------------
        function validateTag(~, tag)
            
            validateattributes(tag, {'char','string'},{'scalartext'}, ...
                'plot', 'Tag');
        end
        
        %------------------------------------------------------------------
        function validateColor(~,color)
            
            if ischar(color) || isstring(color)
                validateattributes(color, {'char','string'}, ...
                    {'nonempty','scalartext'}, 'plot', 'Color');
                
                specOptions = {'red','green','blue','yellow','magenta',...
                    'cyan','white','black','r','g','b','y','m','c','w','k'};
                
                % Find best match for the given color string
                validatestring(color, specOptions, 'plot', 'Color');
            else
                validateattributes(color, {'double'}, ...
                    {'nonempty','>=', 0, '<=', 1, 'size', [1 3]}, ...
                    'plot', 'Color');
            end
        end
    end
    
    
    %----------------------------------------------------------------------
    % Save/Load
    %----------------------------------------------------------------------
    properties (Access = private)
        %Version
        %   Version tag for handling save/load in case of interface
        %   changes.
        Version = ver('driving');
    end
    
    methods (Static, Hidden)
        %------------------------------------------------------------------
        function this = loadobj(that)
            
            this = driving.Path(that.KeyPoses, that.ConnectionMechanism, that.Cost);
        end
    end
    
    methods (Hidden)
        %------------------------------------------------------------------
        function that = saveobj(this)
            
            that.KeyPoses               = this.KeyPoses;
            that.ConnectionMechanism    = this.ConnectionMechanism;
            that.Cost                   = this.Cost;
            
            that.Version                = this.Version;
        end
    end
end

function turnOffLegend(hObj)

for n = 1 : numel(hObj)
    hObj(n).Annotation.LegendInformation.IconDisplayStyle = 'off';
end
end