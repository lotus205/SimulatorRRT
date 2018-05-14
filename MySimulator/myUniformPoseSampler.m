%driving.planning.UniformPoseSampler Uniform pose sampler with goal
%biasing.
%
%   Usage
%   -----
%   % Construct pose sampler
%   mapExtent = [xmin xmax ymin ymax];
%   sampler = driving.planning.UniformPoseSampler(mapExtent, 'double');
%
%   % Sample a pose
%   randPose = sampler.sample()
%
%   % Sample a goal bias
%   goalBias = sampler.sampleGoalBias()
%
%   Notes
%   -----
%   1. This class creates a sample buffer for pose and goal bias of length
%      equal the BufferSize property.
%   2. Precision of samples is determined by second input to the
%      constructor.

% Copyright 2017 The MathWorks, Inc.
classdef myUniformPoseSampler < vision.internal.EnforceScalarHandle
    
    properties (Access = private)
        %Dimensions
        %x y yaw, vx vy w 6 dimensions 
        PoseDimensions = 6;
        %PoseBuffer
        %   Buffer of poses
        PoseBuffer
        
        %PoseIndex
        %   Index to next pose in pose buffer
        PoseIndex
        
        %GoalBiasBuffer
        %   Buffer of goal biases
        GoalBiasBuffer
        
        %GoalBiasIndex
        %   Index to next goal bias in goal bias buffer
        GoalBiasIndex
        
        %LowerLimits
        %   Lower limits of world
        LowerLimits
        
        %UpperLimits
        %   Upper limits of world
        UpperLimits
        
        %BufferSize
        %   Size of buffer
        BufferSize = 5e3;
        
        % Costmap
        %   Costmap for collision checking
        Costmap
        
        % CollisionFree
        %   Indices of collision free poses
        CollisionFree
    end
    
    methods
        %------------------------------------------------------------------
        function this = myUniformPoseSampler(mapExtent, precision)
            
            validateattributes(mapExtent, {'single', 'double'}, ...
                {'vector', 'numel', 4, 'finite'}, 'UniformPoseSampler', ...
                'mapExtent');
            
            if nargin<2
                precision = 'double';
            else
                precision = validatestring(precision, {'single','double'}, ...
                    'UniformPoseSampler', 'precision');
            end
            %x y yaw, vx vy w 6 dimensions 
            this.LowerLimits = cast([mapExtent(1), mapExtent(3), 0, 0, 0, -3]', ...
                precision);
            this.UpperLimits = cast([mapExtent(2), mapExtent(4), 2*pi, 10, 10, 3]', ...
                precision);
            
            this.fillPoseBuffer();
            this.fillGoalBiasBuffer();
        end
        
        %------------------------------------------------------------------
        function reset(this)
            
            this.fillPoseBuffer();
            this.fillGoalBiasBuffer();
        end
        
        %------------------------------------------------------------------
        function configureCollisionChecker(this, cmap)
            
            this.Costmap = cmap;
            
            mapExtent = [this.LowerLimits(1) this.UpperLimits(1) ...
                this.LowerLimits(2) this.UpperLimits(2)];
            
            assert(all(cast(cmap.MapExtent, 'like', mapExtent) ...
                == mapExtent), ...
                'Expected costmap to be consistent with sampling limits');
            this.checkForCollisions();
        end
        
        %------------------------------------------------------------------
        function [pose,collisionFree] = sample(this)
            
            pose = (this.PoseBuffer(:, this.PoseIndex)).';
            
            collisionFree = this.CollisionFree(this.PoseIndex);
            
            this.PoseIndex = this.PoseIndex + 1;
            
            if this.PoseIndex > this.BufferSize
                this.fillPoseBuffer();
            end
        end
        
        %------------------------------------------------------------------
        function goalBias = sampleGoalBias(this)
            
            goalBias = this.GoalBiasBuffer(this.GoalBiasIndex);
            
            this.GoalBiasIndex = this.GoalBiasIndex + 1;
            
            if this.GoalBiasIndex > this.BufferSize
                this.fillGoalBiasBuffer();
            end
        end
        
        %------------------------------------------------------------------
        function pose = sampleCollisionFree(this)
            
            collisionFree = false;
            while ~collisionFree
                
                [pose,collisionFree] = this.sample();
            end
        end
    end
    
    methods (Access = private)
        %------------------------------------------------------------------
        function fillPoseBuffer(this)
            
            this.PoseBuffer = this.LowerLimits + ...
                (this.UpperLimits - this.LowerLimits) .* ...
                rand(this.PoseDimensions, this.BufferSize, 'like', this.LowerLimits);
            
            this.PoseIndex = 1;
        end
        
        %------------------------------------------------------------------
        function fillGoalBiasBuffer(this)
            
            this.GoalBiasBuffer = rand(1, this.BufferSize, 'like', this.LowerLimits);
            
            this.GoalBiasIndex = 1;
        end
        
        %------------------------------------------------------------------
        function checkForCollisions(this)
            
            if ~isempty(this.Costmap)
                vehiclePoses    = this.PoseBuffer(1:3,:)';
                throwError      = false;
                
                this.CollisionFree = checkFreeVehiclePoses(this.Costmap, ...
                    vehiclePoses, throwError);
                fprintf("CollisionFree called in uniformsmapler");
            end
        end
    end
end