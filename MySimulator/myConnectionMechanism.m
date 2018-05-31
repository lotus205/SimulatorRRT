%driving.planning.DubinsConnectionMechanism Dubins connection mechanism
%
%   Usage
%   -----
%   % Create a connection mechanism object
%   turningRadius         = 4;
%   numInterpolationSteps = 20;
%   connectionDistance    = 10;
%
%   connMech = ...
%       driving.planning.DubinsConnectionMechanism(turningRadius, ...
%       numInterpolationSteps, connectionDistance);
%
%   % Define two poses 
%   % Note that heading must be radians.
%   fromPose = [4 4 pi/2];
%   toPose   = [6 4 pi/2]; 
%
%   % Find the Dubins distance between two poses
%   d = connMech.distance(fromPose, toPose)
%
%   % Find wayposes between two poses along Dubins curve.
%   poses = connMech.interpolate(fromPose, toPose)

% Copyright 2017 The MathWorks, Inc.

classdef myConnectionMechanism < driving.planning.ConnectionMechanism
    
    properties
        TurningRadius
    end
    
    properties (Constant)
        Exact = true;
    end
    
    methods
        %------------------------------------------------------------------
        function this = myConnectionMechanism(turningRadius, numSteps, connectionDistance)
            
            this@driving.planning.ConnectionMechanism(numSteps, connectionDistance);
            
            this.TurningRadius = turningRadius;
        end
        
        %------------------------------------------------------------------
        function d = distance(this, from, to)
            
%             d = drivingDubinsDistance(from(:, 1:3), to(:, 1:3), this.TurningRadius);
%             d = d + abs(from(:, 4:6) - to(:, 4:6)) * [3 3 3]' ;
            d = abs(from - to) * [3 3 3 3 3 3]' ;
            
        end
        
        %------------------------------------------------------------------
        function poses = interpolate(this, from, to)
            
%             poses = drivingDubinsInterpolate(from(:, 1:3), to(:, 1:3), ...
%                 this.ConnectionDistance, this.NumSteps, ...
%                 this.TurningRadius);
              poseNum = this.NumSteps;
              connectionDistance = this.ConnectionDistance;
              from = from(:, 1:3);
              to = to(:, 1:3);
              diff = to - from;
              if(norm(diff) > connectionDistance)
                  diff = diff * connectionDistance / norm(diff);
              end
              interp = zeros(poseNum, size(from, 2));
              for i = 1 : size(from, 2)
                  interp(:, i) = diff(:, i) * linspace(0, 1, poseNum);
              end
              poses = from + interp;
              
              poses = poses(:, 1:3);
        end
    end
end