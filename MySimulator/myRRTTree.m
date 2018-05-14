%driving.planning.RRTTree tree data structure for RRT planning.

% Copyright 2017 The MathWorks, Inc.
classdef myRRTTree < vision.internal.EnforceScalarHandle
    
    properties (SetAccess = private)
        %Nodes
        %   Nodes in the tree, of size numNodes-by-3.
        Nodes
        
        %Edges
        %   Edges in the tree, of size numEdges-by-3.
        Edges
        
        %Costs
        %   Edge costs in the tree, of size numEdges-by-1.
        Costs
        
        %NeighborSearcher
        %   Near neighbor searcher object.
        NeighborSearcher = driving.planning.SqrtApproxNeighborSearcher;
    end
    
    properties (Dependent, SetAccess = private)
        %NumNodes
        %   Number of nodes in the tree.
        NumNodes
    end
    
    properties (Access = {?driving.planning.NeighborSearcher})
        % Neighbor searcher has direct access to node buffer to reduce
        % performance hit of using property accessors (get.Nodes) and reduce
        % memory hit of copying nodes over.
        
        NodeBuffer
        NodeBufferLength
        NodeIndex
    end
    
    properties (Access = private)
        EdgeBuffer
        EdgeBufferLength
        EdgeIndex
        
        CostBuffer
        
        BufferSize = 1e4;
        
        NodeDim
        
        Precision
        
        RewireFactor = 1;
        kRRT
    end
    
    
    %----------------------------------------------------------------------
    % API
    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        function this = myRRTTree(nodeDim, precision)
            
            if nargin<2
                precision = 'double';
            else
                precision = validatestring(precision, {'single','double'}, ...
                    'RRTTree', 'precision');
            end
            
            this.NodeDim    = nodeDim;
            this.Precision  = precision;
            
            this.reset();
        end
        
        %------------------------------------------------------------------
        function reset(this)
            
            if isequal(this.NodeIndex,1)
                return;
            end
            
            nodeDim     = this.NodeDim;
            precision   = this.Precision;
            
            % Allocate buffers
            this.NodeBuffer = zeros(this.BufferSize, nodeDim, precision);
            this.EdgeBuffer = zeros(this.BufferSize, 2, 'uint32');
            this.CostBuffer = zeros(this.BufferSize, 1, precision);
            
            this.NodeIndex = 1;
            this.NodeBufferLength = this.BufferSize;
            
            this.EdgeIndex = 1;
            this.EdgeBufferLength = this.BufferSize;
            
            % Compute constant factor for number of neighbors
            % See "Sampling-based Algorithms for Optimal Motion Planning".
            stateDim = this.NodeDim-1;
            this.kRRT = this.RewireFactor * 2^(stateDim + 1) * exp(1) ...
                * (1 + 1/stateDim);
        end
        
        %------------------------------------------------------------------
        function configureNeighborSearcher(this, approxSearch)
            
            switch approxSearch
                case true
                    this.NeighborSearcher = driving.planning.SqrtApproxNeighborSearcher(this);
                case false
                    this.NeighborSearcher = driving.planning.ExactNeighborSearcher(this);
            end
        end
        
        %------------------------------------------------------------------
        function configureConnectionMechanism(this, connMechanism)
            
            this.NeighborSearcher.configureConnectionMechanism(...
                connMechanism);
        end
        
        %------------------------------------------------------------------
        function id = addNode(this, node)
            
            id = this.NodeIndex;
            
            this.NodeBuffer(id,:) = node;
            
            this.NodeIndex = id + 1;
            
            if this.NodeIndex>this.NodeBufferLength
                this.growNodeBuffer();
            end
        end
        
        %------------------------------------------------------------------
        function addEdge(this, fromId, toId)
            
            edgeId = this.EdgeIndex;
            
            this.EdgeBuffer(edgeId,:) = [fromId, toId];
            
            this.CostBuffer(edgeId) = this.costTo(fromId) + ...
                this.edgeCost(fromId, toId);
            
            this.EdgeIndex = edgeId + 1;
        end
        
        %------------------------------------------------------------------
        function parentId = nodeParent(this, childId)
            
            parentId = this.EdgeBuffer(childId-1,1);
        end
        
        %------------------------------------------------------------------
        function replaceParent(this, childId, newParentId)
            
            idx = childId-1;
            
            %assert(idx>0, 'Bad childId')
            
            this.EdgeBuffer(idx,1) = newParentId;
            
            this.CostBuffer(idx) = this.costTo(newParentId) + ...
                this.edgeCost(newParentId, childId);
            
            this.rectifyDownstreamCosts(childId);
        end
        
        %------------------------------------------------------------------
        function cost = edgeCost(this, fromId, toId)
            
            cost = this.NeighborSearcher.distance(...
                this.NodeBuffer(fromId,:), this.NodeBuffer(toId,:));
        end
        
        %------------------------------------------------------------------
        function cost = costTo(this, id)
            
            if id<2
                cost = 0;
            else
                cost = this.CostBuffer(id-1);
            end
        end
        
        %------------------------------------------------------------------
        function [nearestNode, nearestId] = nearest(this, node)
            
            [nearestNode, nearestId] = this.NeighborSearcher.nearest(node);
        end
        
        %------------------------------------------------------------------
        function [nearNodes, nearIds] = near(this, node)
            
            numNodes = this.NodeIndex-1;
            K = ceil( this.kRRT * log(numNodes + 1) );
            
            [nearNodes, nearIds] = this.NeighborSearcher.near(node, K);
        end
        
        %------------------------------------------------------------------
        function [path,totalCost] = shortestPathFromRoot(this, childId, goalNodeIds)
            
            % Compute shortest path
            rootId = 1;
            edgeId = max(childId - 1, 1);
            parentId = this.EdgeBuffer(edgeId, 1);
            
            path = [this.EdgeBuffer(edgeId, 2) parentId];
            
            while parentId ~= rootId
                parentId = this.EdgeBuffer(parentId-1, 1);
                path(end+1) = parentId; %#ok<AGROW>
            end
            path = path(end : -1 : 1);
            
            % Remove self-loops
            [~,repeatedGoalNodeIds] = intersect(path, goalNodeIds, 'stable');
            
            if ~isempty(repeatedGoalNodeIds)
                path = path( 1 : repeatedGoalNodeIds(1) );
            end
            
            % Find cost to childId
            totalCost = this.costTo(childId);
        end
        
        %------------------------------------------------------------------
        function G = toDigraph(this)
            
            % Create node table with poses recorded in variables X, Y and
            % Heading.
            nodes = this.Nodes;
            nodeTable = table(nodes(:,1), nodes(:,2), rad2deg(nodes(:,3)), ...
                'VariableNames', {'X', 'Y', 'Heading'});
            
            % Create edge table with costs as edge weights.
            edges = this.Edges;
            costs = this.Costs;
            edgeTable = table(edges, costs, 'VariableNames', ...
                {'EndNodes', 'Weight'});
            
            G = digraph(edgeTable, nodeTable);
        end
    end
    
    
    %----------------------------------------------------------------------
    % Accessors
    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        function nodes = get.Nodes(this)
            
            nodes = this.NodeBuffer(1 : this.NodeIndex-1, :);
        end
        
        %------------------------------------------------------------------
        function edges = get.Edges(this)
            
            edges = this.EdgeBuffer(1 : this.EdgeIndex-1, :);
        end
        
        %------------------------------------------------------------------
        function costs = get.Costs(this)
            
            costs = this.CostBuffer(1 : this.EdgeIndex-1);
        end
    end
    
    
    %----------------------------------------------------------------------
    % Buffer Management
    %----------------------------------------------------------------------
    methods (Access = private)
        %------------------------------------------------------------------
        function growNodeBuffer(this)
           
            this.NodeBuffer(end+this.BufferSize,end) = 0;
            
            this.NodeBufferLength = size(this.NodeBuffer,1);
        end
        
        %------------------------------------------------------------------
        function growEdgeBuffer(this)
            
            this.EdgeBuffer(end+this.BufferSize,end) = 0;
            this.CostBuffer(end+this.BufferSize,end) = 0;
            
            this.EdgeBufferLength = size(this.EdgeBuffer,1);
        end
    end
    
    %----------------------------------------------------------------------
    % Cost Management
    %----------------------------------------------------------------------
    methods (Access = private)
        %------------------------------------------------------------------
        function rectifyDownstreamCosts(this, childId)
            
            % Find all edges whose parent is childId
            edgeIndicesToRectify = find(this.EdgeBuffer(1 : this.EdgeIndex-1,1) == childId);
            
            if isempty(edgeIndicesToRectify)
                return;
            end
            
            % Update cost buffer for each of these
            for n = 1 : numel(edgeIndicesToRectify)
                edgeId = edgeIndicesToRectify(n);
                
                parentId = this.EdgeBuffer(edgeId,1);
                childId  = this.EdgeBuffer(edgeId,2);
                this.CostBuffer(edgeId) = this.costTo(parentId) + this.edgeCost(parentId,childId);
                
                % Recursive call to rectify costs
                % NOTE: This will terminate because there are no cycles in
                %       the tree.
                this.rectifyDownstreamCosts(childId);
            end
        end
    end
end