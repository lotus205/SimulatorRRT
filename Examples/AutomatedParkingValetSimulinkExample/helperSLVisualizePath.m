function helperSLVisualizePath(pose, steer, splineData, costmapStruct, vehicleDimsStruct)

%helperSLVisualizePath visualize reference path and vehicle position.

% Copyright 2017-2018 The MathWorks, Inc.

persistent pathPoints vehicleDims costmap vehicleBodyHandle 

if isempty(costmap)
    % Initialize vehicleDimensions object
    vehicleDims = vehicleDimensions( ...
        vehicleDimsStruct.Length, ...
        vehicleDimsStruct.Width, ...
        vehicleDimsStruct.Height, ...
        'Wheelbase',    vehicleDimsStruct.Wheelbase, ...
        'RearOverhang', vehicleDimsStruct.RearOverhang, ...
        'WorldUnits',   char(vehicleDimsStruct.WorldUnits));
    
    % Initialize vehicleCostmap object
    costmap = vehicleCostmap(costmapStruct.Costs, ...
        'FreeThreshold',     costmapStruct.FreeThreshold, ...
        'OccupiedThreshold', costmapStruct.OccupiedThreshold, ...
        'MapLocation',       costmapStruct.MapExtent([1, 3]), ...
        'CellSize',          costmapStruct.CellSize, ...
        'VehicleDimensions', vehicleDims, ...
        'InflationRadius',   costmapStruct.InflationRadius);
end

if isempty(pathPoints)
    pathPoints = zeros(size(splineData.x, 1), 2);
end

% Plot smooth path and map
if ~isequal(pathPoints, [splineData.x, splineData.y])
    % Initialize figure
    if ~any(pathPoints)
        % Plot path
        fh             = figure;
        fh.Name        = 'Automated Parking Valet';
        fh.NumberTitle = 'off';
        ax             = axes(fh);
        
        plot(costmap, 'Parent', ax, 'Inflation', 'off');
        legend off
        
        hold(ax, 'on');
        title(ax, '');
        
        ax.XLim = costmap.MapExtent(1:2);
        ax.YLim = costmap.MapExtent(3:4);
    end
    % Plot smooth path
    plot(splineData.x, splineData.y, 'r.', 'LineWidth', 3);
    
    % Update path points for the new path segment
    pathPoints = [splineData.x, splineData.y];
end

% Plot vehicle
if isempty(vehicleBodyHandle) && ~isequal(pose(1:2), [0 0]) % Simulink starts at [0 0 0]
    vehicleBodyHandle = helperPlotVehicle(pose, vehicleDims, steer);
elseif ~isequal(pose(1:2), [0 0])
    vehicleShapes = helperVehiclePolyshape(pose, vehicleDims, steer);
    
    for n = 1 : numel(vehicleBodyHandle)
        vehicleBodyHandle(n).Shape = vehicleShapes(n);
    end
end

uistack(vehicleBodyHandle, 'top');

drawnow('limitrate');
