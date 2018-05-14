mapLayers.StationaryObstacles = imread('stationary.bmp');
mapLayers.RoadBoundaries      = imread('road_markings.bmp');

% Combine map layers struct into a single vehicleCostmap.
combinedMap = mapLayers.StationaryObstacles + mapLayers.RoadBoundaries;

combinedMap = im2single(combinedMap);

res = 0.5; % meters
costmap = vehicleCostmap(combinedMap, 'CellSize', res);