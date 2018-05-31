function costmap = creatMyCostMap()
figure
axis equal
[X,Y]=meshgrid(0:0.1:120,-15:0.1:30);
Y = flipud(Y);

tic

load 'patchRoads.mat'
numTiles = numel(rt);
mapLayers.RoudBoundaries = zeros(size(X));
for iTile=1:numTiles
    tile = rt(iTile);
    % patch('XData',tile.Vertices(:,1),'YData',tile.Vertices(:,2));
    % patch(tile.Vertices(:,1),tile.Vertices(:,2),tile.Vertices(:,3));
    mapLayers.RoudBoundaries = mapLayers.RoudBoundaries + inpolygon(X,Y,tile.Vertices(:,1),tile.Vertices(:,2));
end
mapLayers.RoudBoundaries = ~mapLayers.RoudBoundaries;

load patchActors.mat
mapLayers.Actors = ones(size(X));
for iActor=1:numel(actors)
%     iActor = numel(actors);
    actor = actors(iActor);
                    
    % get faces in scenario coordinates for this actor
    faces = scenarioFaces(actor);
    face = faces(:,:,6)';
    mapLayers.Actors = mapLayers.Actors - inpolygon(X,Y,face(:,1),face(:,2));
end
mapLayers.Actors = ~mapLayers.Actors;

toc

combinedMap = mapLayers.Actors + mapLayers.RoudBoundaries;
combinedMap(find(combinedMap > 1)) = 1;
combinedMap(find(combinedMap < 0)) = 0;

% imshow(combinedMap)
res = 0.1; % meters
costmap = vehicleCostmap(combinedMap, 'CellSize', res, ...
                                       'MapLocation', [0, -15], ...
                                       'InflationRadius', 1.4);
plot(costmap);

end
% figure
% y = rgb2gray(X);
% imshow(y);
% figure
% z = imbinarize(y);
% imshow(z);