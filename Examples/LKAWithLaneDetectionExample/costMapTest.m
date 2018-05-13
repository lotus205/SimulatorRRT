load costMapData.mat
figure
axis equal
[X,Y]=meshgrid(0:0.1:500,-50:0.1:200);
in = zeros(size(X));
tic
for iTile=1:numTiles
tile = rt(iTile);
% patch('XData',tile.Vertices(:,1),'YData',tile.Vertices(:,2));
% patch(tile.Vertices(:,1),tile.Vertices(:,2),tile.Vertices(:,3));
 in = in + inpolygon(X,Y,tile.Vertices(:,1),tile.Vertices(:,2));
end
toc
F = getframe(gcf);
figure
[X, Map] = frame2im(F);
imshow(X)
figure 
imshow(in)
% figure
% y = rgb2gray(X);
% imshow(y);
% figure
% z = imbinarize(y);
% imshow(z);