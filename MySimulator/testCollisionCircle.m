lengthCar = 4.7;
widthCar = 1.8;
margin = 0.1;
lengthCar = lengthCar + margin;
widthCar = widthCar + margin;
figure
rectangle('Position',[0 0 lengthCar widthCar])
axis([-2 8 -2 5])
axis equal

nCircle = 1;

l1 = lengthCar / nCircle;
radius = norm([widthCar/2 l1/2]);
for i = 1:nCircle
    x(i) = l1/2 + (i-1)*l1;
    y(i) = widthCar/2;
    radiuses(i) = radius;
end

viscircles([x' y'],radiuses,'Color','b');


frMargin = radius - l1/2
lrMargin = radius - widthCar/2