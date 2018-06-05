throwError = false;
tic
for i = 1 : 1000
    Costmap.checkFreeVehiclePoses([110 25.5 0], throwError);
end
toc