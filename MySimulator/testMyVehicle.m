clear

vehicle = myVehicle;

currentState = [0 0 0 0 0 0]; % [x, y, yawAngle, longitudeSpeed, lateralSpeed, yawRate]
input = [20*pi/180 2000];     % steering angle in rad
duration = 25;                  % time (s)
tic
[nextState, log, t] = vehicle.propagate(currentState, input, duration, 'Log', true);
toc
nextState(3) = nextState(3) * 180 / pi;
nextState(6) = nextState(6) * 180 /pi;
log(:, 3) = log(:, 3) .* 180/pi;
log(:, 6) = log(:, 6) .* 180/pi;

legend_name = {'X position','Y position','Yaw angle','Longitude speed','Lateral speed','Yaw rate'};
figure
title('Log data for dynamic model')
for(i = 1 : 6)
    subplot(2, 3, i)
    xlabel('time')
    plot(t, log(:, i))
    legend(legend_name(i))
    datacursormode on
    grid on
end
