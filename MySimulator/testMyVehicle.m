clear

vehicle = myVehicle;

currentState = [0 0 0 10 0 0]; % [x, y, yawAngle, longitudeSpeed, lateralSpeed, yawRate]
input = [0, 5000];     % steering angle in rad
duration = 10;                  % time (s)
[nextState, log, t] = vehicle.propagate(currentState, input, duration, 'Log', true);

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
