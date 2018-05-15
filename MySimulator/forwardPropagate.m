% function finalPoses = forwardPropagate(from, actions)
tic
load_system('Vehicle');

m       = 1575;     % Total mass of vehicle                          (kg)
Iz      = 2875;     % Yaw moment of inertia of vehicle               (m*N*s^2)
lf      = 1.2;      % Longitudinal distance from c.g. to front tires (m)
lr      = 1.6;      % Longitudinal distance from c.g. to rear tires  (m)
Cf      = 19000;    % Cornering stiffness of front tires             (N/rad)
Cr      = 33000;    % Cornering stiffness of rear tires              (N/rad)

from = [0 10 0 0 0 0];
x0_ego = from(1);
y0_ego = from(2);
yaw0_ego = from(3);
xdot0 = from(4);
ydot0 = from(5);
yawrate0 = from(6);

            numAngles = 6;
            numForces = 3;
            numActions = numAngles * numForces;
            SteeringAngles = linspace(-40, 40, numAngles);
            Forces = linspace(-500, 500, numForces);
            actions = zeros(numActions, 2);
            
            for i = 1:numAngles
                for j = 1:numForces
                    actions(i * j, :) = [SteeringAngles(i) Forces(j)];
                end
            end
            
for i = 18 : -1 : 1
    t = (0:0.01:0.1)';
    SteeringAngle = actions(i, 1);
    Force = actions(i, 2);
    u2 = SteeringAngle * ones(size(t));
    u1 = Force * ones(size(t));
    in(i) = Simulink.SimulationInput('Vehicle');
    in(i) = in(i).setModelParameter('StartTime','0','StopTime','0.1');%,'SimulationMode','rapid');
%     in(i).ExternalInput = [t, u1, u2];
    in(i) = in(i).setExternalInput([t, u1, u2]);
    in(i) = in(i).setVariable('m',m);
    in(i) = in(i).setVariable('lf',lf);
    in(i) = in(i).setVariable('lr',lr);
    in(i) = in(i).setVariable('Cf',Cf);
    in(i) = in(i).setVariable('Cr',Cr);
    in(i) = in(i).setVariable('Iz',Iz);
    in(i) = in(i).setVariable('x0_ego',x0_ego);
    in(i) = in(i).setVariable('y0_ego',y0_ego);
    in(i) = in(i).setVariable('yaw0_ego',yaw0_ego);
    in(i) = in(i).setVariable('xdot0',xdot0);
    in(i) = in(i).setVariable('ydot0',ydot0);
    in(i) = in(i).setVariable('yawrate0',yawrate0);
    out(i) = sim(in(i));
end

% out = parsim(in,'ShowSimulationManager','on','ShowProgress','on');

finalPoses = zeros(numActions,6);
for i = 1:numActions
    for j = 1:6
     finalPoses(i,j) = out(i).yout{j}.Values.Data(end);
    end
end

% 
% SteeringAngles = 0 : -10 : -60;
% 
% for i = angles_length : -1 : 1
%     t = (1:0.01:2)';
%     u2 = SteeringAngles(i) * ones(size(t));
%     u1 = EgoVleocity * ones(size(t));
%     in(i) = Simulink.SimulationInput('Vehicle');
%     in(i) = in(i).setModelParameter('StartTime','0','StopTime','2');
%     in(i) = in(i).setInitialState(out(i).xFinal);
%     in(i) = in(i).setExternalInput([t, u1, u2]);
%     in(i) = in(i).setVariable('m',m);
%     in(i) = in(i).setVariable('lf',lf);
%     in(i) = in(i).setVariable('lr',lr);
%     in(i) = in(i).setVariable('Cf',Cf);
%     in(i) = in(i).setVariable('Cr',Cr);
%     in(i) = in(i).setVariable('Iz',Iz);
%     in(i) = in(i).setVariable('x0_ego',x0_ego);
%     in(i) = in(i).setVariable('y0_ego',y0_ego);
%     in(i) = in(i).setVariable('yaw0_ego',yaw0_ego);
% %     out(i) = sim(in(i));
% end
% 
% out = parsim(in);%,'ShowSimulationManager','on','ShowProgress','on');
toc