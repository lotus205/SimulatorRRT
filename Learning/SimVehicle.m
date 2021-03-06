tic
load_system('Vehicle');
SteeringAngles = 0 : 10 : 60;
angles_length = length(SteeringAngles);
EgoVleocity = 1;

m       = 1575;     % Total mass of vehicle                          (kg)
Iz      = 2875;     % Yaw moment of inertia of vehicle               (m*N*s^2)
lf      = 1.2;      % Longitudinal distance from c.g. to front tires (m)
lr      = 1.6;      % Longitudinal distance from c.g. to rear tires  (m)
Cf      = 19000;    % Cornering stiffness of front tires             (N/rad)
Cr      = 33000;    % Cornering stiffness of rear tires              (N/rad)
x0_ego = 0;
y0_ego = 0;
yaw0_ego = 0;

for i = angles_length : -1 : 1
    t = (0:0.01:1)';
    u2 = SteeringAngles(i) * ones(size(t));
    u1 = EgoVleocity * ones(size(t));
    in(i) = Simulink.SimulationInput('Vehicle');
    in(i) = in(i).setModelParameter('StartTime','0','StopTime','1');
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
%     out(i) = sim(in(i));
end

out = parsim(in);%,'ShowSimulationManager','on','ShowProgress','on');

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