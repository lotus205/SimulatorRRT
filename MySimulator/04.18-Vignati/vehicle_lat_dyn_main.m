%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vehicle Lateral Dynamics Simulation 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear
close all
clc


Ts = 0.1;               % Simulation sample time                (s)

modelName = 'MoveMove';
wasModelLoaded = bdIsLoaded(modelName);
if ~wasModelLoaded
    load_system(modelName)
end
% load the bus for scenario reader
blk=find_system(modelName,'System','helperScenarioReader');
ss = get_param(blk{1},'PortHandles');
get(ss.Outport(1),'SignalHierarchy');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% load vehicle data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
g   = 9.80665;
data_vehicle

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% load driver data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
data_driver

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% simulation parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Tend    = 25;       % simulation time
dt_save = 1e-2;     % sample rate
t = (0:dt_save:Tend)';

psip0 = 0;
vy0   = 0;
XG0   = 0;
YG0   = 0;
psi0  = 0;
alfaf0= 0;
alfar0= 0;

% deltah(:,1) = [0:0.01:1 Tend]';
% deltah(:,2) = [tanh(((0:0.01:1)-0.5)*10)*0.5+0.5 1]'*10/180*pi;
deltah(:,1) = [0  Tend]';
deltah(:,2) = [1 1]'*20/180*pi;
% deltah(:,1) = [0 Tend]';
% deltah(:,2) = [0 1]'*15/180*pi;

delta_OL = 0;

vxh(:,1)    = [0 Tend]';
vxh(:,2)    = 50*[1 1]'/3.6;
axh(:,1)    = [0 Tend]';
axh(:,2)    = gradient(vxh(:,2),vxh(:,1));
fxh(:,1)    = [0 Tend]';
fxh(:,2)    = [2000 2000]';
sim('vehicle_lat_dyn_sim');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% post processing
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(1)
plot(XG,YG,'-','displayname','veh'); grid on; hold on; box on;
%quiver(-YG(1:10:end),XG(1:10:end),-sin(psi(1:10:end)+beta(1:10:end)),cos(psi(1:10:end)+beta(1:10:end)),'k');
%axis equal;
title('vehicle trajectory');
xlabel('X [m]');
ylabel('Y [m]');

figure(2)
subplot(221)
plot(tout,psip*180/pi); grid on; hold on; box on;
xlabel('time [s]');
ylabel('yaw rate [deg/s]');
subplot(222)
plot(tout,beta*180/pi); grid on; hold on; box on;
xlabel('time [s]');
ylabel('sideslip angle [deg/s]');
subplot(223)
plot(tout,ay); grid on; hold on; box on;
xlabel('time [s]');
ylabel('lat acc [m/s^2]');
subplot(224)
plot(tout,vy); grid on; hold on; box on;
xlabel('time [s]');
ylabel('lat speed [m/s]');


figure(3);
subplot(221)
plot(delta*180/pi,psip*180/pi); grid on; hold on; box on;
xlabel('steer [deg]');
ylabel('yaw rate [deg/s]');
subplot(222)
plot(ay,delta*180/pi); grid on; hold on; box on;
ylabel('steer [deg]');
xlabel('lat acc [m/s^2]');
subplot(223)
plot(delta*180/pi,beta*180/pi); grid on; hold on; box on;
xlabel('steer [deg]');
ylabel('sideslip angle [deg]');
subplot(224)
plot(beta*180/pi,psip*180/pi); grid on; hold on; box on;
xlabel('sideslip angle [deg]');
ylabel('yaw rate [deg/s]');





% muf = 1;
% mur = 1;
% a = (0:0.1:45)/180*pi;
% Bf = bf/muf;
% Br = br/mur;
% Df = muf*df*Fzf;
% Dr = mur*dr*Fzr;
% Fyf = Df*sin(Cf*atan(Bf.*a-Ef*(Bf*a-atan(Bf*a))));
% Fyr = Dr*sin(Cr*atan(Br.*a-Er*(Br*a-atan(Br*a))));
% 
% figure();
% plot(a*180/pi,Fyf/Fzf,'displayname','front'); hold on; grid on; box on;
% plot(a*180/pi,Fyr/Fzr,'displayname','rear'); hold on; grid on; box on;
% 


