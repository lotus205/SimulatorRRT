% dati traiettoria di riferimento
x = [0 20 30 60 85 110 140 1000]';
y = [0 0 0 3.5 3.5 0 0 0]';
xx = [0:.1:x(end)]';
yy = interp1(x,y,xx);
x = xx;
y = yy;
dx = diff(x);
dy = diff(y);
dx = [0; dx];
dy = [0; dy];
for ii=1:length(x)
s(ii,1) = sum(sqrt(dx(1:ii).^2+dy(1:ii).^2));
end
ss = [0:.1:s(end)]';
xx = interp1(s,x,ss);
yy = interp1(s,y,ss);
traiettoria.s = ss;
traiettoria.x = xx;
traiettoria.y = yy;

figure(1);
plot(traiettoria.x,traiettoria.y,'-b',...
    'displayname','ref'); 
grid on; hold on; box on;
%axis equal

% dati pilota
% Steering wheel ratio
TauSW = 18;
% path follower data
driver.t   = 1.5; % [s]
driver.L0  = 0.5; % [m]
driver.kP  = .8;
driver.kD  = 0.005;
driver.Tau = 0.1;
