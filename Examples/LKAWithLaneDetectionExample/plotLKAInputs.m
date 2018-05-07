function plotLKAInputs(scenario,driverPath)

% Overlay driver path and actual path

h = figure('Color','white');
% Widen figure
figPosition = get(h,'Position');
figPosition(3) = figPosition(3) * 1.25;
set(h,'Position',figPosition);

ax1 = axes(h,...
    'Box','on',...
    'Position',[0,0.12,0.5,0.8]);
ax2 = axes(h,...
    'Box','on',...
    'Position',[0,0.12,1,2.75]);

plot(scenario,'Parent',ax1)
ylim([-50 200])

plot(scenario,'Parent',ax2)
xlim([20 145])
ylim([-20 30])

line(ax1,driverPath(:,1),driverPath(:,2),'Color','blue','LineWidth',1)
line(ax2,driverPath(:,1),driverPath(:,2),'Color','blue','LineWidth',1)
ax1.Title = text(0.5,0.5,'Road and driver path');
ax2.Title = text(0.5,0.5,'Driver asssisted at curvature change');


