close all;
clear;
d = importdata('/Users/lingqiujin/Q_MAC/work/mySLAM/data_noOpt.txt');

p1 = plot3(d(:,4),d(:,5),d(:,6),'b','LineWidth',3);
grid minor;
% axis equal
hold on;
ps = plot3(d(1,4),d(1,5),d(1,6),'*','MarkerSize',10,'MarkerFaceColor','r');
hold on;
pend1 = plot3(d(end,4),d(end,5),d(end,6),'x','MarkerSize',10,'MarkerEdgeColor','r');

d = importdata('/Users/lingqiujin/Q_MAC/work/mySLAM/data_Opt.txt');
hold on;
p2 = plot3(d(:,4),d(:,5),d(:,6),'g','LineWidth',3);
% grid minor;
hold on;
plot3(d(1,4),d(1,5),d(1,6),'*','MarkerSize',10,'MarkerFaceColor','r');
hold on;
pend2 = plot3(d(end,4),d(end,5),d(end,6),'x','MarkerSize',10,'MarkerEdgeColor','r');

legend([p1 p2 ps pend1],{'Dead Reckoning','Graph Optimizied','Start Point','End Point'})