% close all;
figure grid minor
clear;
% d = importdata('/Users/lingqiujin/output/poseGT.csv');
d = importdata('/Users/lingqiujin/Data/iphone7_06_04_2019/rgbd/data/CameraTrajectory.txt');
% d = 9.805*d;
% d = d(1:3000,:);
d1 = [d(1:end-1,2),d(1:end-1,3),d(1:end-1,4)]';
d2 = [d(2:end,2),d(2:end,3),d(2:end,4)]';
length = sum(vecnorm(d2-d1))
% figure
% d = importdata('/Users/lingqiujin/output/poseGT.csv');
% d = 9.8*d;
% final = d(end,:);
% de = importdata('/Users/lingqiujin/work/mySLAM/dataOpt.txt');
% d =[d; de.*[1 1 1 1 1 1]+final];
p1 = plot3(d(:,2),d(:,3),d(:,4),'b','LineWidth',3);
grid minor;
% axis equal
hold on;
ps = plot3(d(1,2),d(1,3),d(1,4),'*','MarkerSize',10,'MarkerFaceColor','r');
hold on;
pend1 = plot3(d(end,2),d(end,3),d(end,4),'x','MarkerSize',10,'MarkerEdgeColor','r');


%%
% d = importdata('/Users/lingqiujin/output/vins.txt');
d = importdata('/Users/lingqiujin/Data/iphone7_06_04_2019/rgbd/data/vins_result_loop.csv');
d(:,2:4)=d(:,2:4)*roty(105);
d(:,2:4)=d(:,2:4)*rotz(-90);
d(:,2:4)=d(:,2:4)*roty(-10);
% d(:,2:4)=d(:,2:4)*rotx(10);
d1 = [d(1:end-1,2),d(1:end-1,3),d(1:end-1,4)]';
d2 = [d(2:end,2),d(2:end,3),d(2:end,4)]';
length = sum(vecnorm(d2-d1))

% hold on;
p2 = plot3(d(:,2),d(:,3),d(:,4),'g','LineWidth',3);

%%
hold on;
plot3(d(1,2),d(1,3),d(1,4),'*','MarkerSize',10,'MarkerFaceColor','r');
hold on;
pend2 = plot3(d(end,2),d(end,3),d(end,4),'x','MarkerSize',10,'MarkerEdgeColor','r');

%%
legend([p1 p2 ps pend1],{'Ground truth','VINS','Start Point','End Point'})
axis equal
grid minor;
