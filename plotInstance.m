clear;
clc;
load("Instance.mat");

figure('position',[400,10,1000,1000]);
axis([0 80 0 80]);
axis equal;
axis manual;
robotNum=size(RobotMat,1);
agentNum=size(AgentMat,1);
subtaskNum=size(TaskMat,1);

%% plot depot and stations
plot(StationMat(:,1),StationMat(:,2),'LineStyle','none','Color',[0.7 0.7 0.7],'marker','.','markersize',20);
hold on;
plot(depot(1,1),depot(1,2),'LineStyle','none','Color',[0 0 0],'marker','.','markersize',20);
hold on;

% plot(100,100,'LineStyle','none','Color',[1 1 1],'marker','.','markersize',1);
%pause(5);

%% plot cliques
for i=1:agentNum
    Clique = CliqueCell{i,1};
    x=Clique(:,1);
    y=Clique(:,2);
    dt = delaunayTriangulation(x,y);
    k = convexHull(dt);
    plot(x,y,'LineStyle','none',  'Color',[0.7 0.7 0.7],'marker','.','markersize',20);
    hold on;
    plot(x(k), y(k), 'Color',[0.9 0.9 0.9]);
    hold on;    
end

%pause(5);

%% plot robots
plot(RobotMat(:,1), RobotMat(:,2),'LineStyle','none','marker','o', 'Color',[0 0 0],'markersize',13,'MarkerFaceColor','k');
text(RobotMat(:,1)+1,RobotMat(:,2),'TR1,TR2,TR3');
hold on;
plot(AgentMat(:,1), AgentMat(:,2),'LineStyle','none','marker','s', 'Color',[0 0 0],'markersize',10,'MarkerFaceColor','k');
for i=1:agentNum
    txt=sprintf('SR%d',i);
    text(AgentMat(i,1)+1,AgentMat(i,2),txt);
end
hold on;

%pause(5);

%% plot tasks
ColorMat = [1 0 0;0 1 0;0 0 1;1 1 0;1 0 1;1 1 0];

for i=1:subtaskNum
    subtask=TaskMat(i,:);
    plot(subtask(1,4),subtask(1,5),'LineStyle','none','Color',ColorMat(subtask(1,1),:),'marker','.','MarkerFaceColor',ColorMat(subtask(1,1),:),'markersize',20);
    hold on;
    text(subtask(1,4)-0.7,subtask(1,5)-1,num2str(subtask(1,6)));
end



