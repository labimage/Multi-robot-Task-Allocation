clear;
clc;
load("../Instance.mat");

figure('position',[400,10,1000,1000]);
axis([0 80 0 80]);
axis equal;
axis manual;

robotNum=size(RobotMat,1);
agentNum=size(AgentMat,1);
subtaskNum=size(TaskMat,1);

ColorMat=zeros(robotNum,3);
ColorMat(1,:)=[1 0 0];
ColorMat(1,:)=[0 1 0];
ColorMat(1,:)=[0 0 1];

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

%pause(5);

%% plot optimal solutions
load('solutionSCP.mat');
%% Reorganize all tasks.
RobotTaskCell = cell(robotNum,1);
RobotTaskNumVec = zeros(robotNum,1);
for i=1:robotNum
    task = TaskMat(TaskMat(:,1)==i,:);
    robot=RobotMat(i,:);
    dummyTask = [i 0 0 robot(1,1) robot(1,2) 0 0 0 0];%dummy task comes first
    RobotTaskCell{i,1}=[dummyTask;task];
    RobotTaskNumVec(i,1) = size(task,1)+1;
end

AgentTaskCell = cell(agentNum,1);
AgentTaskNumVec = zeros(agentNum,1);
for i=1:agentNum
    task = TaskMat(TaskMat(:,2)==i,:);
    agent=AgentMat(i,:);
    dummyTask = [0 i 0 agent(1,1) agent(1,2) 0 0 0 0];%dummy task comes first
    AgentTaskCell{i,1}=[dummyTask;task];
    AgentTaskNumVec(i,1)=size(task,1)+1;
end

taskNumVec = zeros(robotNum,1);
taskNumVec(:,1) = sum(BlockNumMat,2);

%% plot routes.
hold on;
for i=1:robotNum
    robot = RobotMat(i,:);
    seq = RobotTaskSequence{i,1};
    task = RobotTaskCell{i,1};
    num = RobotTaskNumVec(i,1);
    for j=0:num
        if j==0
            priorLocation = robot(1,1:2);
        else
            priorLocation = task(seq(j,1),4:5);
        end
        if j~=num
            latterLocation = task(seq(j+1,1),4:5);
        else
            latterLocation = depot;
        end
        line([priorLocation(1,1),latterLocation(1,1)],[priorLocation(1,2),latterLocation(1,2)],'color',ColorMat(i,:),'linestyle','-');
        %pause(0.5);
    end
    %pause(1);
end

hold on;
%pause(5);

for i=1:agentNum
    agent = AgentMat(i,:);
    seq = AgentTaskSequence{i,1};
    task = AgentTaskCell{i,1};
    num = AgentTaskNumVec(i,1);
    for j=0:num-1
        if j==0
            priorLocation = agent(1,1:2);
        else
            priorLocation = task(seq(j,1),4:5);
        end
        latterLocation = task(seq(j+1,1),4:5);
        line([priorLocation(1,1),latterLocation(1,1)],[priorLocation(1,2),latterLocation(1,2)],'color',[0 0 0],'linestyle','--');
        %pause(0.5);
    end
    %pause(1);
end

%pause(3);

%% plot Gantt chart and statistics
totalRobotTravelTime = 0;
totalRobotWaitTime = 0;
totalAgentTravelTime = 0;
totalAgentWaitTime = 0;

solution=[0;StartTimeVec];

figure(2);
TimeCell=cell(robotNum+agentNum,1);
TaskIndexSequenceCell=cell(robotNum+agentNum,1);
priorTotalNum = 0;
for i=1:robotNum
    seq = RobotTaskSequence{i,1};
    task = RobotTaskCell{i,1};
    num = RobotTaskNumVec(i,1);    
    timeMat = zeros(num,6);    
    tempTime = 0;
    for j=1:num
        colIndex = 1+priorTotalNum;
        if j==num
            currentTask = task(1,:);
        else
            currentTask = task(seq(j+1,1),:);
        end
        priorTask = task(seq(j,1),:);
        travelTime = getDistance(priorTask(1,4),priorTask(1,5),currentTask(1,4),currentTask(1,5));
        
        if j==num
            startTime = 0;
        else
            index=find(ismember(TaskMat,currentTask,'rows')',1);
            startTime = solution(1+index,1);
        end
        processTime = currentTask(1,6);
        time1 = tempTime;
        time2 = time1+travelTime;
        time3 = time2;
        time4 = startTime;
        time5 = time4;
        time6 = time5+processTime;
        tempTime = time6;
        
        if startTime == 0
            time4 = time3;
            time5 = time4;
            time6 = time5;
        end
        
        timeMat(j,:) = [time1 time2 time3 time4 time5 time6];
        taskIndexMat(j,:) = currentTask(1,1:3);
        
        %statistics
        totalRobotTravelTime = totalRobotTravelTime+travelTime;
        totalRobotWaitTime = totalRobotWaitTime+time4-time3;
    end
    
    TimeCell{i,1}=timeMat;
    TaskIndexSequenceCell{i,1}=taskIndexMat;
    priorTotalNum = priorTotalNum + num-1;
end

for i=1:agentNum
    seq = AgentTaskSequence{i,1};
    task = AgentTaskCell{i,1};
    num = AgentTaskNumVec(i,1);
    
    timeMat = zeros(num-1,6);
    taskIndexMat = zeros(num-1,3);
    
    tempTime = 0;
    for j=1:num-1
        currentTask = task(seq(j+1,1),:);
        priorTask = task(seq(j,1),:);
        travelTime = alpha*getDistance(priorTask(1,4),priorTask(1,5),currentTask(1,4),currentTask(1,5));       
        index=find(ismember(TaskMat,currentTask,'rows')',1);
        startTime = solution(1+index,1);
        
        processTime = currentTask(1,6);
        time1 = tempTime;
        time2 = time1+travelTime;
        time3 = time2;
        time4 = startTime;
        time5 = time4;
        time6 = time5+processTime;
        tempTime = time6;
        
        if startTime == 0
            time4 = time3;
            time5 = time4;
            time6 = time5;
        end
        
        timeMat(j,:) = [time1 time2 time3 time4 time5 time6];
        taskIndexMat(j,:) = currentTask(1,1:3);
        
        %statistics
        totalAgentTravelTime = totalAgentTravelTime+travelTime;
        totalAgentWaitTime = totalAgentWaitTime+time4-time3;
    end
    
    TimeCell{robotNum+i,1}=timeMat;
    TaskIndexSequenceCell{robotNum+i,1}=taskIndexMat;
end

plotGantt(TimeCell,TaskIndexSequenceCell);

%pause(5);

%% plot Graph
figure(3);
G=digraph;

nodeName = cell(1,sum(RobotTaskNumVec)+agentNum);

for i=1:subtaskNum
    subtask=TaskMat(i,:);
    nodeName{1,i}=strcat('t(',num2str(subtask(1,1)),',',num2str(subtask(1,2)),',',num2str(subtask(1,3)),')');
end

for i=1:robotNum+agentNum
    nodeName{1,subtaskNum+i}=strcat('t(',num2str(i),',',num2str(0),')');
end

G = addnode(G,nodeName');

for i=1:robotNum
    seq=RobotTaskSequence{i,1};
    task=RobotTaskCell{i,1};
    num=RobotTaskNumVec(i,1);
    for j=1:num-1
        priorTask=task(seq(j,1),:);
        latterTask=task(seq(j+1,1),:);
        s=find(ismember(TaskMat,priorTask,'rows')',1);
        t=find(ismember(TaskMat,latterTask,'rows')',1);        
        G=addedge(G,s,t);
    end
end

for i=1:agentNum
    seq=AgentTaskSequence{i,1};
    task=AgentTaskCell{i,1};
    num=AgentTaskNumVec(i,1);
    for j=1:num-1
        priorTask=task(seq(j,1),:);
        latterTask=task(seq(j+1,1),:);
        s=find(ismember(TaskMat,priorTask,'rows')',1);
        t=find(ismember(TaskMat,latterTask,'rows')',1);        
        G=addedge(G,s,t);
    end
end

%plot(G);

% remove all dummy tasks.
H = rmnode(G,subtaskNum+1:subtaskNum+robotNum+agentNum);
plot(H);

[n,I] = toposort(H);

%analysis this DAG
deg = indegree(H);
sourceNum=size(find(deg==0),1);

criticalLength=0;
while (numnodes(H)~=0)
    deg = indegree(H);
    indices = find(deg==0);
    H=rmnode(H,indices);
    criticalLength=criticalLength+1;
end

makespan=0;
for i=1:robotNum
    mat=TimeCell{i,1};
    sb=mat(end,6);
    makespan=max(makespan,sb);    
end
disp(makespan);
disp(totalRobotTravelTime);
disp(totalAgentTravelTime);
disp(totalRobotWaitTime);
disp(totalAgentWaitTime);
disp(sourceNum);
disp(criticalLength);