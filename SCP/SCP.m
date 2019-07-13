clear;
clc;

load('../Instance.mat');

totalTaskNum = sum(sum(BlockNumMat));
robotNum=size(RobotMat,1);
agentNum=size(AgentMat,1);
%% generate TaskCell TaskIndexCell TaskNumMat

BlockTaskCell=cell(robotNum,agentNum);
for i=1:robotNum
    for j=1:agentNum
        index = find(and(TaskMat(:,1)==i,TaskMat(:,2)==j));
        if ~isempty(index)
            BlockTaskCell{i,j} = TaskMat(index,:);
        else
            BlockTaskCell{i,j}=[];
        end
    end
end

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

%%
RobotTaskSequence = cell(robotNum,1);
AgentTaskSequence = cell(agentNum,1);

%% initialization
for i=1:robotNum
    RobotTaskSequence{i,1}=[1];
end
for j=1:agentNum
    AgentTaskSequence{j,1}=[1];
end
tic;
StartTimeVec=zeros(totalTaskNum);
iteration = 1;
scheduledTaskVec=TaskMat(:,9);
while(~isempty(find(scheduledTaskVec==0, 1)))
    %% MILP
    %variable:
    %1-totalTaskNum:task selection binary variable
    %totalTaskNum+1:makespan
    selectVec=taskSelection(BlockNumMat,TaskMat,RobotMat,AgentMat,alpha);  
    indices = find(selectVec==1);
            
    tasks = TaskMat(indices,:);  
    for i=1:size(tasks,1)
        task = tasks(i,:);
        robotId = task(1,1);
        agentId = task(1,2);
        robot = RobotMat(robotId,:);
        agent = AgentMat(agentId,:);
        index1 = find(ismember(RobotTaskCell{robotId,1},task,'rows')',1);
        index2 = find(ismember(AgentTaskCell{agentId,1},task,'rows')',1);
        RobotTaskSequence{robotId,1}=[RobotTaskSequence{robotId,1};index1];
        AgentTaskSequence{agentId,1}=[AgentTaskSequence{agentId,1};index2];
        time1=robot(1,3)+getDistance(robot(1,1),robot(1,2),task(1,4),task(1,5));
        time2=agent(1,3)+alpha*getDistance(agent(1,1),agent(1,2),task(1,4),task(1,5));
        ctime=max(time1,time2);
        %update robots' status
        robot = [task(1,4) task(1,5) ctime+task(1,6) 0];
        agent = [task(1,4) task(1,5) ctime+task(1,6) 0];
        RobotMat(robotId,:) = robot;
        AgentMat(agentId,:) = agent;
        %update task start times.
        TaskMat(indices(i,1),8)=ctime;
    end
        
    %mark finished tasks.
    TaskMat(indices,9)=1;
    
    %update robot finish flag.      
    for i=1:robotNum
        tasks=TaskMat(TaskMat(:,1)==i,:);
        vec = tasks(:,9);
        if isempty(find(vec==0, 1))
            RobotMat(i,4)=1;
        end
    end
    
    %update agent finish flag.
    for i=1:agentNum
        tasks=TaskMat(TaskMat(:,2)==i,:);
        vec = tasks(:,9);
        if isempty(find(vec==0, 1))
            AgentMat(i,4)=1;
        end
    end
        
    iteration = iteration+1;
    scheduledTaskVec=TaskMat(:,9);
end
toc;
StartTimeVec=TaskMat(:,8);

save('solutionSCP.mat','RobotTaskSequence','AgentTaskSequence','StartTimeVec');