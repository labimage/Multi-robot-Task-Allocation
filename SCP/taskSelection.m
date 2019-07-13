function outputSelect = taskSelection(BlockNumMat,TaskMat,RobotMat,AgentMat,alpha)
totalTaskNum=sum(sum(BlockNumMat));
robotNum=size(RobotMat,1);
agentNum=size(AgentMat,1);
varlen=totalTaskNum+1;
M=10000;
Aineq=[];
bineq=[];
%constraint (1)
A=[M*eye(totalTaskNum),-1*ones(totalTaskNum,1)];
b=zeros(totalTaskNum,1);
for i=1:totalTaskNum
    task=TaskMat(i,:);
    robotId=task(1,1);
    robot=RobotMat(robotId,:);
    b(i,1)=-robot(1,3)-getDistance(robot(1,1),robot(1,2),task(1,4),task(1,5))-task(1,6)+M;
end

indices = find(TaskMat(:,9)==1);
A(indices,:)=0;
b(indices,:)=0;

Aineq=[Aineq;A];
bineq=[bineq;b];

%constraint (2)
A=[M*eye(totalTaskNum),-1*ones(totalTaskNum,1)];
b=zeros(totalTaskNum,1);
for i=1:totalTaskNum
    task=TaskMat(i,:);
    agentId=task(1,2);
    agent=AgentMat(agentId,:);
    b(i,1)=-agent(1,3)-alpha*getDistance(agent(1,1),agent(1,2),task(1,4),task(1,5))-task(1,6)+M;
end

indices = find(TaskMat(:,9)==1);
A(indices,:)=0;
b(indices,:)=0;

Aineq=[Aineq;A];
bineq=[bineq;b];

%constraint (3)
A=zeros(robotNum,varlen);
b=zeros(robotNum,1);
for i=1:robotNum
    indices=and(TaskMat(:,1)==i,TaskMat(:,9)==0);
    indices=find(indices==1);
    if ~isempty(indices)
        vec=zeros(1,varlen);
        vec(1,indices)=1;
        A(i,:)=vec;
        b(i,1)=1;
    end    
end

Aineq=[Aineq;A];
bineq=[bineq;b];

%constraint (4)
A=zeros(agentNum,varlen);
b=zeros(agentNum,1);
for i=1:agentNum
    indices=and(TaskMat(:,2)==i,TaskMat(:,9)==0);
    indices=find(indices==1);
    if ~isempty(indices)
        vec=zeros(1,varlen);
        vec(1,indices)=1;
        A(i,:)=vec;
        b(i,1)=1;
    end    
end

Aineq=[Aineq;A];
bineq=[bineq;b];

%constraint (5)
indices = find(TaskMat(:,9)==0);
A=zeros(1,varlen);
A(1,indices)=1;
b=0;
UnfinishedRobotNum = find(RobotMat(:,4)==0);
UnfinishedAgentNum = find(AgentMat(:,4)==0);
b=min(size(UnfinishedRobotNum,1),size(UnfinishedAgentNum,1));

Aeq=A;
beq=b;

intcon=1:varlen-1;
lb = zeros(varlen,1);
ub = inf(varlen,1);
ub(1:varlen-1,1)=1;
%% Objective
f=zeros(varlen,1);
f(end,1)=1;
%% MILP
opts = optimoptions('intlinprog','Display','iter','Heuristics','round-diving','IPPreprocess','none');
[bestSolution,fval,exitflag,output] = intlinprog(f,intcon,Aineq,bineq,Aeq,beq,lb,ub,opts);
outputSelect=bestSolution(1:varlen-1,1);
outputSelect=int32(outputSelect);
end

