clear;
clc;
load("../Instance.mat");

robotNum=size(RobotMat,1);
agentNum=size(AgentMat,1);
subtaskNum=size(TaskMat,1);

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

%% Variable Definition and counting.
%Cmax,S_ijk,X,Y,U_ijk,V_ijk
% count tasks, continuous variables and binary variables.

XNumVec = RobotTaskNumVec.^2;
YNumVec = AgentTaskNumVec.^2;

taskNumVec = zeros(robotNum,1);
taskNumVec(:,1) = sum(BlockNumMat,2);

vecLen = 1+3*subtaskNum+sum(XNumVec)+sum(YNumVec);
M=10000;

%% constraints
Aineq=[];
bineq=[];
Aeq=[];
beq=[];

%constraint (1)
A=zeros(subtaskNum,vecLen);
b=zeros(subtaskNum,1);
A(:,1)=-1;
A(:,2:subtaskNum+1)=eye(subtaskNum);
b(:,1)=-TaskMat(:,6)-getDistance(TaskMat(:,4),TaskMat(:,5),depot(1,1)*ones(subtaskNum,1),depot(1,2)*ones(subtaskNum,1));
Aineq=[Aineq;A];
bineq=[bineq;b];

%constraint (2).
A=zeros(robotNum,vecLen);
b=ones(robotNum,1);
for i=1:robotNum
    indices=find(TaskMat(:,1)==i);
    for j=1:length(indices)
        idx = indices(j,1);
        A(i,1+subtaskNum+idx)=1;
    end
end
Aeq=[Aeq;A];
beq=[beq;b];

%constraint (3)
A=zeros(agentNum,vecLen);
b=ones(agentNum,1);
for i=1:agentNum
    indices=find(TaskMat(:,2)==i);
    for j=1:length(indices)
        idx = indices(j,1);
        A(i,1+2*subtaskNum+idx)=1;
    end
end
Aeq=[Aeq;A];
beq=[beq;b];

%constraint (4)(5)
for i=1:robotNum
    task = RobotTaskCell{i,1};
    num = RobotTaskNumVec(i,1);
    %regard as a transportation problem.
    %each constraint is a  [2*taskNum  *  taskNum^2] matrix
    A=zeros(2*num,vecLen);
    b=ones(2*num,1);
    
    if i==1
        tempXNum=0;
    else
        tempXNum=sum(XNumVec(1:i-1,1));
    end
    
    for j=1:num
        colIndex = 1+3*subtaskNum+tempXNum+(j-1)*num;
        A(j,colIndex +1:colIndex+num)=ones(1,num);
        A(j,colIndex + j)=0; % delete self-loop
        A(j,colIndex + 1)=0; % delete dummy task
        
        % last task
        if j>1
            finalIndex = find(ismember(TaskMat,task(j,:),'rows')',1);
            finalIndex = 1+subtaskNum+finalIndex;
            A(j,finalIndex)=1;
        end
        
        A(num+1:2*num,colIndex +1:colIndex+num)=eye(num);
        A(num+j,colIndex+j)=0; % delete self-loop
        
    end
    
    A(num+1,:)=0; % delete dummy task
    b(num+1,:)=0; % delete dummy task
    Aeq=[Aeq;A];
    beq=[beq;b];
    
end

%constraint (6)(7)
for i=1:agentNum
    task = AgentTaskCell{i,1};
    num = AgentTaskNumVec(i,1);
    %regard as a transportation problem.
    %each constraint is a  [2*taskNum  *   taskNum^2] mRobotix
    A=zeros(2*num,vecLen);
    b=ones(2*num,1);
    
    if i==1
        tempYNum=0;
    else
        tempYNum=sum(YNumVec(1:i-1,1));
    end
    
    for j=1:num
        colIndex = 1+3*subtaskNum+sum(XNumVec)+tempYNum+(j-1)*num;
        A(j,colIndex +1:colIndex+num)=ones(1,num);
        A(j,colIndex + j)=0; % delete self-loop
        A(j,colIndex + 1)=0; % delete dummy task
        
        % last task
        if j>1
            finalIndex = find(ismember(TaskMat,task(j,:),'rows')',1);
            finalIndex = 1+2*subtaskNum+finalIndex;
            A(j,finalIndex)=1;
        end
        
        A(num+1:2*num,colIndex +1:colIndex+num)=eye(num);
        A(num+j,colIndex+j)=0; % delete self-loop
    end
    A(num+1,:)=0; % delete dummy task
    b(num+1,:)=0; % delete dummy task
    
    Aeq=[Aeq;A];
    beq=[beq;b];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% constraint (8)
for i=1:robotNum
    task = RobotTaskCell{i,1};
    num = RobotTaskNumVec(i,1);
    
    if i==1
        tempXNum=0;
    else
        tempXNum=sum(XNumVec(1:i-1,1));
    end
    
    for j=1:num
        colIndex = 1+3*subtaskNum+tempXNum+(j-1)*num;
        A=zeros(num,vecLen);
        b=zeros(num,1);
        priorTask = task(j,:);
        
        if j==1 %dummy task
            index1=0;
        else
            index1=find(ismember(TaskMat,priorTask,'rows')',1);
        end
                
        for k=1:num
            %skip self loop and dummy task.k cannot be a dummy task.
            if k~=j && k~=1
                latterTask = task(k,:);                
                index2=find(ismember(TaskMat,latterTask,'rows')',1);
                tempRow = zeros(1,vecLen);
                if  index1~=0
                    tempRow(1,1+index1) = 1;
                end
                tempRow(1,1+index2) = -1;
                tempRow(1,colIndex +k) = M;
                %fprintf("%d,%d\n",index1,index2);
                A(k,:) = tempRow;
                b(k,1) = M-priorTask(1,6)-getDistance(priorTask(1,4),priorTask(1,5),latterTask(1,4),latterTask(1,5));
            end
        end
        Aineq=[Aineq;A];
        bineq=[bineq;b];
    end
end

% constraint (9)
for i=1:agentNum
    task = AgentTaskCell{i,1};
    num = AgentTaskNumVec(i,1);
    
    if i==1
        tempYNum=0;
    else
        tempYNum=sum(YNumVec(1:i-1,1));
    end
    
    for j=1:num
        colIndex = 1+3*subtaskNum+sum(XNumVec)+tempYNum+(j-1)*num;
        A=zeros(num,vecLen);
        b=zeros(num,1);
        priorTask = task(j,:);
        
        if j==1
            index1=0;
        else
            index1=find(ismember(TaskMat,priorTask,'rows')',1);
        end
        
        for k=1:num
            %skip self loop and dummy task
            if k~=j && k~=1
                latterTask = task(k,:);
                index2=find(ismember(TaskMat,latterTask,'rows')',1);
                %fprintf("%d,%d\n",index1,index2);
                tempRow = zeros(1,vecLen);
                if  index1~=0
                    tempRow(1,1+index1) = 1;
                end
                tempRow(1,1+index2) = -1;
                tempRow(1,colIndex +k) = M;
                A(k,:) = tempRow;
                b(k,1) = M-priorTask(1,6)-alpha*getDistance(priorTask(1,4),priorTask(1,5),latterTask(1,4),latterTask(1,5));
            end
        end
        Aineq=[Aineq;A];
        bineq=[bineq;b];
    end
end

%% Bounds
intcon=1+subtaskNum+1:vecLen;
lb = zeros(vecLen,1);
ub = inf(vecLen,1);
ub(1+subtaskNum+1:vecLen,1)=1;

%% Objective
f=zeros(vecLen,1);
f(1,1)=1;
%% MILP
opts = optimoptions('intlinprog','Display','iter','Heuristics','round-diving','IPPreprocess','none');
tic;
[solutionMILP,fval,exitflag,output] = intlinprog(f,intcon,Aineq,bineq,Aeq,beq,lb,ub,opts);
toc;
solutionMILP=int32(solutionMILP);
disp(solutionMILP);
fprintf('optimal makespan:%f\n',fval);
fprintf('robot number:%d,agent number:%d,task number:%f\n',robotNum,agentNum,subtaskNum);
fprintf('varible size:%d,   continuous variable: %d,   binary variable:%d\n',vecLen,1+subtaskNum,vecLen-1-subtaskNum);
fprintf('equality constraint size:%d\n',size(Aeq,1));
fprintf('inequality constraint size:%d\n',size(Aineq,1));

save('solutionMILP.mat','solutionMILP');

