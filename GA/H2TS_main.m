clear;
clc;

load('../Instance.mat');

global TaskNumMat
TaskNumMat=BlockNumMat;
totalTaskNum = sum(sum(TaskNumMat));
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

%% Customizing the Genetic Algorithm for a Custom Data Type
% By default the population is a matrix of type double,or logical in the case of binary strings.
% In order to use |ga| with a population of type cell array you must provide a
% creation function, a crossover function, and a mutation function that
% will work on your data type, e.g., a cell array.

%% Required Functions for the Traveling Salesman Problem
% An individual in the population for the traveling salesman
% problem is an ordered set, and so the population can easily be
% represented using a cell array. The custom creation function for the
% traveling salesman problem will create a cell array, say |P|, where each
% element represents an ordered set of cities as a permutation vector. That
% is, the salesman will travel in the order specified in |P{i}|. The creation
% function will return a cell array of size |PopulationSize|.

% The custom crossover function takes a cell array, the population, and
% returns a cell array, the children that result from the crossover.

% The custom mutation function takes an individual, which is an ordered set
% of cities, and returns a mutated ordered set.

%% Fitness Function
% We also need a fitness function for the traveling salesman problem. The
% fitness of an individual is the total distance traveled for an ordered
% set of cities. The fitness function also needs the distance matrix to
% calculate the total distance.
% |ga| will call our fitness function with just one argument |x|, but our
% fitness function has two arguments: |x|, |distances|. We can use an
% anonymous function to capture the values of the additional argument, the
% distances matrix. We create a function handle |FitnessFcn| to an
% anonymous function that takes one input |x|, but calls
% |traveling_salesman_fitness| with |x|, and distances. The variable,
% distances has a value when the function handle |FitnessFcn| is created,
% so these values are captured by the anonymous function.
%distances defined earlier

%% For the CROSS Problem
% An individual: a sequence matrix of size m*n ->reshaped to a row vector for fitness evaluation
% The Population: a cell array P
% The agents will travel in the order specified in |P{i}|
% machines in the row sequence, and jobs ihe column sequence

FitnessFcn = @(x) H2TS_fitness(x,BlockTaskCell,RobotMat,AgentMat,depot,alpha);

%% Genetic Algorithm
options = optimoptions(@ga, 'PopulationType', 'custom','InitialPopulationRange',[1;100]);

options = optimoptions(options,'CreationFcn',@H2TS_create, ...
    'CrossoverFcn',@H2TS_crossover, ...
    'MutationFcn',{@H2TS_mutate,0.5}, ...
    'PlotFcn', {@gaplotbestf},...
    'MaxGenerations',100,'PopulationSize',100, ...
    'FunctionTolerance',1,...
    'MaxStallGenerations',100,'UseVectorized',true);
%%
%注意matlab遗传算法的矩阵与向量的转换
%ga函数的第二个参数为向量的长度，因此需要矩阵和向量之间的数据类型转换
%个体用矩阵表示，但是要转化为向量才能传递到适应度函数中，传过去之后再变为矩阵
%矩阵是行向量
numberOfVariables = totalTaskNum;
bestCell=cell(5,1);
bestVec=zeros(5,1);
for i=1:5
    [bestCell{i,1},bestVec(i,1),reason,output] = ga(FitnessFcn,numberOfVariables,[],[],[],[],[],[],[],options);
end


[~,idx]=min(bestVec);
x=bestCell{idx(1,1),1};

bestIndividual = vec2cell(x{1}',TaskNumMat);

% analysis the optimal solution.
BlockNumMat;
BlockTaskCell;
RobotTaskSequence=cell(robotNum,1);
AgentTaskSequence=cell(agentNum,1);
for i=1:robotNum
    RobotTaskSequence{i,1}=[1];
end
for j=1:agentNum
    AgentTaskSequence{j,1}=[1];
end
StartTimeVec=zeros(totalTaskNum,1);
largestRank=zeros(robotNum,agentNum);
for i=1:robotNum
    for j=1:agentNum
        num=BlockNumMat(i,j);
        if num~=0
            largestRank(i,j)=max(bestIndividual{i,j});
        end
    end
end

linearIndividual=cell2vec(bestIndividual,TaskNumMat);
maxRank=max(max(largestRank));
for i=1:maxRank
    indices=find(linearIndividual==i);
    if ~isempty(indices)
        for j=1:length(indices)
            task = TaskMat(indices(j,1),:);
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
            TaskMat(indices(j,1),8)=ctime;            
        end
    end    
end

StartTimeVec=TaskMat(:,8);

save('solutionGA.mat','RobotTaskSequence','AgentTaskSequence','StartTimeVec');