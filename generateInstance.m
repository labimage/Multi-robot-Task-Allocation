clear;
clc;

robotNum=4;
agentNum=3;
taskNum = robotNum;
%velocity ratio
alpha = 2;

%% X-n110-k13 map
% VertexMat=zeros(101,3);
% VertexMat(1,:)=[1 50 50];
% for i=2:101
%     VertexMat(i,:)=[i randi([1 100]) randi([1 100])];
% end
VertexMat=[1 35 35
2 41 49
3 35 17
4 55 45
5 55 20
6 15 30
7 25 30
8 20 50
9 10 43
10 55 60
11 30 60
12 20 65
13 50 35
14 30 25
15 15 10
16 30 5
17 10 20
18 5 30
19 20 40
20 15 60
21 45 65
22 45 20
23 45 10
24 55 5
25 65 35
26 65 20
27 45 30
28 35 40
29 41 37
30 64 42
31 40 60
32 31 52
33 35 69
34 53 52
35 65 55
36 63 65
37 2 60
38 20 20
39 5 5
40 60 12
41 40 25
42 42 7
43 24 12
44 23 3
45 11 14
46 6 38
47 2 48
48 8 56
49 13 52
50 6 68
51 47 47
52 49 58
53 27 43
54 37 31
55 57 29
56 63 23
57 53 12
58 32 12
59 36 26
60 21 24
61 17 34
62 12 24
63 24 58
64 27 69
65 15 77
66 62 77
67 49 73
68 67 5
69 56 39
70 37 47
71 37 56
72 57 68
73 47 16
74 44 17
75 46 13
76 49 11
77 49 42
78 53 43
79 61 52
80 57 48
81 56 37
82 55 54
83 15 47
84 14 37
85 11 31
86 16 22
87 4 18
88 28 18
89 26 52
90 26 35
91 31 67
92 15 19
93 22 22
94 18 24
95 26 27
96 25 24
97 22 27
98 25 21
99 19 21
100 20 26
101 18 18];

VertexMat=VertexMat(:,2:3);
%the first vertex is depot
depot=VertexMat(1,:);
StationMat = VertexMat(2:end,:);
stationNum = size(StationMat,1);

%% Robots and Agents
%Station Allocation by K-means clustering
[idx,C] = kmeans(StationMat,agentNum);
%[idx,C] = kmeans(StationMat,agentNum,'Distance','cityblock');
CliqueCell = cell(agentNum,1);
%[x y time finished]
RobotMat = zeros(robotNum,4);
RobotMat(:,1:2) = repmat(depot,robotNum,1);
AgentMat = zeros(agentNum,4);
for i=1:agentNum
    CliqueCell{i,1} = StationMat(idx==i,:);
    temp = CliqueCell{i,1};
    n=randi(size(temp,1)); 
    AgentMat(i,1:2) = temp(n,:);
end

%% Generate Logistic Tasks.
% Each subtask is denoted by [x y processTime]
processTimeLB = 10;
processTimeUB = 20;
stationIndices = 1:stationNum;
TaskCell = cell(taskNum,1);
subtaskNum=0;
for i=1:taskNum    
    taskLength = round(normrnd(agentNum,0.8))+2;
    %taskLength = agentNum;
    %task = zeros(taskLength,3);
    c=randperm(numel(stationIndices));
    tempIndex=stationIndices(c(1:taskLength));
    %avoid repeated locations.
    stationIndices = stationIndices(c(taskLength+1:end));
    for j=1:taskLength
        %uniform distribution
        tempTime = processTimeLB + int32((processTimeUB-processTimeLB)*rand());
        task(j,:) = [StationMat(tempIndex(j),:),tempTime];        
    end        
    TaskCell{i,1}=task;
    subtaskNum=subtaskNum+taskLength;
end

%% Reorganize Tasks into a matrix including index and temporal-spatial information.
BlockTaskCell=cell(robotNum,agentNum);
BlockNumMat=zeros(robotNum,agentNum);

for i=1:robotNum
    task = TaskCell{i,1};
    for j=1:size(task,1)
        subtaskStation = task(j,1:2);
        rowIndex = and(StationMat(:,1)==subtaskStation(1,1),StationMat(:,2)==subtaskStation(1,2));
        rowIndex = find(rowIndex~=0);
        index = idx(rowIndex,1); %find which cliques this task belongs to
        BlockTaskCell{i,index} = [BlockTaskCell{i,index};task(j,:)];
        BlockNumMat(i,index)=BlockNumMat(i,index)+1;
    end
end

% [robotId,agentId,depthId,x,y,processTime,robotStartTime,agentStartTime,finished] linear order.
TaskMat = zeros(subtaskNum,9);
index=1;
for i=1:robotNum
    for j=1:agentNum
        temp=BlockNumMat(i,j);
        task=BlockTaskCell{i,j};
        if ~isempty(temp)
            for k=1:temp                
                TaskMat(index,:)=[i j k task(k,1) task(k,2) task(k,3) 0 0 0];
                index=index+1;
            end
        end
    end
end


save('Instance.mat','StationMat','depot','CliqueCell','TaskMat','BlockNumMat','RobotMat','AgentMat','alpha');

