function scores = H2TS_fitness(x,TaskCell,RobotMat,AgentMat,depot,alpha)
%x is cell matrix, its element is a row vector
global TaskNumMat;
[m,n]=size(TaskNumMat);
scores = zeros(size(x,1),1);
for id = 1:size(x,1)
    %recover from row vector to cell matrix,p->pp
    p = x{id};  
    pCell = vec2cell(p',TaskNumMat);
    
    robotLatestTime = zeros(m,1);
    agentLatestTime = zeros(n,1);
    robotLatestLoc = RobotMat(:,1:2);
    agentLatestLoc = AgentMat(:,1:2);
    
    largestRankMat = zeros(m,n);
    completeTimeCell = cell(m,n);
    for i=1:m
        for j=1:n
            if TaskNumMat(i,j)~=0
                largestRankMat(i,j)=max(pCell{i,j});
                completeTimeCell{i,j}=zeros(TaskNumMat(i,j),1);
            end
        end
    end
    maxRank=max(max(largestRankMat));
    
    for sb=1:maxRank
        for i=1:m
            for j=1:n
                vec=pCell{i,j};
                index=find(vec==sb, 1);
                if ~isempty(index)
                    Tasks=TaskCell{i,j};
                    task=Tasks(index,:);
                    time1=robotLatestTime(i,1)+getDistance(robotLatestLoc(i,1),robotLatestLoc(i,2),task(1,4),task(1,5));
                    time2=agentLatestTime(j,1)+alpha*getDistance(agentLatestLoc(j,1),agentLatestLoc(j,2),task(1,4),task(1,5));
                    tempVec=completeTimeCell{i,j};
                    tempVec(index,1)=max(time1,time2)+task(1,6);
                    completeTimeCell{i,j}=tempVec;
                    robotLatestTime(i,1)=tempVec(index,1);
                    agentLatestTime(j,1)=tempVec(index,1);
                    robotLatestLoc(i,:)=task(1,4:5);
                    agentLatestLoc(j,:)=task(1,4:5);
                end
            end
        end                        
    end
    save('completeTimeCell.mat','completeTimeCell');
    
    for i=1:m
        robotLatestTime(i,1)=robotLatestTime(i,1)+getDistance(depot(1,1),depot(1,2),robotLatestLoc(i,1),robotLatestLoc(i,2));
    end    
    scores(id,1) = max(robotLatestTime(:,1));
end