function xoverKids  = H2TS_crossover(parents,~,~,~,~,thisPopulation)
%   FITNESSFCN,THISSCORE,THISPOPULATION) crossovers PARENTS to produce
%   the children XOVERKIDS.
%
%   The arguments to the function are
%     PARENTS: Parents chosen by the selection function. index
%     OPTIONS: Options created from OPTIMOPTIONS
%     NVARS: Number of variables
%     FITNESSFCN: Fitness function
%     STATE: State structure used by the GA solver
%     THISSCORE: Vector of scores of the current population
%     THISPOPULATION: Matrix of individuals in the current population
global TaskNumMat;
[m,n]=size(TaskNumMat);
%Crossover combines two individuals, or parents, to form a newchild, for the next generation
nKids = length(parents)/2;
%fprintf("Total crossover is %d",nKids);
xoverKids = cell(nKids,1);
index = 1;
totalTaskSum=sum(sum(TaskNumMat));

for id=1:nKids
    parent1 = thisPopulation{parents(index)};
    parent2 = thisPopulation{parents(index+1)};
    index = index + 2;
       
    crossProb = rand;    
    crossNum = int32(totalTaskSum*crossProb);
    if crossNum==0
        crossNum =1;
    end
    crossLinearIndices = randperm(totalTaskSum);
    crossLinearIndices = crossLinearIndices(1:crossNum);
    
    % generate subtask index matrix.
    taskIndexMat = zeros(totalTaskSum,3);
    temp = 1;
    for i=1:m
        for j=1:n
            if TaskNumMat(i,j)~=0
                for k=1:TaskNumMat(i,j)
                    taskIndexMat(temp,:)=[i j k];
                    temp = temp+1;
                end
            end
        end
    end    
    crossLocations = taskIndexMat(crossLinearIndices,:);
    
    %crossover
    child = parent1;
    child(crossLinearIndices) = parent2(crossLinearIndices);
    
    childCell = vec2cell(child',TaskNumMat);
    childCell = feasibleValidation(childCell,TaskNumMat,crossLocations);
    xoverKids{id} = cell2vec(childCell,TaskNumMat)';
end
