function children = H2TS_mutate(parentsIndex,~,~,~, ~, ~,thisPopulation,~)
global TaskNumMat;
[m,n]=size(TaskNumMat);
totalTaskSum=sum(sum(TaskNumMat));
%   The arguments to the function are
%     PARENTS: Parents chosen by the selection function. index
%     OPTIONS: Options created from OPTIMOPTIONS
%     NVARS: Number of variables
%     FITNESSFCN: Fitness function
%     STATE: State structure used by the GA solver
%     THISSCORE: Vector of scores of the current population
%     THISPOPULATION: Matrix of individuals in the current population
%     MUTATIONRATE: Rate of mutation

children = cell(length(parentsIndex),1); %output children number = parent number
for id=1:length(parentsIndex)
    parent = thisPopulation{parentsIndex(id)};
    parentCell = vec2cell(parent',TaskNumMat);
    largestRankMat=zeros(m,n);
    for i=1:m
        for j=1:n
            if TaskNumMat(i,j)~=0
                largestRankMat(i,j)=max(parentCell{i,j});
            end
        end
    end
    
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
    mutateLinearLocation = randi([1 totalTaskSum]);
    mutateLocation = taskIndexMat(mutateLinearLocation,:);
    
    r = mutateLocation(1,1);
    c = mutateLocation(1,2);
    d = mutateLocation(1,3);
    mutatedValue = parent(mutateLinearLocation);
    maxRowAndColumnRank = max(max(largestRankMat(r,:)),max(largestRankMat(:,c)));
    %mutateSpace:{1,2,...,maxRank,maxRank+1}\{rrc}
    mutateSpace = 1:maxRowAndColumnRank+1;
    mutateSpace = mutateSpace(mutateSpace~=mutatedValue);
    parent(mutateLinearLocation) = mutateSpace(randi([1 length(mutateSpace)]));
    %verification
    childCell = vec2cell(parent',TaskNumMat);
    childCell = feasibleValidation(childCell,TaskNumMat,mutateLocation);
    children{id}=cell2vec(childCell,TaskNumMat)'; %row vector
end
