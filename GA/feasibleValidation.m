function rankCell = feasibleValidation(inputCell,taskNumMat,Locations)
%Locations: many rows means many mutated subtasks.
[m,n]=size(inputCell);
totalTaskNum=sum(sum(taskNumMat));
%linear order of all subtasks.
LinearTaskRankVec=cell2vec(inputCell,taskNumMat);
LinearMutatedIndices=zeros(size(Locations,1),1);

for i=1:size(Locations,1)
    if Locations(i,1)==1
        if Locations(i,2)==1
            LinearMutatedIndices(i,1)=Locations(i,3);
        else
            LinearMutatedIndices(i,1)=sum(taskNumMat(Locations(i,1),1:Locations(i,2)-1))+Locations(i,3);
        end
    else
        temp = sum(sum(taskNumMat(1:Locations(i,1)-1,:)));
        if Locations(i,2)==1
            LinearMutatedIndices(i,1)=temp+Locations(i,3);
        else
            LinearMutatedIndices(i,1)=temp+sum(taskNumMat(Locations(i,1),1:Locations(i,2)-1))+Locations(i,3);
        end
    end            
end
    
largestRankMat=zeros(m,n);
for i=1:m
    for j=1:n
        if taskNumMat(i,j)~=0
            largestRankMat(i,j)=max(inputCell{i,j});
        end
    end
end
maxRank=max(max(largestRankMat));

temp=1;
tempIndices=1:totalTaskNum;
newLinear = zeros(totalTaskNum,1);
for i=1:maxRank
    rowIndices=find(LinearTaskRankVec==i);
    if  ~isempty(rowIndices)
        repeated = find(LinearTaskRankVec(LinearMutatedIndices',1)==i, 1);
        if ~isempty(repeated)
            nRepeated=size(repeated);
            for k=1:nRepeated
                newLinear(LinearMutatedIndices(repeated(k,1),1),1)=temp;
                rowIndices=rowIndices(rowIndices~=LinearMutatedIndices(repeated(k,1),1));
                temp=temp+1;
            end
            if ~isempty(rowIndices)
                newLinear(rowIndices,1)=tempIndices(temp:temp+size(rowIndices,1)-1)';
                temp=temp+length(rowIndices);
            end
        else
            newLinear(rowIndices,1)=tempIndices(temp:temp+size(rowIndices,1)-1)';
            temp=temp+length(rowIndices);
        end
    end
end

tempRankCell=vec2cell(newLinear,taskNumMat);
[RSeq,CSeq]=rank2sequence(tempRankCell);
rankCell=sequence2rank(RSeq,CSeq);
end

