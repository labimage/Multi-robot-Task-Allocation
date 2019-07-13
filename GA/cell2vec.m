function colVec = cell2vec(inputCell,taskNumMat)
[m,n]=size(inputCell);
totalTaskNum = sum(sum(taskNumMat));
colVec = zeros(totalTaskNum,1);
temp=1;
for i=1:m
    for j=1:n
        if taskNumMat(i,j)~=0     
            colVec(temp:temp+taskNumMat(i,j)-1,1)=inputCell{i,j};
            temp=temp+taskNumMat(i,j);
        end
    end
end

end

