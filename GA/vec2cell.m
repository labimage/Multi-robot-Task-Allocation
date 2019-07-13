function outputCell = vec2cell(colVec,taskNumMat)
[m,n]=size(taskNumMat);
outputCell=cell(m,n);
if length(colVec)~=sum(sum(taskNumMat))
    disp("dimension error!");
    return;  
end

temp=1;
for i=1:m
    for j=1:n
        if taskNumMat(i,j)~=0
            outputCell{i,j}=colVec(temp:temp+taskNumMat(i,j)-1,1);
            temp=temp+taskNumMat(i,j);
        end
    end
end

end

