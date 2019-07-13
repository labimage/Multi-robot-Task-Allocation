function RankCell = sequence2rank(RSeqCell,CSeqCell)
% prerequisites: Rseq and Cseq should be acyclic.
[m,n]=size(RSeqCell);
RankCell=cell(m,n);
TaskCell=cell(m,n);
TaskNumMat=zeros(m,n);
totalTaskSum = 0;
for i=1:m
    for j=1:n
        TaskNumMat(i,j)=length(RSeqCell{i,j});
        RankCell{i,j}=zeros(TaskNumMat(i,j),1);
        TaskCell{i,j}=ones(TaskNumMat(i,j),1);
    end
end
totalTaskSum=sum(sum(TaskNumMat));
k=0;
while (totalTaskSum>0)
    k=k+1;
    temp = 1;
    taskIndex = zeros(totalTaskSum,3);
    for i=1:m
        for j=1:n
            [r,~]=find(and(RSeqCell{i,j}==CSeqCell{i,j},RSeqCell{i,j}==1)==1);
            if size(r,1)~=0
                taskIndex(temp,:)=[i,j,r];
                temp=temp+1;
            end
        end
    end    
    sourceNum = sum(taskIndex(:,1)~=0);
    taskIndex=int32(taskIndex(1:sourceNum,:));
    if sourceNum==0
        break;
    else
        for i=1:sourceNum
            tempVec = RankCell{taskIndex(i,1),taskIndex(i,2)};
            tempVec(taskIndex(i,3),1)=k;
            RankCell{taskIndex(i,1),taskIndex(i,2)}=tempVec;
        end
        r=unique(taskIndex(:,1));
        c=unique(taskIndex(:,2));
        for i=1:length(r)
            for j=1:n
                RSeqCell{r(i,1),j}=RSeqCell{r(i,1),j}-1;
            end
        end
        for i=1:length(c)
            for j=1:m
                CSeqCell{j,c(i,1)}=CSeqCell{j,c(i,1)}-1;
            end
        end
        totalTaskSum=totalTaskSum-sourceNum;
    end
end
end %end of function