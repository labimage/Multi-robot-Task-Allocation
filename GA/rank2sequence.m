function  [RSeqCell,CSeqCell]= rank2sequence(RankCell)
[m,n]=size(RankCell);
RSeqCell=cell(m,n);
CSeqCell=cell(m,n);
TaskNumMat=zeros(m,n);
for i=1:m
    for j=1:n
        TaskNumMat(i,j)=length(RankCell{i,j});
        RSeqCell{i,j}=zeros(TaskNumMat(i,j),1);
        CSeqCell{i,j}=ones(TaskNumMat(i,j),1);
    end
end

for i=1:m
    len=sum(TaskNumMat(i,:));
    row=zeros(1,len);
    temp = 1;
    for j=1:n
        row(1,temp:temp+TaskNumMat(i,j)-1)=RankCell{i,j}';
        temp=temp+TaskNumMat(i,j);
    end
    [~,index]=sort(row);
    vector=1:len;
    vector(index)=vector;    
    temp = 1;
    for j=1:n
        RSeqCell{i,j}=vector(1,temp:temp+TaskNumMat(i,j)-1)';
        temp=temp+TaskNumMat(i,j);        
    end
end

for j=1:n
    len=sum(TaskNumMat(:,j));
    column=zeros(len,1);
    temp = 1;
    for i=1:m
        column(temp:temp+TaskNumMat(i,j)-1,1)=RankCell{i,j};
        temp=temp+TaskNumMat(i,j);
    end
    [~,index]=sort(column);
    vector=(1:len)';
    vector(index)=vector;
    temp = 1;
    for i=1:m
        CSeqCell{i,j}=vector(temp:temp+TaskNumMat(i,j)-1,1);
        temp=temp+TaskNumMat(i,j);        
    end
end


end
