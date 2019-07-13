function [r, c, d] = linear2rcd(TaskNumMat,linearLoc)
[robotNum,agentNum]=size(TaskNumMat);
sum=0;
for i=1:robotNum
    for j=1:agentNum
        if ~isempty(TaskNumMat(i,j))
            for k=1:TaskNumMat(i,j)
                sum=sum+1;
                if sum==linearLoc
                    r=i;
                    c=j;
                    d=k;
                    return;
                end
                
            end            
        end
        
    end
end


end

