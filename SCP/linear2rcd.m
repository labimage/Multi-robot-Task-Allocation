function [r, c, d] = linear2rcd(taskNumMat,linearLoc)
[robotNum,agentNum]=size(taskNumMat);
sum=0;
for i=1:robotNum
    for j=1:agentNum
        if ~isempty(taskNumMat(i,j))
            for k=1:taskNumMat(i,j)
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

