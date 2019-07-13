function linearLoc = rcd2linear(taskNumMat,r,c,d)
if r==1
    if c==1
        linearLoc=d;
    else
        vec=taskNumMat(r,:);
        linearLoc=sum(1:c-1)+d;
    end
else
    temp=sum(sum(taskNumMat(1:r-1,:)));
    if c==1
        linearLoc=temp+d;
    else
        vec=taskNumMat(r,:);
        linearLoc=temp+sum(1:c-1)+d;
    end
    
end


end

