function linearLoc = rcd2linear(TaskNumMat,r,c,d)
if r==1
    if c==1
        linearLoc=d;
    else
        vec=TaskNumMat(r,:);
        linearLoc=sum(1:c-1)+d;
    end
else
    temp=sum(sum(TaskNumMat(1:r-1,:)));
    if c==1
        linearLoc=temp+d;
    else
        vec=TaskNumMat(r,:);
        linearLoc=temp+sum(1:c-1)+d;
    end
    
end


end

