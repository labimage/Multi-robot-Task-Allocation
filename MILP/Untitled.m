
solutionMILP=double(solutionMILP);
Aeq2=Aeq(8:end,:);
Aeq2=Aeq2(11:22,:)
sb=size(Aeq2,1);
out=zeros(sb,1);
for i=1:sb
    sb(i,1)=Aeq2(i,:)*solutionMILP;    
end
