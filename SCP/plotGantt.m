function [outputArg] = plotGantt(TimeCell,TaskIndexSequenceCell)
% input1: cell mRobotix. Each row denotes an agent time schedule.
% Each element is a N*6 mRobotix. 
% Travel time -, Travel time +, Idle time -, Idle time + ,Process time -, Process time +
% input2: cell mRobotix. Each row denotes a task index.
% Each element is a N*2 mRobotix.
% Add return to depot as the last task !!!!
barWidth = 2;
XLength = 400;
YLength = 8;
axis([0,XLength,0,YLength]);
set(gca,'xtick',0:20:XLength);
set(gca,'ytick',0:1:YLength);
xlabel('Time/s','FontName','Î¢ÈíÑÅºÚ','Color','k','FontSize',16);
ylabel('Robot/ID','FontName','Î¢ÈíÑÅºÚ','Color','k','FontSize',16,'Rotation',90);
%title('Optimal Task Scheduling for Robots','fontname','Î¢ÈíÑÅºÚ','Color','k','FontSize',16);
RGBcolors=[0.9,0.9,0.9;1,1,1;0.4,0.4,0.4];
ColorMat = [1 0 0;0 1 0;0 0 1;1 1 0;1 0 1;1 1 0];
len = size(TimeCell,1); 
for i=1:len
    timeMat = TimeCell{i,1};
    taskIndices = TaskIndexSequenceCell{i,1};
    rowSize = size(timeMat,1);
    for j=1:rowSize
        timeRow = timeMat(j,:);
        index = taskIndices(j,:);
        for k=1:3            
            rec=zeros(1,4);
            rec(1,1) = timeRow(1,2*k-1);
            rec(1,2) = i-0.3;
            rec(1,3) = timeRow(1,2*k)-timeRow(1,2*k-1);
            rec(1,4) = 0.6;            
            rectangle('Position',rec,'LineWidth',0.5,'LineStyle','-','FaceColor',RGBcolors(k,:));
            if  k==3
                txt=sprintf('t(%d,%d,%d)',index(1,1),index(1,2),index(1,3));
                text(timeRow(1,2*k-1),i+0.5,txt,'FontWeight','Bold','FontSize',10);
                if index(1,1)~=0
                rectangle('Position',rec,'LineWidth',0.5,'LineStyle','-','FaceColor',ColorMat(index(1,1),:));
                end
            end
        end
    end    
end
end

