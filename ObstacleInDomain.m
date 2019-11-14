function feasible=ObstacleInDomain(map,row,col,extent)%extent为船舶领域的大小
[m,n]=size(map);
if (row-extent<=0||row+extent>=m||col-extent<=0||col+extent>=n)%length(map(1,:))
    feasible = 0;
else
    range=map((row-extent):(row+extent),(col-extent):(col+extent));
    %%%%%%%%%20171124更改%%%%%%%
    [r,c]=size(range);
    total=0;
    for ii=1:r
        for jj=1:c
            if range(ii,jj)==1
                total=total+1;
            end
        end
    end
%     total=sum(sum(range));
    if(total>0)
        feasible = 0;
    else
        feasible = 1;
    end
end
% end