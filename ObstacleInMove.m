function feasible=ObstacleInMove(map,position_row,position_col,nextposition_row,nextposition_col,HalfofShip)
% clear all;close all;clc
% background=zeros(512,512);
% background(:,140:150)=1;
% background(200:250,140:150)=0;
% background(:,340:350)=1;
% background(300:350,340:350)=0;
% background(290:300,240:340)=1;
% map=background;
% position_row=350;position_col=240;
% nextposition_row=380;nextposition_col=200;
% HalfofShip=5;
total=0;
if (nextposition_row-HalfofShip<=0||nextposition_row+HalfofShip>=length(map(:,1))||nextposition_col-HalfofShip<=0||nextposition_col+HalfofShip>=length(map(1,:))) ...
        ||(position_row-HalfofShip<=0||position_row+HalfofShip>=length(map(:,1))||position_col-HalfofShip<=0||position_col+HalfofShip>=length(map(1,:)))
    feasible = 0;
else
    if (nextposition_col > position_col)
        num = ceil(sqrt((position_row-nextposition_row)^2+(position_col-nextposition_col)^2)/(HalfofShip*2));
        for ii = 1:num
            x = floor(position_col+ii*(nextposition_col-position_col)/num);
            y = floor(position_row+ii*(nextposition_row-position_row)/num);
            if ObstacleInDomain(map,y,x,HalfofShip)==0
                total=total+1;
            end
        end
    else
        num = ceil(sqrt((position_row-nextposition_row)^2+(position_col-nextposition_col)^2)/(HalfofShip*2));
        for ii = 1:num
            x = floor(nextposition_col+ii*(position_col-nextposition_col)/num);
            y = floor(nextposition_row+ii*(position_row-nextposition_row)/num);
            if ObstacleInDomain(map,y,x,HalfofShip)==0
                total=total+1;
            end
        end
    end
    if(total>0)
        feasible = 0;
    else
        feasible = 1;
    end
end
% imshow(map);
end