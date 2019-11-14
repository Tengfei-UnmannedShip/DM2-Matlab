function [ curShipHeading ] = GetNextHeading( curShipHeading, goalHeading )
% 根据当前航向和目标航向，计算下一周期的航向
% 每0.1s最大转向角度为1°

maxChange = 0.5;
difference = goalHeading - curShipHeading;
difference = (difference > 180)*(difference-360) + (difference <= 180)*difference;
difference = (difference < -180)*(difference+360) + (difference >= -180)*difference;

curShipHeading = curShipHeading + difference*(difference>-maxChange&&difference<maxChange);
curShipHeading = curShipHeading + (difference<-maxChange)*-maxChange + (difference>maxChange)*maxChange;

end

