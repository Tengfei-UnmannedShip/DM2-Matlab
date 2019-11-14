function [ curShipHeading ] = GetNextHeading( curShipHeading, goalHeading )
% ���ݵ�ǰ�����Ŀ�꺽�򣬼�����һ���ڵĺ���
% ÿ0.1s���ת��Ƕ�Ϊ1��

maxChange = 0.5;
difference = goalHeading - curShipHeading;
difference = (difference > 180)*(difference-360) + (difference <= 180)*difference;
difference = (difference < -180)*(difference+360) + (difference >= -180)*difference;

curShipHeading = curShipHeading + difference*(difference>-maxChange&&difference<maxChange);
curShipHeading = curShipHeading + (difference<-maxChange)*-maxChange + (difference>maxChange)*maxChange;

end

