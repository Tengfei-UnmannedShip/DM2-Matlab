% function feasible=plotShip(position,direction,map,robotHalfDiagonalDistance)
function plotShip(position,direction,ShipHalfDiagonalDistance)
% plot robot with specified configuration and check its feasibility
 corner1=position+ShipHalfDiagonalDistance*[sin(direction-pi/4) cos(direction-pi/4)];
 corner2=position+2*ShipHalfDiagonalDistance*[sin(direction) cos(direction)];
 corner3=position+ShipHalfDiagonalDistance*[sin(direction+pi/4) cos(direction+pi/4)];
 corner4=position+ShipHalfDiagonalDistance*[sin(direction-pi/4+pi) cos(direction-pi/4+pi)];
 corner5=position+ShipHalfDiagonalDistance*[sin(direction+pi/4+pi) cos(direction+pi/4+pi)];
 line([corner1(2);corner2(2);corner3(2);corner4(2);corner5(2);corner1(2)],[corner1(1);corner2(1);corner3(1);corner4(1);corner5(1);corner1(1)],'color','red','LineWidth',2);
%  if ~feasiblePoint(int16(corner1),map) || ~feasiblePoint(int16(corner2),map) || ~feasiblePoint(int16(corner3),map) || ~feasiblePoint(int16(corner4),map)
%      feasible=false;
%  else
%      feasible=true;
%  end