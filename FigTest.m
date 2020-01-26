% close all
%% 绘图程序：show ships' positions at every moment
figure
%WTF:画出船舶的初始位置
drawShip0(ship(1).pos(1,:),ship(1).Course(1),1,400);
drawShip0(ship(2).pos(1,:),ship(2).Course(1),2,400);
drawShip0(ship(3).pos(1,:),ship(3).Course(1),3,400);
drawShip0(ship(4).pos(1,:),ship(4).Course(1),4,400);

%WTF:画出过往的航迹图
plot(ship(1).posData(:,1),ship(1).posData(:,2),'r-');

%WTF:画出船舶的结束位置
drawShip0(ship(1).OSWayPoint(end,:),ship(1).courseData(end),1,400);



grid on;
xlabel('\it n miles', 'Fontname', 'Times New Roman');
ylabel('\it n miles', 'Fontname', 'Times New Roman');
box on;

