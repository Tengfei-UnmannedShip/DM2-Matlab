function drawShip(pos,course,k,size)%WTF:k代表船号，不同的船号颜色不同
%WTF：船舶角度的转换
d=size/1852;
theta1=course-10;
theta2=course+10;
%WTF：船舶位置的转换，我的程序还涉及到直角坐标转换为极坐标
pos1=[pos(1)+d*sind(theta1) pos(2)+d*cosd(theta1)];
pos2=[pos(1)+d*sind(theta2) pos(2)+d*cosd(theta2)];
pos3=[pos(1)-d*sind(theta1) pos(2)-d*cosd(theta1)];
pos4=[pos(1)-d*sind(theta2) pos(2)-d*cosd(theta2)];
pos5=[pos(1)+1.3*d*sind(course) pos(2)+1.3*d*cosd(course)];

hold on;
if k==1
    plot([pos1(1) pos5(1)],[pos1(2) pos5(2)],'r-','LineWidth',1);
    plot([pos2(1) pos5(1)],[pos2(2) pos5(2)],'r-','LineWidth',1);
    plot([pos2(1) pos3(1)],[pos2(2) pos3(2)],'r-','LineWidth',1);
    plot([pos3(1) pos4(1)],[pos3(2) pos4(2)],'r-','LineWidth',1);
    plot([pos4(1) pos1(1)],[pos4(2) pos1(2)],'r-','LineWidth',1);
elseif k==2
    plot([pos1(1) pos5(1)],[pos1(2) pos5(2)],'g-','LineWidth',1);
    plot([pos2(1) pos5(1)],[pos2(2) pos5(2)],'g-','LineWidth',1);
    plot([pos2(1) pos3(1)],[pos2(2) pos3(2)],'g-','LineWidth',1);
    plot([pos3(1) pos4(1)],[pos3(2) pos4(2)],'g-','LineWidth',1);
    plot([pos4(1) pos1(1)],[pos4(2) pos1(2)],'g-','LineWidth',1);
elseif k==3
    plot([pos1(1) pos5(1)],[pos1(2) pos5(2)],'b-','LineWidth',1);
    plot([pos2(1) pos5(1)],[pos2(2) pos5(2)],'b-','LineWidth',1);
    plot([pos2(1) pos3(1)],[pos2(2) pos3(2)],'b-','LineWidth',1);
    plot([pos3(1) pos4(1)],[pos3(2) pos4(2)],'b-','LineWidth',1);
    plot([pos4(1) pos1(1)],[pos4(2) pos1(2)],'b-','LineWidth',1);
else
    plot([pos1(1) pos5(1)],[pos1(2) pos5(2)],'k-','LineWidth',1);
    plot([pos2(1) pos5(1)],[pos2(2) pos5(2)],'k-','LineWidth',1);
    plot([pos2(1) pos3(1)],[pos2(2) pos3(2)],'k-','LineWidth',1);
    plot([pos3(1) pos4(1)],[pos3(2) pos4(2)],'k-','LineWidth',1);
    plot([pos4(1) pos1(1)],[pos4(2) pos1(2)],'k-','LineWidth',1);
end