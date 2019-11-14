function drawShip0(pos,course,k,size)%WTF:k�����ţ���ͬ�Ĵ�����ɫ��ͬ
%WTF�������Ƕȵ�ת��
d=size/1852;
theta1=course-10;
theta2=course+10;
%WTF������λ�õ�ת�����ҵĳ����漰��ֱ������ת��Ϊ������
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
    text(pos(1)+0.2,pos(2)+0.2,['S_',num2str(k)],'Color','r');
elseif k==2
    plot([pos1(1) pos5(1)],[pos1(2) pos5(2)],'g-','LineWidth',1);
    plot([pos2(1) pos5(1)],[pos2(2) pos5(2)],'g-','LineWidth',1);
    plot([pos2(1) pos3(1)],[pos2(2) pos3(2)],'g-','LineWidth',1);
    plot([pos3(1) pos4(1)],[pos3(2) pos4(2)],'g-','LineWidth',1);
    plot([pos4(1) pos1(1)],[pos4(2) pos1(2)],'g-','LineWidth',1);
    text(pos(1)+0.2,pos(2)+0.2,['S_',num2str(k)],'Color','g');
elseif k==3
    plot([pos1(1) pos5(1)],[pos1(2) pos5(2)],'b-','LineWidth',1);
    plot([pos2(1) pos5(1)],[pos2(2) pos5(2)],'b-','LineWidth',1);
    plot([pos2(1) pos3(1)],[pos2(2) pos3(2)],'b-','LineWidth',1);
    plot([pos3(1) pos4(1)],[pos3(2) pos4(2)],'b-','LineWidth',1);
    plot([pos4(1) pos1(1)],[pos4(2) pos1(2)],'b-','LineWidth',1);
    text(pos(1),pos(2)+0.2,['S_',num2str(k)],'Color','b');
else
    plot([pos1(1) pos5(1)],[pos1(2) pos5(2)],'k-','LineWidth',1);
    plot([pos2(1) pos5(1)],[pos2(2) pos5(2)],'k-','LineWidth',1);
    plot([pos2(1) pos3(1)],[pos2(2) pos3(2)],'k-','LineWidth',1);
    plot([pos3(1) pos4(1)],[pos3(2) pos4(2)],'k-','LineWidth',1);
    plot([pos4(1) pos1(1)],[pos4(2) pos1(2)],'k-','LineWidth',1);
    text(pos(1)-0.4,pos(2)-0.4,['S_',num2str(k)],'Color','k');
end