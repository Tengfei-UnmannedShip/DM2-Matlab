% close all
%% ��ͼ����show ships' positions at every moment

figure
ha = MarginEdit(3,2,[.05  0.05],[.05  0.05],[0.05  0.05],1);
for fig=1:6
    
    switch fig
        case 1
            k=1;
        case 2
            k=500;
        case 3
            k=1000;
        case 4
            k=1500;
        case 5
            k=2000;
        case 6
            k=2500;
    end
    
%     subplot(3,2,fig);
%     hold on;
    axes(ha(fig)); 
    %WTF:���������ĳ�ʼλ��
    drawShip0(pos1(1,:),c1(1),1,400);
    drawShip0(pos2(1,:),c2(1),2,400);
    drawShip0(pos3(1,:),c3(1),3,400);
    drawShip0(pos4(1,:),c4(1),4,400);
    
    %WTF:���������Ľ���λ��
    drawShip(pos1(k,:),c1(k),1,400);
    drawShip(pos2(k,:),c2(k),2,400);
    drawShip(pos3(k,:),c3(k),3,400);
    drawShip(pos4(k,:),c4(k),4,400);
    
    %WTF:���������ĺ���ͼ
    plot(pos1(1:k,1),pos1(1:k,2),'r-');
    plot(pos2(1:k,1),pos2(1:k,2),'g-');
    plot(pos3(1:k,1),pos3(1:k,2),'b-');
    plot(pos4(1:k,1),pos4(1:k,2),'k-');
    
    %WTF:������ͷ��Բ���ڱ�ʾ��ȫ��Χ
    circle(pos1(k,:),900/1852,1);
    circle(pos2(k,:),900/1852,2);
    circle(pos3(k,:),900/1852,3);
    circle(pos4(k,:),900/1852,4);
    
    
    grid on;
    axis on
    xlabel('\it n miles', 'Fontname', 'Times New Roman','FontSize',15);
    ylabel('\it n miles', 'Fontname', 'Times New Roman','FontSize',15);
    title(['t=',num2str(k),'s'], 'Fontname', 'Times New Roman','FontSize',15);
    box on;
end
% print('-djpeg', '-r300', 'figure1');

figure
ha = MarginEdit(1,1,[.05  0.05],[.05  0.05],[0.05  0.05],1);
axes(ha(1)); 
k=1500;
%WTF:���������ĳ�ʼλ��
drawShip1(pos1(1,:),c1(1),1,200,1.5);
drawShip1(pos2(1,:),c2(1),2,200,1.5);
drawShip1(pos3(1,:),c3(1),3,200,1.5);
drawShip1(pos4(1,:),c4(1),4,200,1.5);
%WTF:���������Ľ���λ��
drawShip1(pos1(k,:),c1(k),1,200,1.5);
drawShip1(pos2(k,:),c2(k),2,200,1.5);
drawShip1(pos3(k,:),c3(k),3,200,1.5);
drawShip1(pos4(k,:),c4(k),4,200,1.5);

%WTF:���������ĺ���ͼ
plot(pos1(1:k,1),pos1(1:k,2),'r-','LineWidth',2);
plot(pos2(1:k,1),pos2(1:k,2),'g-','LineWidth',2);
plot(pos3(1:k,1),pos3(1:k,2),'b-','LineWidth',2);
plot(pos4(1:k,1),pos4(1:k,2),'k-','LineWidth',2);

%WTF:������ͷ��Բ���ڱ�ʾ��ȫ��Χ
circle2(pos1(k,:),600/1852,1,2);
circle2(pos2(k,:),600/1852,2,2);
circle2(pos3(k,:),600/1852,3,2);
circle2(pos4(k,:),600/1852,4,2);
grid on;
box on;
set(gca,'LineWidth',1.5)
% xlabel('\it n miles', 'Fontname', 'Times New Roman','FontSize',15);
% ylabel('\it n miles', 'Fontname', 'Times New Roman','FontSize',15);
% title(['t=',num2str(k),'s'], 'Fontname', 'Times New Roman','FontSize',15);
% print('-djpeg', '-r300', 'figure2');

