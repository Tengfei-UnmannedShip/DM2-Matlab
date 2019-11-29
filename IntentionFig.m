%% Intention Map��ͼ����
clear
clc
close all

load IntentionMapData
load pos
[X,Y]=meshgrid(-10:0.01:10,-10:0.01:10);  % ע���ͼ�ĳߴ磬ÿһ��������0.01��������ģ������Ҫ��������ӵļ��������µİ��ţ�����˼������õ�������
[m0,n0]=size(X);
map=zeros(m0,n0);
%
% figure
% ha = MarginEdit(3,3,[.05  0.05],[.05  0.05],[0.05  0.05],1);
for figNO=3
    %     for TSi=1:3
    figure
    if figNO>1
        k=300*(figNO-1); %ѡȡʱ�̷ֱ�Ϊ50��300*(figNO-1)
    else
        k=50;
    end
    
    %     subplot(3,2,fig);
    %     hold on;
    %     axes(ha(figNO));
    %WTF:����2~4�Ŵ���ͷ��Ԥ��켣
    
    %         OSInferMap=IntentionMap1;
    
    OSInferMap0=TotalMap(figNO).Ship2;
    OSInferMap1=OSInferMap0;
    OSInferMap0(OSInferMap0>50)=50;
    
            OSInferMap1(all(OSInferMap1==0,2),:) = [];%ȥ�������е�ȫ0��
    %         OSInferMap0(all(OSInferMap0==0,1),:) = [];%ȥ�������е�ȫ0��
    
    ss=pcolor(X,Y,OSInferMap0);  %����pcolor�Ĺٷ�ʾ��
    colorpan=ColorPanSet(1);
    colormap(colorpan);%����ɫ��
    set(ss, 'LineStyle','none');
    
    %     hold on
    %     %WTF:���������ĳ�ʼλ��
    %     drawShip0(pos1(1,:),c1(1),1,400);
    %     drawShip0(pos2(1,:),c2(1),2,400);
    %     drawShip0(pos3(1,:),c3(1),3,400);
    %     drawShip0(pos4(1,:),c4(1),4,400);
    %
    %     %WTF:���������Ľ���λ��
    %     drawShip(pos1(k,:),c1(k),1,400);
    %     drawShip(pos2(k,:),c2(k),2,400);
    %     drawShip(pos3(k,:),c3(k),3,400);
    %     drawShip(pos4(k,:),c4(k),4,400);
    %
    %     %WTF:���������ĺ���ͼ
    %     plot(pos1(1:k,1),pos1(1:k,2),'r-');
    %     plot(pos2(1:k,1),pos2(1:k,2),'g-');
    %     plot(pos3(1:k,1),pos3(1:k,2),'b-');
    %     plot(pos4(1:k,1),pos4(1:k,2),'k-');
    
    %     %WTF:������ͷ��Բ���ڱ�ʾ��ȫ��Χ
    %     circle(pos1(k,:),900/1852,1);
    %     circle(pos2(k,:),900/1852,2);
    %     circle(pos3(k,:),900/1852,3);
    %     circle(pos4(k,:),900/1852,4);
    
    
    %     grid on;
    axis equal
    xlabel('\it n miles', 'Fontname', 'Times New Roman','FontSize',15);
    ylabel('\it n miles', 'Fontname', 'Times New Roman','FontSize',15);
    title(['t=',num2str(k),'s'], 'Fontname', 'Times New Roman','FontSize',15);
    box on;
%     figure
%     OSInferMap0=TotalMap(figNO).Ship3;
%     OSInferMap1=OSInferMap0;
%     OSInferMap0(OSInferMap0>50)=50;
%     
%     ss=pcolor(X,Y,OSInferMap0);  %����pcolor�Ĺٷ�ʾ��
%     colorpan=ColorPanSet(1);
%     colormap(colorpan);%����ɫ��
%     set(ss, 'LineStyle','none');
%     
%     axis equal
%     xlabel('\it n miles', 'Fontname', 'Times New Roman','FontSize',15);
%     ylabel('\it n miles', 'Fontname', 'Times New Roman','FontSize',15);
%     title(['t=',num2str(k),'s'], 'Fontname', 'Times New Roman','FontSize',15);
%     box on;
%         figure
%     OSInferMap0=TotalMap(figNO).Ship4;
%     OSInferMap1=OSInferMap0;
%     OSInferMap0(OSInferMap0>50)=50;
%     
%     ss=pcolor(X,Y,OSInferMap0);  %����pcolor�Ĺٷ�ʾ��
%     colorpan=ColorPanSet(1);
%     colormap(colorpan);%����ɫ��
%     set(ss, 'LineStyle','none');
%     
%     axis equal
%     xlabel('\it n miles', 'Fontname', 'Times New Roman','FontSize',15);
%     ylabel('\it n miles', 'Fontname', 'Times New Roman','FontSize',15);
%     title(['t=',num2str(k),'s'], 'Fontname', 'Times New Roman','FontSize',15);
%     box on;
    %     end
end