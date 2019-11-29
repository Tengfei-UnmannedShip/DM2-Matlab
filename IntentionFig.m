%% Intention Map绘图程序
clear
clc
close all

load IntentionMapData
load pos
[X,Y]=meshgrid(-10:0.01:10,-10:0.01:10);  % 注意地图的尺寸，每一个格子是0.01海里见方的，因此需要对下面格子的检索进行新的安排，并且思考如何用到函数中
[m0,n0]=size(X);
map=zeros(m0,n0);
%
% figure
% ha = MarginEdit(3,3,[.05  0.05],[.05  0.05],[0.05  0.05],1);
for figNO=3
    %     for TSi=1:3
    figure
    if figNO>1
        k=300*(figNO-1); %选取时刻分别为50，300*(figNO-1)
    else
        k=50;
    end
    
    %     subplot(3,2,fig);
    %     hold on;
    %     axes(ha(figNO));
    %WTF:画出2~4号船船头的预测轨迹
    
    %         OSInferMap=IntentionMap1;
    
    OSInferMap0=TotalMap(figNO).Ship2;
    OSInferMap1=OSInferMap0;
    OSInferMap0(OSInferMap0>50)=50;
    
            OSInferMap1(all(OSInferMap1==0,2),:) = [];%去掉矩阵中的全0行
    %         OSInferMap0(all(OSInferMap0==0,1),:) = [];%去掉矩阵中的全0列
    
    ss=pcolor(X,Y,OSInferMap0);  %来自pcolor的官方示例
    colorpan=ColorPanSet(1);
    colormap(colorpan);%定义色盘
    set(ss, 'LineStyle','none');
    
    %     hold on
    %     %WTF:画出船舶的初始位置
    %     drawShip0(pos1(1,:),c1(1),1,400);
    %     drawShip0(pos2(1,:),c2(1),2,400);
    %     drawShip0(pos3(1,:),c3(1),3,400);
    %     drawShip0(pos4(1,:),c4(1),4,400);
    %
    %     %WTF:画出船舶的结束位置
    %     drawShip(pos1(k,:),c1(k),1,400);
    %     drawShip(pos2(k,:),c2(k),2,400);
    %     drawShip(pos3(k,:),c3(k),3,400);
    %     drawShip(pos4(k,:),c4(k),4,400);
    %
    %     %WTF:画出过往的航迹图
    %     plot(pos1(1:k,1),pos1(1:k,2),'r-');
    %     plot(pos2(1:k,1),pos2(1:k,2),'g-');
    %     plot(pos3(1:k,1),pos3(1:k,2),'b-');
    %     plot(pos4(1:k,1),pos4(1:k,2),'k-');
    
    %     %WTF:画出船头的圆用于表示安全范围
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
%     ss=pcolor(X,Y,OSInferMap0);  %来自pcolor的官方示例
%     colorpan=ColorPanSet(1);
%     colormap(colorpan);%定义色盘
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
%     ss=pcolor(X,Y,OSInferMap0);  %来自pcolor的官方示例
%     colorpan=ColorPanSet(1);
%     colormap(colorpan);%定义色盘
%     set(ss, 'LineStyle','none');
%     
%     axis equal
%     xlabel('\it n miles', 'Fontname', 'Times New Roman','FontSize',15);
%     ylabel('\it n miles', 'Fontname', 'Times New Roman','FontSize',15);
%     title(['t=',num2str(k),'s'], 'Fontname', 'Times New Roman','FontSize',15);
%     box on;
    %     end
end