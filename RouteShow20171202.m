%%%·����ʾ���������ļ�
clear all;close all;clc
load('���ӷ���Ƴ�ͼ.mat');

% ����·���滮����
% load('C:\Users\Administrator\Desktop\����\Files\��������\��ɫ�ü�����ͼ.mat')
background=im2bw(Q);
SelectedAreaLT.La=30.90;        SelectedAreaRB.La=30.70;
SelectedAreaLT.Lo=121.83;       SelectedAreaRB.Lo=122.03;
[SaLT_la,SaLT_lo]=MKT(SelectedAreaLT.Lo,SelectedAreaLT.La);
[SaRB_la,SaRB_lo]=MKT(SelectedAreaRB.Lo,SelectedAreaRB.La);
[height,width]=size(background(:,:,1));
scale_y=height/(SaLT_la-SaRB_la);
scale_x=width /(SaRB_lo-SaLT_lo);

% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % % ���ط��Ϊ�ϰ���
% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% load('C:\Users\Administrator\Desktop\����\Files\CircleAStar1125�ɱ䲽���޸��ƶ����ۼ�����close���������\FansLocation.mat');%�������λ��
% enlarge=3;
% for ii=1:62
%     [Fy,Fx]=MKT(FansLocation(ii).Lo,FansLocation(ii).La);
%     a=floor((SaLT_la-Fy)*scale_y);
%     b=floor((Fx-SaLT_lo)*scale_x);
%     background((a-enlarge):(a+enlarge),(b-enlarge):(b+enlarge),1)=255;
%     background((a-enlarge):(a+enlarge),(b-enlarge):(b+enlarge),2)=0;
%     background((a-enlarge):(a+enlarge),(b-enlarge):(b+enlarge),3)=0;
% end

% %%%��ʾGPS�켣%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% load('C:\Users\Administrator\Desktop\����\Files\��������\ʵ��GPS�켣.mat')
% enlarge=1;
% L=length(Trace(:,1));
% for ii=1:floor(L/4)
%     [Fy,Fx]=MKT(Trace(ii,2),Trace(ii,1));
%     a=floor((SaLT_la-Fy)*scale_y);
%     b=floor((Fx-SaLT_lo)*scale_x);
%     background((a-enlarge):(a+enlarge),(b-enlarge):(b+enlarge),1)=0;
%     background((a-enlarge):(a+enlarge),(b-enlarge):(b+enlarge),2)=1;
%     background((a-enlarge):(a+enlarge),(b-enlarge):(b+enlarge),3)=0;
% %     [Fy2,Fx2]=MKT(Trace(ii-1,1),Trace(ii-1,2));
% %     c=floor((SaLT_la-Fy2)*scale_y);
% %     d=floor((Fx2-SaLT_lo)*scale_x);
% %     line([b;d],[a;c],'color','b','LineWidth',1);
% end
%%%��ʾOpen�б�%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% load('C:\Users\Administrator\Desktop\����\Files\��������\�滮·�����\MoveLength5-4Dir.mat')
% enlarge=1;
% for ii=1:length(SetOpen)
%     background((SetOpen(ii).row-enlarge):(SetOpen(ii).row+enlarge),(SetOpen(ii).col-enlarge):(SetOpen(ii).col+enlarge),1)=0;
%     background((SetOpen(ii).row-enlarge):(SetOpen(ii).row+enlarge),(SetOpen(ii).col-enlarge):(SetOpen(ii).col+enlarge),2)=255;
%     background((SetOpen(ii).row-enlarge):(SetOpen(ii).row+enlarge),(SetOpen(ii).col-enlarge):(SetOpen(ii).col+enlarge),3)=0;
% end
%%%��ʾ·��%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[starty,startx]=MKT(121.8433833,30.8396833);
    a=floor((SaLT_la-starty)*scale_y);
    b=floor((startx-SaLT_lo)*scale_x);
% [endy,endx]=MKT(121.9924833,30.7886);
[endy,endx]=MKT(122.0064,30.7759);
    c=floor((SaLT_la-endy)*scale_y);
    d=floor((endx-SaLT_lo)*scale_x);
    
% [SetClose,SetOpen]=CircleAStar(Q,a,b,c-20,d+20);
    
    
imagesc(Q);
line([b-3;b+3;b+3;b-3;b-3],[a-3;a-3;a+3;a+3;a-3],'color','g','LineWidth',5);
line([d-3;d+3;d+3;d-3;d-3],[c-3;c-3;c+3;c+3;c-3],'color','y','LineWidth',5);

Points=[264 50;539 660; 534 686; 612 743; 623 657; 592 645; 548 660];
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%    20��·��չʾ
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % load('C:\Users\Administrator\Desktop\����\Files\��������\�滮·�����\20������·��Close12041559.mat')
% load('C:\Users\Administrator\Desktop\����\Files\��������\�滮·�����\1MoveLength10-20Dir.mat')
% Curr=SetClose(end);
% while ~(Curr.row==a && Curr.col==b)
%     line([Curr.col;Curr.father.col],[Curr.row;Curr.father.row],'color','r','LineWidth',3);
%     Curr=Curr.father;
% end
% SetClose=[];

load('C:\Users\Administrator\Desktop\����\Files\CircleAStar1216���ĳ�ͼ\·���滮һ��\���ӷ���Ƴ�ͼ·��.mat')
Curr=SetClose(end);
while ~(Curr.row==a+30 && Curr.col==b)
    line([Curr.col;Curr.father.col],[Curr.row;Curr.father.row],'color','g','LineWidth',3);
    Curr=Curr.father;
end

 line([Curr.col;b],[Curr.row;a],'color','g','LineWidth',3);
