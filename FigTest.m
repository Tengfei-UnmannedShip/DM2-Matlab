%% ���ڻ�ͼ�ϵĸ��ֲ��ԣ���Ҫ��imagesc������pcolor����
% �������ݣ�
% 1.ɫ������
% 2.�����᷽������
% 3.pcolor����������
%��������
clear

map=zeros(100,100);
map0=zeros(100,100);
map1=zeros(100,100);
for i=1:1:50
    
    map0(i,2*i)=1*i;
    map1(i,i)=2*i;
    map=map0+map1;
    
end

% ����ɫ�̾���
% ������ɫ����
% ��ɫ	RGB ��Ԫ��
% ��ɫ	[1 1 0]
% Ʒ��ɫ  [1 0 1]
% ����ɫ  [0 1 1]
% ��ɫ    [1 0 0]
% ��ɫ	[0 1 0]
% ��ɫ	[0 0 1]
% ��ɫ	[1 1 1]
% ��ɫ	[0 0 0]


colorpan=[1 1 1]; %һ��Ϊ0ʱ����ʾ��ɫ
% ����1���Ӱ׵�����[1 1 1]~[0 0 1]��
% for k=1:1:100
%     colorpan=[colorpan;1-0.01*k 1-0.01*k 1];
% end

% ����1.5���Ӱ׵���������������
% �ף�λ��1����ɫ[1 1 1]
% ������λ��30����ɫ[0 1 1]
% ����λ��80����ɫ[0 0 1]
% ������λ��101����ɫ[20/255 43/255 140/255](RGB=[20 43 140])
for k=1:1:99
    if k<30
        colorpan=[colorpan;1-(k/29),1,1];
    elseif k>=30 && k<80
        colorpan=[colorpan;0,1-((k-29)/(79-29)),1];
    else
        colorpan=[colorpan;(20/255)*((k-79)/(100-79)),(43/255)*((k-79)/(100-79)),1-(140/255)*((k-79)/(100-79))]; 
    end
end


% % ����2���ɱ�׼ɫjet�޸Ķ����Ӱ׵������Ƶ��죨[1 1 1]��[0 0 1]��[1 1 0]��[1 0 0])
% % �ף�λ��1����ɫ[1 1 1]
% % ������λ��20����ɫ[0 1 1]
% % �ƣ�λ��60����ɫ[1 1 0]
% % �죺λ��90����ɫ[1 0 0]
% % ��죺λ��100����ɫ[132/255 0 0](RGB=[132 0 0])
% for k=1:1:99
%     if k<20
%         colorpan=[colorpan;1-(k/19),1,1];
%     elseif k>=20 && k<60
%         colorpan=[colorpan;(k-19)/(59-19),1,1-((k-19)/(59-19))];
%     elseif k>=60 && k<90
%         colorpan=[colorpan;1,1-((k-59)/(89-59)),0]; 
%     else
%         colorpan=[colorpan;1-(132/255)*((k-89)/(100-89)),0,0]; 
%     end
% end




% % ��ԭʼ��imagesc��pcolor
% figure
% subplot(1,2,1)
% imagesc(map)
% subplot(1,2,2)
% pcolor(map)
%
% %ԭʼ��imagesc��ɫ������
% figure
% imagesc(map)
% colormap(colorpan);%����ɫ��
% colorbar
% hold on
% plot(20,20,'o','color','red');
%
% %imagesc�����᷽������
% figure
% imagesc(map)
% colormap(colorpan);%����ɫ��
% colorbar
% set(gca,'YDir','normal')
% hold on
% plot(20,20,'o','color','red');
%
% %ԭʼpcolor��ɫ������
% figure
% pcolor(map);  %����pcolor�Ĺٷ�ʾ��
% colormap(colorpan);%����ɫ��
% colorbar
% hold on
% plot(20,20,'o','color','red');

%����ʾ�����ߵ�pcolor
figure
gca=pcolor(map);  %����pcolor�Ĺٷ�ʾ��
colormap(colorpan);%����ɫ��
colorbar
set(gca, 'LineStyle','none');
% hold on
% plot(20,20,'o','color','red');
