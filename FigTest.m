%% ���ڻ�ͼ�ϵĸ��ֲ��ԣ���Ҫ��imagesc������pcolor����
% �������ݣ�
% 1.ɫ������
% 2.�����᷽������
% 3.pcolor����������
%��������
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
for k=1:1:100
colorpan=[colorpan;1-0.01*k 1-0.01*k 1];
end
% colorpan=[1 1 1
%     0 0 1];


% ��ԭʼ��imagesc��pcolor
figure
subplot(1,2,1)
imagesc(map)
subplot(1,2,2)
pcolor(map)

%ԭʼ��imagesc��ɫ������
figure
imagesc(map)
colormap(colorpan);%����ɫ��
colorbar
hold on
plot(20,20,'o','color','red');

%imagesc�����᷽������
figure
imagesc(map)
colormap(colorpan);%����ɫ��
colorbar
set(gca,'YDir','normal')
hold on
plot(20,20,'o','color','red');

%ԭʼpcolor��ɫ������
figure
pcolor(map);  %����pcolor�Ĺٷ�ʾ��
colormap(colorpan);%����ɫ��
colorbar
hold on
plot(20,20,'o','color','red');

%����ʾ�����ߵ�pcolor
figure
gca=pcolor(map);  %����pcolor�Ĺٷ�ʾ��
colormap(colorpan);%����ɫ��
colorbar
hold on
plot(20,20,'o','color','red');
set(gca, 'LineStyle','none');