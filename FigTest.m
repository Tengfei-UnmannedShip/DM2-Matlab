%% 用于绘图上的各种测试，主要是imagesc函数和pcolor函数
% 测试内容：
% 1.色盘设置
% 2.坐标轴方向设置
% 3.pcolor网格线设置
%基本数据
clear

map=zeros(100,100);
map0=zeros(100,100);
map1=zeros(100,100);
for i=1:1:50
    
    map0(i,2*i)=1*i;
    map1(i,i)=2*i;
    map=map0+map1;
    
end

% 定义色盘矩阵
% 关于颜色设置
% 颜色	RGB 三元组
% 黄色	[1 1 0]
% 品红色  [1 0 1]
% 青蓝色  [0 1 1]
% 红色    [1 0 0]
% 绿色	[0 1 0]
% 蓝色	[0 0 1]
% 白色	[1 1 1]
% 黑色	[0 0 0]


colorpan=[1 1 1]; %一切为0时，显示白色
% 设置1，从白到蓝（[1 1 1]~[0 0 1]）
% for k=1:1:100
%     colorpan=[colorpan;1-0.01*k 1-0.01*k 1];
% end

% 设置1.5，从白到青蓝到蓝到深蓝
% 白：位置1，颜色[1 1 1]
% 青蓝：位置30，颜色[0 1 1]
% 蓝：位置80，颜色[0 0 1]
% 深蓝：位置101，颜色[20/255 43/255 140/255](RGB=[20 43 140])
for k=1:1:99
    if k<30
        colorpan=[colorpan;1-(k/29),1,1];
    elseif k>=30 && k<80
        colorpan=[colorpan;0,1-((k-29)/(79-29)),1];
    else
        colorpan=[colorpan;(20/255)*((k-79)/(100-79)),(43/255)*((k-79)/(100-79)),1-(140/255)*((k-79)/(100-79))]; 
    end
end


% % 设置2，由标准色jet修改而来从白到蓝到黄到红（[1 1 1]～[0 0 1]～[1 1 0]～[1 0 0])
% % 白：位置1，颜色[1 1 1]
% % 青蓝：位置20，颜色[0 1 1]
% % 黄：位置60，颜色[1 1 0]
% % 红：位置90，颜色[1 0 0]
% % 深红：位置100，颜色[132/255 0 0](RGB=[132 0 0])
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




% % 最原始的imagesc和pcolor
% figure
% subplot(1,2,1)
% imagesc(map)
% subplot(1,2,2)
% pcolor(map)
%
% %原始的imagesc和色盘设置
% figure
% imagesc(map)
% colormap(colorpan);%定义色盘
% colorbar
% hold on
% plot(20,20,'o','color','red');
%
% %imagesc坐标轴方向设置
% figure
% imagesc(map)
% colormap(colorpan);%定义色盘
% colorbar
% set(gca,'YDir','normal')
% hold on
% plot(20,20,'o','color','red');
%
% %原始pcolor和色盘设置
% figure
% pcolor(map);  %来自pcolor的官方示例
% colormap(colorpan);%定义色盘
% colorbar
% hold on
% plot(20,20,'o','color','red');

%不显示网格线的pcolor
figure
gca=pcolor(map);  %来自pcolor的官方示例
colormap(colorpan);%定义色盘
colorbar
set(gca, 'LineStyle','none');
% hold on
% plot(20,20,'o','color','red');
