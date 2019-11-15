%% 用于绘图上的各种测试，主要是imagesc函数和pcolor函数
% 测试内容：
% 1.色盘设置
% 2.坐标轴方向设置
% 3.pcolor网格线设置
%基本数据
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
for k=1:1:100
colorpan=[colorpan;1-0.01*k 1-0.01*k 1];
end
% colorpan=[1 1 1
%     0 0 1];


% 最原始的imagesc和pcolor
figure
subplot(1,2,1)
imagesc(map)
subplot(1,2,2)
pcolor(map)

%原始的imagesc和色盘设置
figure
imagesc(map)
colormap(colorpan);%定义色盘
colorbar
hold on
plot(20,20,'o','color','red');

%imagesc坐标轴方向设置
figure
imagesc(map)
colormap(colorpan);%定义色盘
colorbar
set(gca,'YDir','normal')
hold on
plot(20,20,'o','color','red');

%原始pcolor和色盘设置
figure
pcolor(map);  %来自pcolor的官方示例
colormap(colorpan);%定义色盘
colorbar
hold on
plot(20,20,'o','color','red');

%不显示网格线的pcolor
figure
gca=pcolor(map);  %来自pcolor的官方示例
colormap(colorpan);%定义色盘
colorbar
hold on
plot(20,20,'o','color','red');
set(gca, 'LineStyle','none');