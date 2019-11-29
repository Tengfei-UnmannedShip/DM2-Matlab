%% 贝叶斯推断测试，针对单船的测试成功

%本程序用于测试贝叶斯推断函数
%关键是考虑地图的统一
clear
load pos.mat
%此处用pos1做本船，pos2做目标船，当前为第21s
boatsize = [ 250, 30
    250, 30
    250, 30
    250, 30
    ];
OS.speed=18/3600;
TS.speed=18/3600;
OS.length=boatsize(1,1)/1852;
TS.length=boatsize(2,1)/1852;
OS.pos=pos1(51,:);
OS.Course=c1(51,:);
TS.pos=pos2(51,:);
TS.Course=c1(51,:);


% [X,Y]=meshgrid(0:0.01:6,0:0.01:6);      % Pnew = (Pold - Pref)*scale,这里Pref是（0,0），所以起始位置只能是（0,0）,15 Nov 2019, by WS
[X,Y]=meshgrid(-10:0.01:10,-10:0.01:10);  % 注意地图的尺寸，每一个格子是0.01海里见方的，因此需要对下面格子的检索进行新的安排，并且思考如何用到函数中
[m0,n0]=size(X);
map=zeros(m0,n0);
IntentionMap0=zeros(m0,n0);
IntentionMap=zeros(m0,n0);

WayPointOT0 = WayPoint(OS,TS,1500);
WayPointTO0 = WayPoint(TS,OS,1500);

WayPointOT = floor((WayPointOT0+10*ones(size(WayPointOT0)))*100); %由于地图是-10:0.01:10,所以，每一个海里为单位的要放大100倍取整数来归入某一个格子
WayPointTO = floor((WayPointTO0+10*ones(size(WayPointTO0)))*100);
% BAYESIANINTENTIONPRED 用于船舶意图预测
% 参考论文：Bayesian Intention Inference for Trajectory Prediction with an Unknown Goal Destination
% inputs：
%    OtherTrack: n*2数组，他船轨迹(n>=2)
%                此处用pos1做本船，pos2做目标船，前100s轨迹做预测
OtherTrack0=pos2(1:50,:);
OtherTrack=floor((OtherTrack0+10*ones(size(OtherTrack0)))*100); 
%    likelihood: 2*2数组,likelihood(1,1)本船猜测他船从本船船头经过的似然度
%                        likelihood(1,2)本船猜测他船从本船船尾经过的似然度
%                        likelihood(2,1)本船猜测他船，猜测本船从他船船头经过的似然度
%                        likelihood(2,2)本船猜测他船，猜测本船从他船船尾经过的似然度
%                likelihood为对称矩阵，例如[0.3, 0.7; 0.7, 0.3]
likelihood=[0.95, 0.05; 0.05, 0.95];

%    pointOfPass: 2*2数组，pointOfPass(1,:)本船猜测他船从本船船头经过的点
%                          pointOfPass(2,:)本船猜测他船从本船船尾经过的点

pointOfPass=[WayPointTO(1:2)
    WayPointTO(3:4)] ;

%     IntentionMap0=BayesianIntentionPred(OtherTrack, pointOfPass, likelihood, map);
%% 贝叶斯推断
alpha = 1;
N = 100; % 蒙特卡洛取样次数
k = 500; % 预测步数

n = size(OtherTrack, 1);
m = size(likelihood, 2);
goalPoints = pointOfPass; %船头船尾的目标点
PrXTheta = zeros(n-1, m); % 文献中公式(1)
for i=1:n-1
    for j=1:m
        tempX = zeros(3, 2);
        tempP = zeros(1, 3);
        tempX(1, :) = OtherTrack(i+1, :);
        tempArrow = OtherTrack(i+1, :) - OtherTrack(i, :);
        tempSpeed = sqrt(sum(tempArrow.^2));
        tempAlpha = atan2d(tempArrow(2), tempArrow(1));
        tempX(2, :) = OtherTrack(i, :) + tempSpeed * [cosd(tempAlpha+45), sind(tempAlpha+45)];
        tempX(3, :) = OtherTrack(i, :) + tempSpeed * [cosd(tempAlpha-45), sind(tempAlpha-45)];
        for jj=1:3
            tempDistans1 = sqrt(sum((OtherTrack(i, :)-tempX(jj, :)).^2));
            tempDistans2 = sqrt(sum((goalPoints(j, :)-tempX(jj, :)).^2));
            tempDistans3 = sqrt(sum((goalPoints(j, :)-OtherTrack(i, :)).^2));
            tempP(jj) = exp(-alpha*(tempDistans1+tempDistans2-tempDistans3));
        end
        PrXTheta(i, j) = tempP(1)/sum(tempP); %PrXTheta是公式（1）的结果
    end
end

PrTheta = zeros(n, m);
PrTheta(1, :) = likelihood(1, :);
for i=1:n-1     % 文献中公式(2)
    PrTheta(i+1, :) = PrTheta(i, :).*PrXTheta(i, :);
    PrTheta(i+1, :) = PrTheta(i+1, :)./sum(PrTheta(i+1, :)); %归一化
end

for i=1:1:N      %开始蒙特卡洛仿真
    Bpos1 = OtherTrack(n-1, :); %当前时刻和上一个时刻的位置，用于进行贝叶斯推断，这两个时刻的间距决定了步长
    Bpos2 = OtherTrack(n, :);
    for j=1:k
        PrX = zeros(1, 3);
        tempX = zeros(3, 2);
        tempP = zeros(3, m);
        tempArrow = Bpos2 - Bpos1;
        tempSpeed = sqrt(sum(tempArrow.^2));  %当前的速度／步长，即两个位置之间的实际距离
        tempAlpha = atan2d(tempArrow(2), tempArrow(1));   %当前的航向角
        tempX(1, :) = Bpos2 + tempSpeed * [cosd(tempAlpha), sind(tempAlpha)];   %当前位置不变方向
        tempX(2, :) = Bpos2 + tempSpeed * [cosd(tempAlpha+45), sind(tempAlpha+45)];   %+45度
        tempX(3, :) = Bpos2 + tempSpeed * [cosd(tempAlpha-45), sind(tempAlpha-45)];   %-45度
        for ii=1:m       %更新一步的PrXTheta
            for jj=1:3
                tempDistans1 = sqrt(sum((Bpos2-tempX(jj, :)).^2));
                tempDistans2 = sqrt(sum((goalPoints(ii, :)-tempX(jj, :)).^2));
                tempDistans3 = sqrt(sum((goalPoints(ii, :)-Bpos2).^2));
                tempP(jj, ii) = exp(-alpha*(tempDistans1+tempDistans2-tempDistans3)); %公式（3）第一部分
            end
            tempP(:, ii) = tempP(:, ii)/sum(tempP(:, ii));
        end
        for jj=1:3
            for ii=1:m
                PrX(jj) = PrX(jj)+tempP(jj, ii)*PrTheta(n, ii);
            end
        end
        select = rand();
        upper = [PrX(1), PrX(1)+PrX(2), 1];
        for jj=1:3
            if select < upper(jj)
                break;
            end
        end
        Bpos1 = Bpos2;
        Bpos2 = tempX(jj, :);
        point0= Bpos2;
        point = floor(Bpos2);
        map(point(2), point(1)) = map(point(2), point(1)) + 1;
    end
end
IntentionMap0=map;
% 
% [row,col]=find(IntentionMap0~=0);%返回矩阵B中非零元素对应的行和列
% TS.pos(1)=row(1);
% TS.pos(2)=col(1);
% WayPointTO = WayPoint( TS,OS,1500 );
% pointOfPass=[WayPointTO(1:2);WayPointTO(3:4)] ;
% IntentionMap=IntentionMap+IntentionMap0;
% v=find(IntentionMap~=0);%返回B中非零元素
% [row,col]=find(IntentionMap~=0);%返回矩阵B中非零元素对应的行和列

IntentionMap1=IntentionMap0;


colorpan=[1,1,1]; %设置色盘一切为0时，显示白色
% 设置3，由标准色jet修改而来从白到浅蓝到蓝到青蓝到黄到红到深红
% 白：位置1，颜色[1 1 1]
% 浅蓝：位置15，颜色[173/255 235/255 1](RGB=[173 235 255])
% 蓝：位置40，颜色[0 0 1]
% 青蓝：位置55，颜色[0 1 1]
% 黄：位置70，颜色[1 1 0]
% 红：位置90，颜色[1 0 0]
% 深红：位置100，颜色[132/255 0 0](RGB=[132 0 0])
for k=1:1:99
    if k<15
        colorpan=[colorpan;1-((255-173)/255)*(k/14),1-((255-235)/255)*(k/14),1];  % 浅蓝：位置15，颜色[173/255 235/255 1](RGB=[173 235 255])
    elseif k>=15 && k<40
        colorpan=[colorpan;173/255*(1-(k-14)/(39-14)),235/255*(1-(k-14)/(39-14)),1]; % 蓝：位置40，颜色[0 0 1]
    elseif k>=40 && k<55
        colorpan=[colorpan;0,(k-39)/(54-39),1]; % 青蓝：位置55，颜色[0 1 1]
    elseif k>=55 && k<70
        colorpan=[colorpan;(k-54)/(69-54),1,1-((k-54)/(69-54))]; % 黄：位置70，颜色[1 1 0]
    elseif k>=70 && k<90
        colorpan=[colorpan;1,1-((k-69)/(89-69)),0]; % 红：位置90，颜色[1 0 0]
    else
        colorpan=[colorpan;1-(132/255)*((k-89)/(100-89)),0,0]; % 深红：位置100，颜色[132/255 0 0](RGB=[132 0 0])
    end
end
% 
% ppos1 = OtherTrack0(n-1, :); %当前时刻和上一个时刻的位置，用于进行贝叶斯推断，这两个时刻的间距决定了步长
% ppos2 = OtherTrack0(n, :);
%不显示网格线的pcolor
figure
ss=pcolor(X,Y,IntentionMap1);  %来自pcolor的官方示例
colormap(colorpan);%定义色盘
colorbar
set(ss, 'LineStyle','none');
hold on
plot(pos2(40,1),pos2(40,2),'or')
hold on
plot(pos2(50,1),pos2(50,2),'ob')
hold on
plot(0,0,'*')
hold on
xpos=pos2(1:400,1);
ypos=pos2(1:400,2);
plot(xpos,ypos,'LineWidth',1);
axis equal          % 坐标轴等比例,15 Nov 2019, by WS
% axis([2, 5, 0, 5])  % 限制显示范围,15 Nov 2019, by WS


