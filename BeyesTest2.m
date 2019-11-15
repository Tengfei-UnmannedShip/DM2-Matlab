%% 贝叶斯推断测试
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

WayPointOT = WayPoint( OS,TS,1500 );
WayPointTO = WayPoint( TS,OS,1500 );
% BAYESIANINTENTIONPRED 用于船舶意图预测
% 参考论文：Bayesian Intention Inference for Trajectory Prediction with an Unknown Goal Destination
% inputs：
%    OtherTrack: n*2数组，他船轨迹(n>=2)
%                此处用pos1做本船，pos2做目标船，前100s轨迹做预测

OtherTrack=pos2(1:10:50*10,:);
%    likelihood: 2*2数组,likelihood(1,1)本船猜测他船从本船船头经过的似然度
%                        likelihood(1,2)本船猜测他船从本船船尾经过的似然度
%                        likelihood(2,1)本船猜测他船，猜测本船从他船船头经过的似然度
%                        likelihood(2,2)本船猜测他船，猜测本船从他船船尾经过的似然度
%                likelihood为对称矩阵，例如[0.3, 0.7; 0.7, 0.3]
likelihood=[0.3, 0.7; 0.7, 0.3];

%    pointOfPass: 2*2数组，pointOfPass(1,:)本船猜测他船从本船船头经过的点
%                          pointOfPass(2,:)本船猜测他船从本船船尾经过的点


pointOfPass=[WayPointTO(1:2)
    WayPointTO(3:4)] ;

[X,Y]=meshgrid(-10:0.01:10,-10:0.01:10);    %注意地图的尺寸，每一个格子是0.01海里见方的，因此需要对下面格子的检索进行新的安排，并且思考如何用到函数中
[m,n]=size(X);
map=zeros(m,n);
IntentionMap0=zeros(m,n);
IntentionMap=zeros(m,n);
for t=1:1  
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
            point = ceil(Bpos2);    %ceil是向上取整数，使用floor是向下取整数，有可能取到0，而map里没有0
            map(point(2), point(1)) = map(point(2), point(1)) + 1;
        end
    end
    IntentionMap0=map;
    
    [row,col]=find(IntentionMap0~=0);%返回矩阵B中非零元素对应的行和列
    TS.pos(1)=row(1);
    TS.pos(2)=col(1);
    WayPointTO = WayPoint( TS,OS,1500 );
    pointOfPass=[WayPointTO(1:2);WayPointTO(3:4)] ;
    IntentionMap=IntentionMap+IntentionMap0;   
end
v=find(IntentionMap~=0);%返回B中非零元素
[row,col]=find(IntentionMap~=0);%返回矩阵B中非零元素对应的行和列

figure
mesh(X,Y,IntentionMap);
% axis on;
axis equal;
xlabel('\it n miles', 'Fontname', 'Times New Roman');
ylabel('\it n miles', 'Fontname', 'Times New Roman');

