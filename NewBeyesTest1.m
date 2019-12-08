%% 改进后的贝叶斯推断测试
%本程序用于测试新的贝叶斯推断函数
% 新的贝叶斯公式：
% 1.原基本公式（1）Pr(Xi+1|Xi=xi,theta)=kexp(a(d1-d2))
% 其中: d1=d(xi,Xi+1,theta0);
%      d2=d(xi,theta0);
% 2.现加上判断是否已经经过了路径点，改为：
% if d(xi,theta0)>d(theta1,theta0)  尚未经过路径点
%  d1=d(xi,Xi+1,theta1,theta0);
%  d2=d(xi,theta1,theta0);
% elseif d(xi,theta0)<=d(theta1,theta0)   已经过路径点
%  d1=d(xi,Xi+1,theta0);
%  d2=d(xi,theta0);

clear
load pos.mat
%此处用pos1做本船，pos2做目标船，当前为第21s
boatsize = [ 250, 30
    250, 30
    250, 30
    250, 30
    ];

%当前本船与目标船关系
CAL=[2 0 0 1
    1 2 1 0
    1 0 2 0
    0 1 1 2];

% [X,Y]=meshgrid(0:0.01:6,0:0.01:6);      % Pnew = (Pold - Pref)*scale,这里Pref是（0,0），所以起始位置只能是（0,0）,15 Nov 2019, by WS
[X,Y]=meshgrid(-10:0.01:10,-10:0.01:10);  % 注意地图的尺寸，每一个格子是0.01海里见方的，因此需要对下面格子的检索进行新的安排，并且思考如何用到函数中
[m0,n0]=size(X);


for figNO=1:1:4
    if figNO>1
        time=300*(figNO-1); %选取时刻分别为50，300*(figNO-1)
    else
        time=50;
    end
    OS.speed=18/3600;
    TS(1).speed=18/3600;
    TS(2).speed=18/3600;
    TS(3).speed=18/3600;
    
    OS.length=boatsize(1,1)/1852;
    TS(1).length=boatsize(2,1)/1852;
    TS(2).length=boatsize(3,1)/1852;
    TS(3).length=boatsize(4,1)/1852;
    
    OS.pos=pos1(1:time+1,:);
    OS.Course=c1(1:time+1,:);
    
    TS(1).pos=pos2(1:time+1,:);
    TS(2).pos=pos3(1:time+1,:);
    TS(3).pos=pos4(1:time+1,:);
    
    TS(1).Course=c2(1:time+1,:);
    TS(2).Course=c3(1:time+1,:);
    TS(3).Course=c4(1:time+1,:);
    TS(1).infer=0; %即开始预测
    TS(2).infer=0; %即开始预测
    TS(3).infer=0; %即开始预测
    for TSi=3:1:3
        %       for TSi=3:3
        
        IntentionMap2=zeros(m0,n0);
        IntentionMap3=zeros(m0,n0);
        IntentionMap4=zeros(m0,n0);
        
        CPA_temp = computeCPA0(OS,TS(TSi),1500);
        CPA(figNO).ship(TSi,:)=CPA_temp;
        if CollisionRisk0(OS,TS(TSi),1852) %无风险为0，有风险为1，即有风险时才执行
            TS(TSi).infer=1; %即开始预测
            disp(['当前本船与目标船',num2str(TSi)+1,'有碰撞风险，DCPA=',num2str(CPA_temp(1,5)),'，TCPA= ',num2str(CPA_temp(1,6))]);
            %有风险，执行预测
            
            map=zeros(m0,n0);
            IntentionMap0=zeros(m0,n0);
            
            OS_waypoint.pos=OS.pos(end,:);
            OS_waypoint.Course=OS.Course(end,:);
            OS_waypoint.speed=OS.speed;
            OS_waypoint.length=OS.length;
            TS_waypoint.pos=TS(TSi).pos(end,:);
            TS_waypoint.Course=TS(TSi).Course(end,:);
            TS_waypoint.speed=TS(TSi).speed;
            TS_waypoint.length=TS(TSi).length;
            
            WayPointOT0 = WayPoint(OS_waypoint,TS_waypoint);
            WayPointTO0 = WayPoint(TS_waypoint,OS_waypoint);
            
            WayPointOT = floor((WayPointOT0+10*ones(size(WayPointOT0)))*100); %由于地图是-10:0.01:10,所以，每一个海里为单位的要放大100倍取整数来归入某一个格子
            WayPointTO = floor((WayPointTO0+10*ones(size(WayPointTO0)))*100);
            % BAYESIANINTENTIONPRED 用于船舶意图预测
            % 参考论文：Bayesian Intention Inference for Trajectory Prediction with an Unknown Goal Destination
            % inputs：
            %    OtherTrack: n*2数组，他船轨迹(n>=2)
            %                此处用pos1做本船，pos2做目标船，前100s轨迹做预测
            %         OtherTrack0=pos2(1:50,:);
            OtherTrack0=TS(TSi).pos(1:time,:);
            OtherTrack=floor((OtherTrack0+10*ones(size(OtherTrack0)))*100);
            %    likelihood: 2*2数组,likelihood(1,1)本船猜测他船从本船船头经过的似然度
            %                        likelihood(1,2)本船猜测他船从本船船尾经过的似然度
            %                        likelihood(2,1)本船猜测他船，猜测本船从他船船头经过的似然度
            %                        likelihood(2,2)本船猜测他船，猜测本船从他船船尾经过的似然度
            %                likelihood为对称矩阵，例如[0.3, 0.7; 0.7, 0.3]
            disp(['目标船',num2str(TSi)+1,',CAL=',num2str(CAL(1,TSi+1))]);
            if CAL(1,TSi+1)==1
                likelihood=[0.05, 0.95;0.95, 0.05];
                
            else
                likelihood=[0.95, 0.05;0.05, 0.95];
            end
            
            %    pointOfPass: 2*2数组，pointOfPass(1,:)本船猜测他船从本船船头经过的点
            %                          pointOfPass(2,:)本船猜测他船从本船船尾经过的点
            
            pointOfPass=[WayPointTO(1:2)
                WayPointTO(3:4)] ;
            
            %     IntentionMap0=BayesianIntentionPred(OtherTrack, pointOfPass, likelihood, map);
            %% 贝叶斯推断
            alpha = 1;
            N = 100; % 蒙特卡洛取样次数
            k = 300; % 预测步数
            
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
                    % 防止两步落在一个格子里的情况发生
                    if isequal(tempArrow, [0 0]) && i>5
                        iii=1;
                        while isequal(tempArrow, [0 0])
                            tempArrow = OtherTrack(i+1, :) - OtherTrack(i-iii, :);
                            iii=iii+1;
                        end
                    end
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
                    % 防止两步落在一个格子里的情况发生
                    if isequal(tempArrow, [0 0])
                        iii=1;
                        while isequal(tempArrow, [0 0])
                            tempArrow = OtherTrack(n, :) - OtherTrack(n-iii, :);
                            iii=iii+1;
                        end
                    end
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
                    select = rand();                     %轮盘法的蒙特卡洛分析
                    upper = [PrX(1), PrX(1)+PrX(2), 1];  %不转向，右转，左转
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
            IntentionMap=IntentionMap0;
        else  %没有碰撞风险时，即DCPA大于某数之后，预测将不准确，此时也不用预测
            TS(TSi).infer=0; %即不预测
            IntentionMap=zeros(m0,n0);%不执行预测的话，直接为0
            disp(['当前本船与目标船',num2str(TSi)+1,'无碰撞风险，DCPA=',num2str(CPA_temp(1,5)),'，TCPA= ',num2str(CPA_temp(1,6))]);
        end
        %存储数据
        %         TotalMap(figNO).OSinfer(:,:,TSi)=IntentionMap;
        if TSi==1
            IntentionMap2=IntentionMap;
        elseif TSi==2
            IntentionMap3=IntentionMap;
        elseif TSi==3
            IntentionMap4=IntentionMap;
        end
        
    end
    
    TotalMap(figNO).Ship2=IntentionMap2;
    TotalMap(figNO).Ship3=IntentionMap3;
    TotalMap(figNO).Ship4=IntentionMap4;
    %     IntentionMap2=[];
    %     IntentionMap3=[];
    %     IntentionMap4=[];
    
    
    %% 绘图测试
    figure
    if TS(1).infer ==0 && TS(2).infer ==0 && TS(3).infer ==0
        
        %WTF:画出船舶的初始位置
        drawShip0(pos1(1,:),c1(1),1,400);
        drawShip0(pos2(1,:),c2(1),2,400);
        drawShip0(pos3(1,:),c3(1),3,400);
        drawShip0(pos4(1,:),c4(1),4,400);
        
        %WTF:画出船舶的结束位置
        drawShip(pos1(time,:),c1(time,:),1,400);
        drawShip(pos2(time,:),c2(time,:),2,400);
        drawShip(pos3(time,:),c3(time,:),3,400);
        drawShip(pos4(time,:),c4(time,:),4,400);
        
        %WTF:画出过往的航迹图
        plot(pos1(1:time,1),pos1(1:time,2),'r-');
        plot(pos2(1:time,1),pos2(1:time,2),'g-');
        plot(pos3(1:time,1),pos3(1:time,2),'b-');
        plot(pos4(1:time,1),pos4(1:time,2),'k-');
        
        axis equal
        xlabel('\it n miles', 'Fontname', 'Times New Roman','FontSize',15);
        ylabel('\it n miles', 'Fontname', 'Times New Roman','FontSize',15);
        title(['t=',num2str(time),'s'], 'Fontname', 'Times New Roman','FontSize',15);
        box on;
        
    else
        OSInferMap01=TotalMap(figNO).Ship2;
        OSInferMap02=TotalMap(figNO).Ship3;
        OSInferMap03=TotalMap(figNO).Ship4;
        OSInferMap=OSInferMap01+OSInferMap02+OSInferMap03;
        %         TopValue=FindValue(OSInferMap,10);
        TopValue=60;
        OSInferMap(OSInferMap>TopValue)=TopValue;
        
        ss=pcolor(X,Y,OSInferMap);  %来自pcolor的官方示例
        set(ss, 'LineStyle','none');
        colorpan=ColorPanSet(6);
        colormap(colorpan);%定义色盘
        hold on
        
        %WTF:画出船舶的初始位置
        drawShip0(pos1(1,:),c1(1),1,400);
        drawShip0(pos2(1,:),c2(1),2,400);
        drawShip0(pos3(1,:),c3(1),3,400);
        drawShip0(pos4(1,:),c4(1),4,400);
        
        %WTF:画出船舶的结束位置
        drawShip(pos1(time,:),c1(time,:),1,400);
        drawShip(pos2(time,:),c2(time,:),2,400);
        drawShip(pos3(time,:),c3(time,:),3,400);
        drawShip(pos4(time,:),c4(time,:),4,400);
        
        %WTF:画出过往的航迹图
        plot(pos1(1:time,1),pos1(1:time,2),'r-');
        plot(pos2(1:time,1),pos2(1:time,2),'g-');
        plot(pos3(1:time,1),pos3(1:time,2),'b-');
        plot(pos4(1:time,1),pos4(1:time,2),'k-');
        
        axis equal
        xlabel('\it n miles', 'Fontname', 'Times New Roman','FontSize',15);
        ylabel('\it n miles', 'Fontname', 'Times New Roman','FontSize',15);
        title(['t=',num2str(time),'s'], 'Fontname', 'Times New Roman','FontSize',15);
        box on;
    end
end