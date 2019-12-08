%% 两艘船的测试程序
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 1.2019年12月5日第一版
% 2.测试版本，算法框架为：
% （1）准备工作
% （2）判断当前状态的让路或直航船状态（Fun1）
% （3）判断CPA，在沿途船头
% （4）
% 3.当前的预测，只采用
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
clc;
% close all;
tic;%tic1
t1=clock;
%% WTF:参数初始化
%基本参数设置，方便修改,分别是：
% 1.compliance:避碰规则符合性;2.inferLabbel:是否推测
shipLabel=[
    0 0
    1 0
    1 0
    1 0];
% shipLabel=zeros(4,2);%检测是否在中间交汇
%1~2位置(中间的位置，不是起始位置)、3航速(节)、4初始航向（deg，正北为0），5决策周期时长，6检测范围（range，nm）
boat=[
    0.0, 0.0,  18,   0,    3,  6
    0.0, 0.0,  18, 230,    4,  6
    0.0, 0.0,  16, 300,    5,  6
    0.0, 0.0,  13, 135,    5,  6
    ];
boatsize = [ 250, 30
    250, 30
    250, 30
    250, 30
    ];
%具体参数设置
ShipNum=4;
MapSize=[10,10];  %每一艘船最大的活动范围
t=2500;
drawAPFtime=[1,500,1000,1500,2500];
tt=2000;
for j=1:1:ShipNum
    
    ship(j).speed = boat(j,3)*1852/3600;%WTF：航速为18海里/小时。1海里=1.852公里，因此1海里/小时=1.852公里／小时=1852/3600米/秒
    ship(j).ratio=1;             %有速度改变时改变比例
    ship(j).initialCourse = boat(j,4);
    ship(j).compliance= shipLabel(j,1); %compliance是对避碰规则的符合性，0：直线前进不避让，1：遵守，2：不遵守，反着来
    ship(j).decisioncycle=boat(j,5);
    ship(j).range=boat(j,6)*1852;  %detect range
    ship(j).no=j;  %receive information from other ships
    ship(j).inferLabbel=shipLabel(j,2); %本船是否推测标志，1为推测，0不推测
    ship(j).data=[];%WTF：ship.data置为空，用于存放每一步的历史数据
    ship(j).compliance_data=[];
    ship(j).decision_lable=0;
    ship(j).infer=[];
    ship(j).OSdecision=0;
    ship(j).Vratio=1;
    ship(j).pos=[boat(j,1)-ship(j).speed*sind(ship(j).initialCourse)*0.5*t, boat(j,2)-ship(j).speed*cosd(ship(j).initialCourse)*0.5*t];
    %   ship(i).pos = boat(i,1:2)*1852;
    ship(j).courseAlter = 0; %初始状态的航向角该变量为0
    ship(j).Course = ship(j).initialCourse+ship(j).courseAlter;
    ship(j).DCPA_Record = [];
    ship(j).TCPA_Record = [];
    ship(j).length=boatsize(j,1);
    ship(j).width=boatsize(j,2);
    GoalRange=MapSize-[1.5,1.5];
    ship(j).goalPiont= Goal_point(ship(j).pos(1),ship(j).pos(2),ship(j).Course,GoalRange);
end

pos1=zeros(t,2);
pos1(1,:)=ship(1).pos;%WTF：船舶位置矩阵第一行，但是t的值尚未确定，%%猜测船舶原定最初位置
pos2=zeros(t,2);
pos2(1,:)=ship(2).pos;
pos3=zeros(t,2);
pos3(1,:)=ship(3).pos;
pos4=zeros(t,2);
pos4(1,:)=ship(4).pos;

OSdecision=[0 0 0 0];
OSdecision_time=[ ];
OSdecConut=zeros(t,4);
s=0;
%% WTF:开始执行分布式决策
for i=2:t
    %% 全局环境
    tic ;%tic2
    t2=clock;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Update%WTF：船舶信息更新
    for j=1:ShipNum
        if ship(j).decision_lable~=0 && OSdecision(j)~=0 && ship(j).compliance~=0 %决策指标不为0,且已经决策过，即本船正常决策，位置即更新自上一步的A*决策结果，直到下一步
            ship(j).pos=ship(j).DM_pos(i-ship(j).decision_lable,2*j-1:2*j);
            ship(j).Course=ship(j).DM_c(i-ship(j).decision_lable,j);
            ship(j).courseAlter=ship(j).Course-ship(j).initialCourse;
            ship(j).Vratio=ship(j).DM_r(i-ship(j).decision_lable,j);
        else                           %上述三个都不为0才说明正常决策，否则，本船按照初设状态航行
            ship(j).Course=ship(j).initialCourse+ship(j).courseAlter;
            speed_now=ship(j).Vratio*ship(j).speed;
            %WTF：不决策的话，当前的船舶位置为（原位置x+1*当前速度v*sin（原航向角alpha），原位置y+1*当前速度v*cos（原航向角alpha））
            ship(j).pos =[ship(j).pos(1)+speed_now*sind(ship(j).Course),ship(j).pos(2)+speed_now*cosd(ship(j).Course)];
        end
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    pos1(i,:)=ship(1).pos;%WTF：扩充船舶位置矩阵，每一一个循环为一秒，每一秒就在位置矩阵下加一行，最后就是1500行。
    pos2(i,:)=ship(2).pos;
    pos3(i,:)=ship(3).pos;
    pos4(i,:)=ship(4).pos;
    
    c1(i,:)=ship(1).Course;%WTF：扩充船舶航向矩阵，每个循环更新船舶当前航向（原航向角alpha+当前的偏转角beta），最后就是1500行。
    c2(i,:)=ship(2).Course;
    c3(i,:)=ship(3).Course;
    c4(i,:)=ship(4).Course;
    
    %% 本船开始决策%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for OS=1:1:ShipNum  %从第一艘船到第四艘船的遍历
        if decisioncycle(OS,i,ship)&& ship(OS).compliance~=0%判断当前i时刻是在j船的决策周期中且compliance==1即本船正常
            %compliance=0 说明完全不变，此时不决策就好；compliance=2 反着来
            risk_neighbor_temp1=SearchNeighbor(ship(OS),ship,ship(OS).range);  %进入本船检测范围的船
            if ~isempty(risk_neighbor_temp1) && CollisionRisk0(ship(OS),risk_neighbor_temp1) %WTF:本船与临船有碰撞风险时
                % 本船决策次数计数
                TS=risk_neighbor_temp1; %现在TS的顺序是ship的顺序去掉OS之后的了
                OSdecConut(i,OS)=1;
                OSdecision(OS)=OSdecision(OS)+1;
                OSdecision_time(OS,OSdecision(OS))=i;
                disp(['t=',num2str(i),',  ','第',num2str(OS),'艘船的第',num2str(OSdecision(OS)),'次决策']);
                % STEP 0.绘出包括当前可识别到风险船舶在内的APF图，作为风险底图
                if i==ismember(drawAPFtime,1)   %确定是否APF底图
                    drawAPF=1;
                else
                    drawAPF=0;
                end
                APF  = DrawAPF(ship(OS),TS, MapSize, drawAPF );
                % STEP 1.判断TSi是让路船还是直航船
                TS_GSR = GSR(ship(OS), TS);
                % STEP 2.找到CPA，沿线向前1.5倍船长为theta1、向后1倍船长为theta2
                WayPointTS = WayPoint(ship(OS),TS,t);     %OS眼中各个TS的WayPoint
                for i_wp=1:1:length(TS)                    %各个TS眼中OS的WayPoint
                    WayPointOS = WayPoint(TS(i_wp),ship(OS),t);
                end
                %% inferLabbel==0 时本船不推测其他船
                if ship(OS).inferLabbel==0 %不管其他船舶，不推测；
                    %不推测的时候，直接在风险底图上决策
                    [SetClose,SetOpen]=AStar1(APF.map0,ship(OS).pos(1),ship(OS).pos(2),ship(OS).goalPiont(1),...
                        ship(OS).goalPiont(2),ship(OS).speed,ship(OS).length,MapSize,1);%直接调用函数，% 原版作者薛双飞
                else
                    %% inferLabbel==1 本船正常决策，用证据理论推导出当前的CAL矩阵
                    % neighbor_temp=SearchNeighbor(ship(j),ship,ship(j).range);%第j艘船的邻船，即进入检测范围的所有船
                    % 领域内所有船都要考虑到，所以不再判断范围船
                    % 如果是第一个决策周期
                    if OSdecision(OS)==1  %在第一个决策周期
                        %不推测的时候，直接在风险底图上决策
                        [SetClose,SetOpen]=AStar1(APF.map0,ship(OS).pos(1),ship(OS).pos(2),ship(OS).goalPiont(1),...
                            ship(OS).goalPiont(2),ship(OS).speed,ship(OS).length,MapSize,0);%直接调用函数，% 原版作者薛双飞
                        
                        
                    else  %非第一个决策周期
                        %% 3.用贝叶斯推测的程序生成风险图
                        % BAYESIANINTENTIONPRED 用于船舶意图预测
                        % 参考论文：Bayesian Intention Inference for Trajectory Prediction with an Unknown Goal Destination
                        % inputs：
                        %    OtherTrack:n*2数组，他船轨迹(n>=2)
                        %    likelihood: 2*2数组，likelihood(1,1)本船猜测他船从本船船头经过的似然度
                        %                        likelihood(1,2)本船猜测他船从本船船尾经过的似然度
                        %                        likelihood(2,1)本船猜测他船，猜测本船从他船船头经过的似然度
                        %                        likelihood(2,2)本船猜测他船，猜测本船从他船船尾经过的似然度
                        %                likelihood为对称矩阵，例如[0.3, 0.7; 0.7, 0.3]
                        %    pointOfPass: 2*2数组，pointOfPass(1,:)本船猜测他船从本船船头经过的点
                        %                          pointOfPass(2,:)本船猜测他船从本船船尾经过的点
                        [Astarmap] = BayesianIntentionPred(OtherTrack, pointOfPass, likelihood, map);
                        %% 4.用A*算法生成避碰路径
                        
                        [SetClose,SetOpen]=AStar1(APF.map0,ship(OS).pos(1),ship(OS).pos(2),ship(OS).goalPiont(1),...
                            ship(OS).goalPiont(2),ship(OS).speed,ship(OS).length,MapSize,1);%直接调用函数，% 原版作者薛双飞
                        
                    end
                end
            end
        end
    end
end
