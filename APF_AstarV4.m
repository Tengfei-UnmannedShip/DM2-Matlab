% A*与APF结合的第三版，APF使用王宁的ship domain模型

clear all;
clc;
tic;%tic1
t1=clock;
%% 场景设置，参数初始化
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
%每一艘船最大的活动范围
% map=[x1,x2,y1,y2],表示4个边界，单位海里
MapSize=[10,10];
GoalRange=MapSize-[1.5,1.5];
APFmapSize = [MapSize(1),MapSize(1),MapSize(2),MapSize(2)];
tMax=2500;
t1=0;
tt=2000;
for j=1:1:ShipNum
    
    ship(j).speeds = boat(j,3)/3600;%WTF：将航速转换为海里／秒
    ship(j).speed = boat(j,3);%WTF：原航速，用于ship domain
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
    ship(j).courseAlter = 0; %初始状态的航向角该变量为0
    ship(j).Course = ship(j).initialCourse+ship(j).courseAlter;
    ship(j).Start_pos=[boat(j,1)-ship(j).speeds*sind(ship(j).initialCourse)*0.5*tMax, boat(j,2)-ship(j).speeds*cosd(ship(j).initialCourse)*0.5*tMax];
    ship(j).pos=[ship(j).Start_pos(1)+ship(j).speeds*sind(ship(j).Course)*t1, ship(j).Start_pos(1)+ship(j).speeds*cosd(ship(j).Course)*t1];
    ship(j).DCPA_Record = [];
    ship(j).TCPA_Record = [];
    ship(j).length=boatsize(j,1);
    ship(j).width=boatsize(j,2);
    ship(j).goalPiont0= Goal_point(ship(j).Start_pos(1),ship(j).Start_pos(2),ship(j).Course,GoalRange);
end

t=1;
%场景设置
%CAL矩阵中，每一行为本船，每一列为目标船；
%CAL(j,k)意为本船j对目标船k的避碰意图，0为从船头走，1为从船尾，2为自己对自己
%场景1，全部正常
CAL=[2 0 0 1
    1 2 0 1
    1 1 2 0
    0 0 1 2];

%% 每个时刻的决策
%绘制当前时刻的APF地图，把四艘船的一次全算出来，之后每次使用都叠加，这样可以少算8艘船的
for i=1:1:ShipNum
    
    APF(i).map= DrawAPF(ship(i),ship(i),APFmapSize,0);
    
end

for i=1:1:ShipNum
    %开始每艘船的决策
    
    %% 计算本船的路径点和目标点
    %找到当前时刻本船眼中每艘目标船的决策路径点（船头的船尾的）
    kk=1;
    WP_Pos=ship(i).pos(end,:);
    for ts=1:1:ShipNum
        Dis_temp0=ship(i).pos(end,:)-ship(ts).pos(end,:);
        ship(i).dis(t,ts)=norm(Dis_temp0);
        if ts~=i
            TS(kk)= ship(ts);
            kk=kk+1;
            WP_Pos=[WP_Pos;ship(i).pos(end,:)];
        end
    end
    ship(i).TSWayPoint0 = WayPoint0(ship(i),TS,1000);
    %根据CAL筛选路径点
    kk=1;
    for ts=1:1:ShipNum
        if ts~=i
            if CAL(i,kk)==0
                ship(i).TSWayPoint(kk,:)=ship(i).TSWayPoint0(kk,1:2);
            elseif  CAL(i,kk)==1
                ship(i).TSWayPoint(kk,:)=ship(i).TSWayPoint0(kk,3:4);
            end
            kk=kk+1;
        end
    end
    %根据三个点计算的本船的综合路径点
    ship(i).OSWayPoint=WP_allShips(WP_Pos,ship(i).TSWayPoint);
    
    %% 规划本船的路径
    
    %起点到waypoint
    
    
    
    
    %waypoint到终点
    
    
    
    
    %数据存储
    
    
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp(['程序总运行时间：',num2str(etime(clock,t1))]);
