%% 最终测试（试验步骤3）
% 试验第3步，测试单船从SP到WP到GP

clear all;
clc;
tic;%tic1
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
    ship(j).speed1 = boat(j,3)*1852/3600;%WTF：原航速，用于ship domain
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
    ship(j).pos=ship(j).Start_pos*1852;
    ship(j).DCPA_Record = [];
    ship(j).TCPA_Record = [];
    ship(j).length=boatsize(j,1);
    ship(j).width=boatsize(j,2);
    ship(j).goalPiont0= Goal_point(ship(j).Start_pos(1),ship(j).Start_pos(2),ship(j).Course,GoalRange);
    ship(j).posData=[];
    ship(j).courseData=[];
    
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
Re=100; %栅格化的分辨率
%% 每个时刻的决策
%绘制当前时刻的APF地图，把四艘船的一次全算出来，之后每次使用都叠加，这样可以少算8艘船的
for i=1:1:ShipNum
    
    APF(i).map= DrawAPF(ship(i),ship(i),APFmapSize,Re,0);
    
end
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% APFvalue=APF(i).map;
% for i=2:1:length(APF)
%     APFvalue=APFvalue+APF(i).map;
% end
%
% x1=-APFmapSize(1)*1852;
% x2=APFmapSize(1)*1852;
% y1=-APFmapSize(2)*1852;
% y2=APFmapSize(2)*1852;
%
% [APF_X,APF_Y]=meshgrid(x1:100:x2,y1:100:y2);
%
% figure
% mesh(APF_X,APF_Y,APFvalue);
% axis equal;
% axis off;
%
% figure
% contourf(APF_X,APF_Y,APFvalue,'LevelStep',30);  %带填充颜色的等高线图
% axis equal;
% axis off;
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i=1:1:1
    %开始每艘船的决策
    %% 计算本船的路径点和目标点
    %找到当前时刻本船眼中每艘目标船的决策路径点（船头的船尾的）
    kk=1;
    WP_Pos=ship(i).pos(end,:); %WP_Pos第一行是本船当前位置
    for ts=1:1:ShipNum
        Dis_temp0=ship(i).pos(end,:)-ship(ts).pos(end,:);
        ship(i).dis(t,ts)=norm(Dis_temp0);
        if ts~=i
            TS(kk).length = ship(ts).length;
            TS(kk).Course = ship(ts).Course(end,:);
            TS(kk).speed = ship(ts).speed(end,:);
            TS(kk).pos = ship(ts).pos(end,:);
            kk=kk+1;
            WP_Pos=[WP_Pos;ship(ts).pos(end,:)]; %WP_Pos之后每行是按顺序的目标船的位置
        end
    end
    ship(i).TSWayPoint0 = WayPoint0(ship(i),TS,1000);
    ship(i).APF=zeros(size(APF(ts).map));
    %根据CAL筛选路径点
    kk=1;
    for ts=1:1:ShipNum
        if ts~=i
            ship(i).APF=ship(i).APF+APF(ts).map;
            if CAL(i,ts)==0
                ship(i).TSWayPoint(kk,:)=ship(i).TSWayPoint0(kk,1:2);
            elseif  CAL(i,ts)==1
                ship(i).TSWayPoint(kk,:)=ship(i).TSWayPoint0(kk,3:4);
            end
            kk=kk+1;
        end
    end
    %根据三个点计算的本船的综合路径点
    ship(i).OSWayPoint=WP_allShips(WP_Pos,ship(i).TSWayPoint);
    
    %% 起点到waypoint
    % function [SetClose,SetOpen]=CircleAStar(map,start_row,start_col,end_row,end_col)
    % 算法5:多向A*算法
    %输入: 惩罚图像(矩阵)map,起点图像坐标(start_row,start_col),目标点图像坐标(destination_row, destination_col),船舶长度ShipLong,旋回半斤Rmin
    %输出: 搜索过的点集open列表，被选为最优路径节点的点集close列表
    % line1. 设置初始船舶艏向；
    
    background=ship(i).APF;
    map=ship(i).APF;
    
    start_row=round((ship(i).pos(1)+MapSize(1)*1852)/100);%APF地图分辨率是100米，因此除以100
    start_col=round((ship(i).pos(2)+MapSize(2)*1852)/100);
    
    end_row=round((ship(i).goalPiont0(1)+MapSize(1)*1852)/100);
    end_col=round((ship(i).goalPiont0(2)+MapSize(2)*1852)/100);
    OSLength=ship(i).length;
    speed=ship(i).speed;
    Course=ship(i).Course;
    dire=20; % direction跳整方向数，n向的A*
    Data=AStar2(ship(i).APF,start_row,start_col,end_row,end_col,speed,OSLength,Course,dire,0);

    ship(i).posData=Data(:,1:2);
    ship(i).courseData=Data(:,3);
    
    %% waypoint到终点
    background=ship(i).APF;
    map=ship(i).APF;
    
    start_row=round((ship(i).OSWayPoint(1)+MapSize(1)*1852)/100);%APF地图分辨率是100米，因此除以100
    start_col=round((ship(i).OSWayPoint(2)+MapSize(2)*1852)/100);
    
    end_row=round((ship(i).OSWayPoint(1)+MapSize(1)*1852)/100);
    end_col=round((ship(i).OSWayPoint(2)+MapSize(2)*1852)/100);
    OSLength=ship(i).length;
    speed=ship(i).speed;
    Course=ship(i).Course;
    dire=20; % direction跳整方向数，n向的A*
    Data=AStar2(ship(i).APF,start_row,start_col,end_row,end_col,speed,OSLength,Course,dire,0);

    ship(i).posData=Data(:,1:2);
    ship(i).courseData=Data(:,3);    
    
    
    %数据存储
    
    
end

toc
disp(['本次运行时间: ',num2str(toc)]);

%把每个位置占的格子回归到点，以中间点代替
for i=1:1:1
    for j=1:1:length(ship(i).posData)
        ship(i).pos(j+1,1)=(ship(i).posData(j,1)-0.5)*Re+(-MapSize(1)*1852);
        ship(i).pos(j+1,2)=(ship(i).posData(j,2)-0.5)*Re+(-MapSize(2)*1852);   
    end
end
%WTF:画出船舶的初始位置
drawShip0(ship(1).pos(1,:),ship(1).Course(1),1,400);
drawShip0(ship(2).pos(1,:),ship(2).Course(1),2,400);
drawShip0(ship(3).pos(1,:),ship(3).Course(1),3,400);
drawShip0(ship(4).pos(1,:),ship(4).Course(1),4,400);

%WTF:画出过往的航迹图
plot(ship(1).pos(:,1),ship(1).pos(:,2),'r-');

%WTF:画出船舶的结束位置
drawShip0(ship(1).OSWayPoint(end,:),ship(1).courseData(end),1,400);



grid on;
xlabel('\it n miles', 'Fontname', 'Times New Roman');
ylabel('\it n miles', 'Fontname', 'Times New Roman');
% title(['t=',num2str(k),'s'], 'Fontname', 'Times New Roman');
box on;


