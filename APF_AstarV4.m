%% 最终测试（试验步骤1，步骤2）
% 测试第1步: 找准WP
% 测试第2步: 绘制单船从SP到WP的路径。
% 要求: 1.结果输出每一步的路线；
%      2.路径尽量自然
%      3.计算效率与计算精度的调试；

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
    
    ship(j).speed_Ns = boat(j,3)/3600;%WTF：将航速转换为海里／秒
    ship(j).speed = boat(j,3);%WTF：原航速，用于ship domain
    ship(j).speed_ms = boat(j,3)*1852/3600;%WTF：原航速，用于ship domain
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
    ship(j).Start_pos=[boat(j,1)-ship(j).speed_Ns*sind(ship(j).initialCourse)*0.5*tMax, boat(j,2)-ship(j).speed_Ns*cosd(ship(j).initialCourse)*0.5*tMax];
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
%APF的地图，是实际地图的栅格化，不是栅格地图
for i=1:1:ShipNum
    
    APF(i).map= DrawAPF(ship(i),ship(i),APFmapSize,Re,0);
    
end
% % APF绘图%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
    
    start_row=round((ship(i).pos(1)+MapSize(1)*1852)/Re);%APF地图分辨率是100米，因此除以100
    start_col=round((ship(i).pos(2)+MapSize(2)*1852)/Re);
    
    end_row=round((ship(i).goalPiont0(1)+MapSize(1)*1852)/Re);
    end_col=round((ship(i).goalPiont0(2)+MapSize(2)*1852)/Re);
    OSLength=ship(i).length;
    speed=ship(i).speed;
    Course=ship(i).Course;
    dire=20; % direction跳整方向数，n向的A*
    %     Data=AStar2(ship(i).APF,start_row,start_col,end_row,end_col,speed,OSLength,Course,dire,0);
    Node_opti=1;
    background=map;
    
    start_point.row=start_row;
    start_point.col=start_col;
    
    destination.row=end_row;
    destination.col=end_col;
    Movelength=round((speed*1852/60)/100);
    ShipLong=round(OSLength/100);
    SurroundPointsNum=dire; % direction跳整方向数，n向的A*
    RudderAngle=2*pi/SurroundPointsNum;
    Rmin=2*Movelength/3; %转弯半径
    valueAPF=10;  %APF势场的价值函数
    %     Rmin=0;
    
    %开始计算
    % line2. 初始准备：
    %如果船舶位置在地图范围之外或者船舶状态不安全，算法结束，提示无安全路径，
    %否则计算起始节点各属性(坐标、艏向、惩罚值、移动代价G、到目标点的预计代价H、总代价F、下一步移动距离r、父节点、子节点等)
    %并将该节点放到open表中,初始化close列表为空；
    if (0<start_point.col<length(background(1,:))&&0<start_point.row<length(background(:,1)))
        start_point.G=0; %移动代价 G
        start_point.H=sqrt((destination.col-start_point.col)^2+(destination.row-start_point.row)^2);  %到目标点的预计代价H
        start_point.F=start_point.G+start_point.H; %总代价F
        start_point.R= Movelength; %下一步移动距离r
        %方向没有对正！！注意修改
        start_point.Dir=Course*pi/180;  %起始点艏向
        
        Open(1)=start_point; %起始点坐标
        Open(1).father=nan; %父节点
        Close(1)=Open(1); %并将该节点放到open表中,初始化close列表为空；
    end
    % 开始计算
    while  ~isempty(Open)  %line3.While: open 列表不为空
        for ii=2:length(Open)  %line4.寻找open列表中F值最小的节点，记为FMin；
            if Open(ii).F < Open(1).F
                OpenTemp=Open(ii);
                Open(ii)=Open(1);
                Open(1)=OpenTemp;
            end
        end
        Close=[Close;Open(1)]; %line5-1.将FMin加入close列表,所以FMin就是SetClose(end),同时在open列表中删除该点；
        Open(1)=[]; %line5-2.将FMin加入close列表，同时在open列表中删除该点；
        Surround=[];
        if Node_opti==1 %如果选择节点优化optimization
            % 算法4：邻近节点优选
            %输入：A*算法中的close列表
            %输出：优化后的close列表
            %%%回馈处理%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            L_Close=length(Close);
            ComPoint=[];
            if L_Close>2
                ComPoint=Close(end).father;
                while ~(ComPoint.row==start_row && ComPoint.col==start_col)
                    if ((Close(end).row-ComPoint.row)^2+(Close(end).col-ComPoint.col)^2)<(ComPoint.R)^2
                        Close(end).father=ComPoint;
                        Close(end).G=ComPoint.G+movecost+movecost*map(ComPoint.row,ComPoint.col);
                    end
                    ComPoint=ComPoint.father;
                end
            end
            Close(end).father=ComPoint;
        end
        %
        %变步长设置%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %line7. 根据 FMin 节点的惩罚值大小计算下一步应该移动的步长 ShipSpeed；
        ShipSpeed=Movelength * (1-map(Close(end).row,Close(end).col));
        if ShipSpeed<1
            ShipSpeed=1;
        end
        %          ShipSpeed=Movelength;
        %line8.计算船舶移动一步的距离代价movecost和应扩展的邻域节点数Num；
        movecost=10; %如果为变速的A*，所以movecost在这里，会改变
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        for ii=1:SurroundPointsNum  %line9. For 生成的每一个邻域节点Surround(i)；
            Surround(ii).row=floor(Close(end).row-ShipSpeed*sin((ii-1)*RudderAngle));
            Surround(ii).col=floor(Close(end).col+ShipSpeed*cos((ii-1)*RudderAngle));
            Surround(ii).R= ShipSpeed;
            Surround(ii).Dir = ShipDirection(Close(end).row,Close(end).col,Surround(ii).row,Surround(ii).col);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%%不再计算相邻点的条件
            if ~isempty( Open)
                openitear=1;
                mindis = 1000;
                while (openitear<length(Open))
                    dis=sqrt((Surround(ii).row -Open(openitear).row)^2+(Surround(ii).col-Open(openitear).col)^2);
                    if(dis<mindis)
                        mindis=dis;
                        replace=openitear;
                    end
                    openitear=openitear+1;
                end
                if (mindis<Movelength/4 && ObstacleInMove(background,Surround(ii).row,Surround(ii).col,Open(replace).row,Open(replace).col,ShipLong/2)==1)
                    %                         if (mindis<6)
                    Surround(ii).row=Open(replace).row;
                    Surround(ii).col=Open(replace).col;
                end
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % line10.If(Sourround(i)在目标区域范围之内，且Sourround(i)处不是障碍物，且Sourround(i)不在close列表中,且从FMin移动到Sourround(i)过程中船舶安全,且运动过程不受船舶运动规律限制)
            if (0>=Surround(ii).col||Surround(ii).col>=length(background(1,:))||0>=Surround(ii).row||Surround(ii).row>=length(background(:,1))...
                    || background(Surround(ii).row,Surround(ii).col)==1 ||alreadyexist(Surround(ii),Close)==1 ...
                    ||ObstacleInMove(background,Close(end).row,Close(end).col,Surround(ii).row,Surround(ii).col,ShipLong/2)==0 ...
                    ||ObstacleInDomain(background,Surround(ii).row,Surround(ii).col,ShipLong/2)==0)...
                    ||PointsCanReach(ShipSpeed,Rmin,Surround(ii).Dir,Close(end).Dir)==0
            else
                %line11. 计算Sourround(i)的G、H、F值,设置FMin为Sourround(i)的父节点；
                Surround(ii).H=sqrt((destination.col-Surround(ii).col)^2+(destination.row-Surround(ii).row)^2);
                Surround(ii).G=Close(end).G+movecost+valueAPF*movecost*map(Surround(ii).row,Surround(ii).col);%movecost用于调整势场代价值
                Surround(ii).F=Surround(ii).G+Surround(ii).H;
                Surround(ii).father=Close(end); %设置FMin为Sourround(i)的父节点；
                
                if alreadyexist(Surround(ii),Open)==0 %line12. If(Sourround(i)所在坐标不同于open列表中任意点坐标)
                    Open=[Open;Surround(ii)]; %line13. 将Sourround(i)加入open列表；
                else %line14
                    % line15.比较Sourround(i)与open列表中具有相同坐标节点的G值，设置较小者的父节点为FMin；
                    for kk=1:length(Open)
                        %                         if abs(Surround(ii).row - SetOpen(kk).row)<=1/4*ShipLong && abs(Surround(ii).col-SetOpen(kk).col)<=1/4*ShipLong
                        if (Surround(ii).row == Open(kk).row && Surround(ii).col==Open(kk).col)
                            rember=kk;                       %找到Sourround(i)与open列表中具有相同坐标的节点
                        end
                    end
                    if Surround(ii).G < Open(rember).G     %比较G值
                        Open(rember).father=Close(end); %设置较小者的父节点为FMin；
                    end
                end %line16.
            end     %line17.
        end         %line18.
        if Close(end).H < ShipSpeed %line19. 如果FMin到目标点的距离小于移动步长，算法结束；
            break;
        end
    end
    destination.father=Close(end);
    destination.Dir=ShipDirection(Close(end).row,Close(end).col,end_row,end_col);
    
    %原程序采用了嵌套的方式，最后还是要把决策的内容（每一步的位置、航向）取出来
    CurrentPoint=destination;
    PosTemp=[];
    courseTemp=[];
    posData=[];
    courseData=[];
    while ~(CurrentPoint.row==start_point.row && CurrentPoint.col==start_point.col)
        position=[CurrentPoint.row CurrentPoint.col];
        if (CurrentPoint.col>=CurrentPoint.father.col)
            ShipDir=atan((CurrentPoint.row-CurrentPoint.father.row)/(CurrentPoint.col-CurrentPoint.father.col));
        else
            ShipDir=pi+atan((CurrentPoint.row-CurrentPoint.father.row)/(CurrentPoint.col-CurrentPoint.father.col));
        end
        %         plotShip(position,ShipDirection,ShipLong/2);
        PosTemp=[PosTemp;position];
        courseTemp=[courseTemp;ShipDir];
        CurrentPoint=CurrentPoint.father;
    end
    posData=[posData;flipud(PosTemp)];
    courseData=[courseData;flipud(courseTemp)];
    Data=[posData,courseData];
    ship(i).posData=Data(:,1:2);
    ship(i).courseData=Data(:,3);
    
end

toc
disp(['本次运行时间: ',num2str(toc)]);

%把每个位置占的格子回归到点，以中间点代替
a0=(MapSize(1)-(-MapSize(1)))*1852/Re;
b0=(MapSize(2)-(-MapSize(2)))*1852/Re;
for i=1:1:1
    for j=1:1:length(ship(i).posData)
        ship(i).pos(j+1,1)=(ship(i).posData(j,1)*a0+0.5*a0)/1852-MapSize(1); %x轴从栅格坐标回归正常坐标
        ship(i).pos(j+1,2)=(ship(i).posData(j,2)*b0+0.5*b0)/1852-MapSize(2); %y轴从栅格坐标回归正常坐标
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


