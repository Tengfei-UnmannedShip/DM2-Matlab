%% A*与APF结合的第二版，A*中加上了中间点，APF成为一个函数
% 原版作者薛双飞
% 第一版程序问题：0。没有中间点??中间点应该是在APF地图画好之后的，所以，APF应该也是一个函数
% 1.地图是反的??坐标转换
% 2.颜色为黑白-是否把路径取出来放到等高线图里显示，参考classicAPF?new-main里的方法

clear all;
clc;
tic;%tic1
t1=clock;

[X,Y]=meshgrid(-300:1:300,-300:1:300);
[m,n]=size(X);
%%==================================================
%环境船舶参数设置
% 基本模型参考桥墩计算方法
% ==================================================
Boat_Speed_Factor=1;        %速度方向势场衰减因子，取值越大在速度方向上影响越大
BoatRiskFieldPeakValue=100;   %风险最大值可以根据需要随意设置
Boat_eta=1;  %改变影响范围，或曲线下降坡度，越大范围越小
Boat_alfa=0.1; %改变影响范围，或曲线下降坡度，越大范围越小
BoatCut=0;
RiskFieldValue=zeros(m,n);%wtf--Z=RiskFieldValue，即每一点的势场值。此处先将其归零为一个与X相同大小的0矩阵

U_att=zeros(m,n);
APFValue=zeros(m,n);

Boat_Num=3;%船舶数量
%观察者位置
Boat.State(1,:)=[   0   220   180     5  30  10];  %船舶状态坐标（x,y,ang,v,l,w）ang为船舶航向 v为速度大小,l为船长，w为船宽
Boat.State(2,:)=[-200    50   120     5  20   6];  %船舶状态坐标（x,y,ang,v）ang为船舶航向
Boat.State(3,:)=[ 150    70  -120     8  10   3];
Boat.State(4,:)=[ 120  -150   -45    10  20   5]; %ship4是本船
%Boat.State(5,:)=[-70   -15    90    10  20   5];
goal=[-280  250];%此时本船在（120，-150）角度-45
k_near=10;%计算目标点附近引力需要的增益系数
k_far=10; %计算引力门槛之后引力恒定时的增益系数，一般与k_near相同，否则在门槛处会出现引力场突变
Po_att=100;%引力门槛半径
k=0.3;%引力系数
d_goal=sqrt((X-goal(1,1)).^2+(Y-goal(1,2)).^2);
% U_att=0.5*k_near*d_obs_APF.^2;%不加引力门槛的

% U_att=0.5*k_near*d_goal.^2.*(d_goal<=Po_att)+k_far*Po_att*d_goal.*(d_goal>Po_att);%加引力门槛的
% U_att=d_goal.*k ;  %引力为恒力的，引力场是距离的一次函数

for i=1:1:Boat_Num
    
    %航标参数初始化
    Boat_x(i)=Boat.State(i,1);                  %第i个船x坐标
    Boat_y(i)=Boat.State(i,2);                  %第i个船y坐标
    Boat_theta(i)=-Boat.State(i,3)/180*pi;       %第i个船艏向角度
    Boat_Speed(i)=Boat.State(i,4);              %第i个船速度大小
    Boat_length(i)=Boat.State(i,5);             %第i个船度
    Boat_width(i)=Boat.State(i,6);              %第i个船宽
    
    %把计算势场点坐标（X,Y）变换到船舶坐标系下点（BoatX{i},BoatY{i}）
    BoatX{i} = (X-Boat_x(i))*cos(Boat_theta(i))+(Y-Boat_y(i))*sin(Boat_theta(i));
    BoatY{i} = (Y-Boat_y(i))*cos(Boat_theta(i))-(X-Boat_x(i))*sin(Boat_theta(i));
    
    BoatSpeedFactor{i}=Boat_Speed_Factor*Boat_Speed(i);
    
    %计算空间中点到第i个船边沿距离Dis{i}
    Dis{i}=zeros(m,n);
    Dis{i}=sqrt(BoatX{i}.^2+BoatY{i}.^2).*(BoatY{i}<=0)+sqrt((BoatY{i}/(Boat_Speed(i)+1)).^2+BoatX{i}.^2).*(BoatY{i}>0);
    
    %计算第i个船风险场
    BoatRiskField{i}=BoatRiskFieldPeakValue.*(exp(-Boat_eta*Boat_alfa*Dis{i})./(Boat_alfa*Dis{i}));
    
    if  BoatRiskField{i}>BoatRiskFieldPeakValue
        BoatRiskField{i}=BoatRiskFieldPeakValue;
    end
    BoatRiskField{i}(BoatRiskField{i}>BoatRiskFieldPeakValue)=BoatRiskFieldPeakValue;
    
    if BoatCut==1
        BoatRiskField{i}=BoatRiskField{i}.*(BoatX{i}>=0)+0.*(BoatX{i}<0);
    end
    
    %这里每个点在不同船下的场之间暂采用简单加和
    RiskFieldValue=RiskFieldValue+BoatRiskField{i};
end
% APFValue=max(U_att,RiskFieldValue);
APFValue=RiskFieldValue;
newfield=RiskFieldValue/BoatRiskFieldPeakValue;
figure;
mesh(X,Y,APFValue);
hold on;
plot(goal(1,1),goal(1,2),'ro','MarkerFaceColor','r');
%     hold on;
%     ship_icon(Boat.State(4,1),Boat.State(4,2),Boat.State(4,5), Boat.State(4,6), Boat.State(4,3),0 );
axis equal;
axis off;
%     surf(X,Y,APFValue);
figure
contourf(X,Y,APFValue,'LevelStep',30);  %带填充颜色的等高线图
hold on;
plot(goal(1,1),goal(1,2),'ro','MarkerFaceColor','r');
hold on;
ship_icon(Boat.State(4,1),Boat.State(4,2),Boat.State(4,5), Boat.State(4,6), Boat.State(4,3),0 );

%     plot(goal(1,1),goal(1,2),'rx');
axis equal;
axis off;

% newfield1=zeros(size(newfield));
%% A*算法
% [SetClose,SetOpen]=CircleAStar(newfield,300,500,500,100);%直接调用函数，% 原版作者薛双飞

% function [SetClose,SetOpen]=CircleAStar(map,start_row,start_col,end_row,end_col)
% 算法5:多向A*算法
%输入: 惩罚图像(矩阵)map,起点图像坐标(start_row,start_col),目标点图像坐标(destination_row, destination_col),船舶长度ShipLong,旋回半斤Rmin
%输出: 搜索过的点集open列表，被选为最优路径节点的点集close列表
% line1. 设置初始船舶艏向；
background=newfield;
map=newfield;
MapSize=[300,300];
% start_row=300; 
% start_col=500; 
start_row=round(Boat.State(4,1)+MapSize(1)); 
start_col=round(Boat.State(4,2)+MapSize(2)); 
start_point.row=start_row;
start_point.col=start_col;
GoalRange=MapSize-[20,20];
%     ship(j).goalPiont= Goal_point(ship(j).pos(1),ship(j).pos(2),ship(j).Course,GoalRange);
goalPiont=Goal_point1(Boat.State(4,1),Boat.State(4,2),Boat.State(4,3),GoalRange);
% end_row=500; 
% end_col=100; 
end_row=round(goalPiont(1)+MapSize(1)); 
end_col=round(goalPiont(2)+MapSize(2)); 
destination.row=end_row;
destination.col=end_col;
ShipLong=4;
Movelength=20;  %步长
SurroundPointsNum=20; %跳整方向数，n向的A*
RudderAngle=2*pi/SurroundPointsNum;
Rmin=2*Movelength/3; %转弯半径
valueAPF=2;  %APF势场的价值函数
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
    start_point.Dir=pi/2;  %起始点艏向
    SetOpen(1)=start_point; %起始点坐标
    SetOpen(1).father=nan; %父节点
    SetClose(1)=SetOpen(1); %并将该节点放到open表中,初始化close列表为空；
end
% 开始计算
while  ~isempty(SetOpen)  %line3.While: open 列表不为空
    for ii=2:length(SetOpen)  %line4.寻找open列表中F值最小的节点，记为FMin；
        if SetOpen(ii).F < SetOpen(1).F
            a=SetOpen(ii);
            SetOpen(ii)=SetOpen(1);
            SetOpen(1)=a;
        end
    end
    SetClose=[SetClose;SetOpen(1)]; %line5-1.将FMin加入close列表,所以FMin就是SetClose(end),同时在open列表中删除该点；
    SetOpen(1)=[]; %line5-2.将FMin加入close列表，同时在open列表中删除该点；
    Surround=[];
    %         %% 算法4：邻近节点优选
    %         %输入：A*算法中的close列表
    %         %输出：优化后的close列表
    %         %%%回馈处理%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %         L_Close=length(SetClose);
    %         ComPoint=[];
    %         if L_Close>2
    %             ComPoint=SetClose(end).father;
    %             while ~(ComPoint.row==start_row && ComPoint.col==start_col)
    %                 if ((SetClose(end).row-ComPoint.row)^2+(SetClose(end).col-ComPoint.col)^2)<(ComPoint.R)^2
    %                     SetClose(end).father=ComPoint;
    %                     SetClose(end).G=ComPoint.G+movecost+movecost*map(ComPoint.row,ComPoint.col);
    %                 end
    %                 ComPoint=ComPoint.father;
    %             end
    %         end
    %         SetClose(end).father=ComPoint;
    
    %变步长设置%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %line7. 根据 FMin 节点的惩罚值大小计算下一步应该移动的步长 ShipSpeed；
    ShipSpeed=Movelength * (1-map(SetClose(end).row,SetClose(end).col));
    if ShipSpeed<1
        ShipSpeed=1;
    end
    %          ShipSpeed=Movelength;
    %line8.计算船舶移动一步的距离代价movecost和应扩展的邻域节点数Num；
    movecost=10; %如果为变速的A*，所以movecost在这里，会改变
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for ii=1:SurroundPointsNum  %line9. For 生成的每一个邻域节点Surround(i)；
        Surround(ii).row=floor(SetClose(end).row-ShipSpeed*sin((ii-1)*RudderAngle));
        Surround(ii).col=floor(SetClose(end).col+ShipSpeed*cos((ii-1)*RudderAngle));
        Surround(ii).R= ShipSpeed;
        Surround(ii).Dir = ShipDirection(SetClose(end).row,SetClose(end).col,Surround(ii).row,Surround(ii).col);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%不再计算相邻点的条件
        if ~isempty( SetOpen)
            openitear=1;
            mindis = 1000;
            while (openitear<length(SetOpen))
                dis=sqrt((Surround(ii).row -SetOpen(openitear).row)^2+(Surround(ii).col-SetOpen(openitear).col)^2);
                if(dis<mindis)
                    mindis=dis;
                    replace=openitear;
                end
                openitear=openitear+1;
            end
            if (mindis<Movelength/4 && ObstacleInMove(background,Surround(ii).row,Surround(ii).col,SetOpen(replace).row,SetOpen(replace).col,ShipLong/2)==1)
                %                         if (mindis<6)
                Surround(ii).row=SetOpen(replace).row;
                Surround(ii).col=SetOpen(replace).col;
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % line10.If(Sourround(i)在目标区域范围之内，且Sourround(i)处不是障碍物，且Sourround(i)不在close列表中,且从FMin移动到Sourround(i)过程中船舶安全,且运动过程不受船舶运动规律限制)
        if (0>=Surround(ii).col||Surround(ii).col>=length(background(1,:))||0>=Surround(ii).row||Surround(ii).row>=length(background(:,1))...
                || background(Surround(ii).row,Surround(ii).col)==1 ||alreadyexist(Surround(ii),SetClose)==1 ...
                ||ObstacleInMove(background,SetClose(end).row,SetClose(end).col,Surround(ii).row,Surround(ii).col,ShipLong/2)==0 ...
                ||ObstacleInDomain(background,Surround(ii).row,Surround(ii).col,ShipLong/2)==0)...
                ||PointsCanReach(ShipSpeed,Rmin,Surround(ii).Dir,SetClose(end).Dir)==0
        else
            %line11. 计算Sourround(i)的G、H、F值,设置FMin为Sourround(i)的父节点；
            Surround(ii).H=sqrt((destination.col-Surround(ii).col)^2+(destination.row-Surround(ii).row)^2);
            Surround(ii).G=SetClose(end).G+movecost+valueAPF*movecost*map(Surround(ii).row,Surround(ii).col);%movecost用于调整势场代价值
            Surround(ii).F=Surround(ii).G+Surround(ii).H;
            Surround(ii).father=SetClose(end); %设置FMin为Sourround(i)的父节点；
            
            if alreadyexist(Surround(ii),SetOpen)==0 %line12. If(Sourround(i)所在坐标不同于open列表中任意点坐标)
                SetOpen=[SetOpen;Surround(ii)]; %line13. 将Sourround(i)加入open列表；
            else %line14
                % line15.比较Sourround(i)与open列表中具有相同坐标节点的G值，设置较小者的父节点为FMin；
                for kk=1:length(SetOpen)
                    %                         if abs(Surround(ii).row - SetOpen(kk).row)<=1/4*ShipLong && abs(Surround(ii).col-SetOpen(kk).col)<=1/4*ShipLong
                    if (Surround(ii).row == SetOpen(kk).row && Surround(ii).col==SetOpen(kk).col)
                        rember=kk;                       %找到Sourround(i)与open列表中具有相同坐标的节点
                    end
                end
                if Surround(ii).G < SetOpen(rember).G     %比较G值
                    SetOpen(rember).father=SetClose(end); %设置较小者的父节点为FMin；
                end
            end %line16.
        end     %line17.
    end         %line18.
    if SetClose(end).H < ShipSpeed %line19. 如果FMin到目标点的距离小于移动步长，算法结束；
        break;
    end
end
destination.father=SetClose(end);
destination.Dir=ShipDirection(SetClose(end).row,SetClose(end).col,end_row,end_col);
%绘制路径
figure
%     background=K;
imshow(background);
rectangle('position',[1 1 size(background)-1],'edgecolor','k')%设置图片边框大小及颜色
t=1;
M(t)=getframe;
t=t+1;

background2(:,:,1)=background;background2(:,:,2)=background;background2(:,:,3)=background;
CurrentPoint=destination;

while ~(CurrentPoint.row==start_point.row && CurrentPoint.col==start_point.col)
    position=[CurrentPoint.row CurrentPoint.col];
    %         if (CurrentPoint.col>=CurrentPoint.father.col)
    %             ShipDirection=atan((CurrentPoint.row-CurrentPoint.father.row)/(CurrentPoint.col-CurrentPoint.father.col));
    %         else
    %             ShipDirection=pi+atan((CurrentPoint.row-CurrentPoint.father.row)/(CurrentPoint.col-CurrentPoint.father.col));
    %         end
    %         plotShip(position,ShipDirection,ShipLong/2);
    plotShip(position,CurrentPoint.Dir,ShipLong/2);
    CurrentPoint=CurrentPoint.father;
    M(t)=getframe;t=t+1;
end
line([start_point.col-3;start_point.col+3;start_point.col+3;start_point.col-3;start_point.col-3],[start_point.row-3;start_point.row-3;start_point.row+3;start_point.row+3;start_point.row-3],'color','g','LineWidth',5);
line([destination.col-3;destination.col+3;destination.col+3;destination.col-3;destination.col-3],[destination.row-3;destination.row-3;destination.row+3;destination.row+3;destination.row-3],'color','b','LineWidth',5);
disp(['程序总运行时间：',num2str(etime(clock,t1))]);


