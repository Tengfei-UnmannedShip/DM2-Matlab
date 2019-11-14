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
MapSize=[10,10];  %每一艘船最大的活动范围
t=2500;
t1=0;
tt=2000;
for j=1:1:ShipNum
    
    ship(j).speeds = boat(j,3)/3600;%WTF：将航速转换为海里／秒
    ship(j).speed0 = boat(j,3);%WTF：原航速，用于ship domain
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
    ship(j).Start_pos=[boat(j,1)-ship(j).speeds*sind(ship(j).initialCourse)*0.5*t, boat(j,2)-ship(j).speeds*cosd(ship(j).initialCourse)*0.5*t];
    ship(j).pos1=[ship(j).Start_pos(1)+ship(j).speeds*sind(ship(j).Course)*t1, ship(j).Start_pos(1)+ship(j).speeds*cosd(ship(j).Course)*t1];
    ship(j).DCPA_Record = [];
    ship(j).TCPA_Record = [];
    ship(j).length=boatsize(j,1);
    ship(j).width=boatsize(j,2);
    GoalRange=MapSize-[1.5,1.5];
    ship(j).goalPiont0= Goal_point(ship(j).Start_pos(1),ship(j).Start_pos(2),ship(j).Course,GoalRange);
end

%% APF绘图程序
[X,Y]=meshgrid(-MapSize(1):0.01:MapSize(1),-MapSize(2):0.01:MapSize(2));
[m,n]=size(X);
%%==================================================
%环境船舶参数设置
% 基本模型参考桥墩计算方法
% ==================================================

ShipDomainMap=zeros(m,n);
SCR=zeros(m,n);
Boat_Num=4;%船舶数量

for i=1:1:Boat_Num
    
    %航标参数初始化
    Boat_x(i)=ship(i).Start_pos(1);                  %第i个船x坐标
    Boat_y(i)=ship(i).Start_pos(2);                 %第i个船y坐标
    Boat_theta0(i)=ship(i).Course;              %转换前第i个船艏向角度
    Boat_theta(i)=-ship(i).Course/180*pi;       %第i个船艏向角度
    Boat_Speed(i)=ship(i).speeds*ship(i).ratio;  %第i个船速度大小
    Boat_length(i)=ship(i).length;              %第i个船度
    Boat_width(i)=ship(i).width;                %第i个船宽
    
    %把计算势场点坐标（X,Y）变换到船舶坐标系下点（BoatX{i},BoatY{i}）
    BoatX{i} = (X-Boat_x(i))*cos(Boat_theta(i))+(Y-Boat_y(i))*sin(Boat_theta(i));
    BoatY{i} = (Y-Boat_y(i))*cos(Boat_theta(i))-(X-Boat_x(i))*sin(Boat_theta(i));
    
    r0 = 0.5;
    L = Boat_length(i)/1852;
    k=2;
    Vos = ship(i).speed0;
    k_AD = 10^(0.3591*log10(Vos)+0.0952);
    k_DT = 10^(0.5441*log10(Vos)-0.0795);
    
    Rfore = (1+1.34*sqrt(k_AD^2+0.25*k_DT^2))*L;
    Raft = (1+0.67*sqrt(k_AD^2+0.25*k_DT^2))*L;
    Rstarb = (0.2+k_DT)*L;
    Rport = (0.2+0.75*k_DT)*L;
    R = [Rfore Raft Rstarb Rport];
    Sigma = R/((log(1/r0))^(1/k));
    a=Sigma(1);
    
    SCR= exp(-(2*BoatY{i}./((1+sign(BoatY{i}))*Sigma(1)+(1-sign(BoatY{i}))*Sigma(2))).^k-(2*BoatX{i}./((1+sign(BoatX{i}))*Sigma(3)+(1-sign(BoatX{i}))*Sigma(4))).^k);
    ShipDomainMap=ShipDomainMap+SCR;
end

figure
mesh(X,Y,ShipDomainMap);
% axis on;
axis equal;
xlabel('\it n miles', 'Fontname', 'Times New Roman');
ylabel('\it n miles', 'Fontname', 'Times New Roman');

figure
mesh(X,Y,ShipDomainMap);
hold on
surfc(X,Y,-5+0*ShipDomainMap,ShipDomainMap,'edgecolor','none','facecolor','interp') %不带填充颜色的等高线图
axis equal
xlabel('\it n miles', 'Fontname', 'Times New Roman');
ylabel('\it n miles', 'Fontname', 'Times New Roman');

figure
contourf(X,Y,ShipDomainMap,'LineStyle','none');  %带填充颜色的等高线图
axis equal;
xlabel('\it n miles', 'Fontname', 'Times New Roman');
ylabel('\it n miles', 'Fontname', 'Times New Roman');

% figure
% contourf(X,Y,ShipDomainMap,'LineStyle','none');  %带填充颜色的等高线图
% % for i=1:1:4
% %     hold on;
% %     plot(ship(1).goalPiont0(1),ship(1).goalPiont0(2),'ro','MarkerFaceColor','r');
% %     hold on;
% %     ship_icon(ship(1).Start_pos(1),ship(1).Start_pos(2),ship(1).length, ship(1).width, ship(1).Course,0 );
% %     plot(goal(1,1),goal(1,2),'rx');
% % end
% % axis on;
% axis equal;

% % newfield1=zeros(size(ShipDomainMap));
% %% A*算法
% % [SetClose,SetOpen]=CircleAStar(newfield,300,500,500,100);%直接调用函数，% 原版作者薛双飞
% 
% % function [SetClose,SetOpen]=CircleAStar(map,start_row,start_col,end_row,end_col)
% % 算法5:多向A*算法
% %输入: 惩罚图像(矩阵)map,起点图像坐标(start_row,start_col),目标点图像坐标(destination_row, destination_col),船舶长度ShipLong,旋回半斤Rmin
% %输出: 搜索过的点集open列表，被选为最优路径节点的点集close列表
% % line1. 设置初始船舶艏向；
% background=ShipDomainMap;
% map=ShipDomainMap;
% start_row=round((ship(4).Start_pos(1)+MapSize(1))*100);
% start_col=round((ship(4).Start_pos(2)+MapSize(2))*100);
% start_point.row=start_row;
% start_point.col=start_col;
% end_row=round((ship(4).goalPiont0(1)+MapSize(1))*100);
% end_col=round((ship(4).goalPiont0(2)+MapSize(2))*100);
% 
% destination.row=end_row;
% destination.col=end_col;
% Movelength=round(ship(4).speed0/10);
% ShipLong=round(ship(4).length);
% SurroundPointsNum=20; %跳整方向数，n向的A*
% RudderAngle=2*pi/SurroundPointsNum;
% Rmin=2*Movelength/3; %转弯半径
% valueAPF=10;  %APF势场的价值函数
% %     Rmin=0;
% 
% %开始计算
% % line2. 初始准备：
% %如果船舶位置在地图范围之外或者船舶状态不安全，算法结束，提示无安全路径，
% %否则计算起始节点各属性(坐标、艏向、惩罚值、移动代价G、到目标点的预计代价H、总代价F、下一步移动距离r、父节点、子节点等)
% %并将该节点放到open表中,初始化close列表为空；
% if (0<start_point.col<length(background(1,:))&&0<start_point.row<length(background(:,1)))
%     start_point.G=0; %移动代价 G
%     start_point.H=sqrt((destination.col-start_point.col)^2+(destination.row-start_point.row)^2);  %到目标点的预计代价H
%     start_point.F=start_point.G+start_point.H; %总代价F
%     start_point.R= Movelength; %下一步移动距离r
%     start_point.Dir=pi/2;  %起始点艏向
%     SetOpen(1)=start_point; %起始点坐标
%     SetOpen(1).father=nan; %父节点
%     SetClose(1)=SetOpen(1); %并将该节点放到open表中,初始化close列表为空；
% end
% % 开始计算
% while  ~isempty(SetOpen)  %line3.While: open 列表不为空
%     for ii=2:length(SetOpen)  %line4.寻找open列表中F值最小的节点，记为FMin；
%         if SetOpen(ii).F < SetOpen(1).F
%             a=SetOpen(ii);
%             SetOpen(ii)=SetOpen(1);
%             SetOpen(1)=a;
%         end
%     end
%     SetClose=[SetClose;SetOpen(1)]; %line5-1.将FMin加入close列表,所以FMin就是SetClose(end),同时在open列表中删除该点；
%     SetOpen(1)=[]; %line5-2.将FMin加入close列表，同时在open列表中删除该点；
%     Surround=[];
%     %         %% 算法4：邻近节点优选
%     %         %输入：A*算法中的close列表
%     %         %输出：优化后的close列表
%     %         %%%回馈处理%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     %         L_Close=length(SetClose);
%     %         ComPoint=[];
%     %         if L_Close>2
%     %             ComPoint=SetClose(end).father;
%     %             while ~(ComPoint.row==start_row && ComPoint.col==start_col)
%     %                 if ((SetClose(end).row-ComPoint.row)^2+(SetClose(end).col-ComPoint.col)^2)<(ComPoint.R)^2
%     %                     SetClose(end).father=ComPoint;
%     %                     SetClose(end).G=ComPoint.G+movecost+movecost*map(ComPoint.row,ComPoint.col);
%     %                 end
%     %                 ComPoint=ComPoint.father;
%     %             end
%     %         end
%     %         SetClose(end).father=ComPoint;
% 
%     %变步长设置%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     %line7. 根据 FMin 节点的惩罚值大小计算下一步应该移动的步长 ShipSpeed；
%     ShipSpeed=Movelength * (1-map(SetClose(end).row,SetClose(end).col));
%     if ShipSpeed<1
%         ShipSpeed=1;
%     end
%     %          ShipSpeed=Movelength;
%     %line8.计算船舶移动一步的距离代价movecost和应扩展的邻域节点数Num；
%     movecost=10; %如果为变速的A*，所以movecost在这里，会改变
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     for ii=1:SurroundPointsNum  %line9. For 生成的每一个邻域节点Surround(i)；
%         Surround(ii).row=floor(SetClose(end).row-ShipSpeed*sin((ii-1)*RudderAngle));
%         Surround(ii).col=floor(SetClose(end).col+ShipSpeed*cos((ii-1)*RudderAngle));
%         Surround(ii).R= ShipSpeed;
%         Surround(ii).Dir = ShipDirection(SetClose(end).row,SetClose(end).col,Surround(ii).row,Surround(ii).col);
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         %%%不再计算相邻点的条件
%         if ~isempty( SetOpen)
%             openitear=1;
%             mindis = 1000;
%             while (openitear<length(SetOpen))
%                 dis=sqrt((Surround(ii).row -SetOpen(openitear).row)^2+(Surround(ii).col-SetOpen(openitear).col)^2);
%                 if(dis<mindis)
%                     mindis=dis;
%                     replace=openitear;
%                 end
%                 openitear=openitear+1;
%             end
%             if (mindis<Movelength/4 && ObstacleInMove(background,Surround(ii).row,Surround(ii).col,SetOpen(replace).row,SetOpen(replace).col,ShipLong/2)==1)
%                 %                         if (mindis<6)
%                 Surround(ii).row=SetOpen(replace).row;
%                 Surround(ii).col=SetOpen(replace).col;
%             end
%         end
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         % line10.If(Sourround(i)在目标区域范围之内，且Sourround(i)处不是障碍物，且Sourround(i)不在close列表中,且从FMin移动到Sourround(i)过程中船舶安全,且运动过程不受船舶运动规律限制)
%         if (0>=Surround(ii).col||Surround(ii).col>=length(background(1,:))||0>=Surround(ii).row||Surround(ii).row>=length(background(:,1))...
%                 || background(Surround(ii).row,Surround(ii).col)==1 ||alreadyexist(Surround(ii),SetClose)==1 ...
%                 ||ObstacleInMove(background,SetClose(end).row,SetClose(end).col,Surround(ii).row,Surround(ii).col,ShipLong/2)==0 ...
%                 ||ObstacleInDomain(background,Surround(ii).row,Surround(ii).col,ShipLong/2)==0)...
%                 ||PointsCanReach(ShipSpeed,Rmin,Surround(ii).Dir,SetClose(end).Dir)==0
%         else
%             %line11. 计算Sourround(i)的G、H、F值,设置FMin为Sourround(i)的父节点；
%             Surround(ii).H=sqrt((destination.col-Surround(ii).col)^2+(destination.row-Surround(ii).row)^2);
%             Surround(ii).G=SetClose(end).G+movecost+valueAPF*movecost*map(Surround(ii).row,Surround(ii).col);%movecost用于调整势场代价值
%             Surround(ii).F=Surround(ii).G+Surround(ii).H;
%             Surround(ii).father=SetClose(end); %设置FMin为Sourround(i)的父节点；
% 
%             if alreadyexist(Surround(ii),SetOpen)==0 %line12. If(Sourround(i)所在坐标不同于open列表中任意点坐标)
%                 SetOpen=[SetOpen;Surround(ii)]; %line13. 将Sourround(i)加入open列表；
%             else %line14
%                 % line15.比较Sourround(i)与open列表中具有相同坐标节点的G值，设置较小者的父节点为FMin；
%                 for kk=1:length(SetOpen)
%                     %                         if abs(Surround(ii).row - SetOpen(kk).row)<=1/4*ShipLong && abs(Surround(ii).col-SetOpen(kk).col)<=1/4*ShipLong
%                     if (Surround(ii).row == SetOpen(kk).row && Surround(ii).col==SetOpen(kk).col)
%                         rember=kk;                       %找到Sourround(i)与open列表中具有相同坐标的节点
%                     end
%                 end
%                 if Surround(ii).G < SetOpen(rember).G     %比较G值
%                     SetOpen(rember).father=SetClose(end); %设置较小者的父节点为FMin；
%                 end
%             end %line16.
%         end     %line17.
%     end         %line18.
%     if SetClose(end).H < ShipSpeed %line19. 如果FMin到目标点的距离小于移动步长，算法结束；
%         break;
%     end
% end
% destination.father=SetClose(end);
% destination.Dir=ShipDirection(SetClose(end).row,SetClose(end).col,end_row,end_col);
% CurrentPoint=destination;
% decision=[];
% while ~(CurrentPoint.row==start_point.row && CurrentPoint.col==start_point.col)
%     Dec_pos=[CurrentPoint.row CurrentPoint.col];
%     Dec_course=CurrentPoint.Dir;
%     decision=[decision;Dec_pos,Dec_course];
% end
% 
% % %绘制路径
% % figure
% % %     background=K;
% % imshow(background);
% % rectangle('position',[1 1 size(background)-1],'edgecolor','k')%设置图片边框大小及颜色
% % t=1;
% % M(t)=getframe;
% % t=t+1;
% % 
% % background2(:,:,1)=background;
% % background2(:,:,2)=background;
% % background2(:,:,3)=background;
% % CurrentPoint=destination;
% % 
% % while ~(CurrentPoint.row==start_point.row && CurrentPoint.col==start_point.col)
% %     position=[CurrentPoint.row CurrentPoint.col];
% %     plot(position,'o','Color','y');
% %     CurrentPoint=CurrentPoint.father;
% %     M(t)=getframe;
% %     t=t+1;
% % end
% % plot([start_point.col,start_point.row],'*','Color','w');
% % plot([destination.col,destination.row],'*','Color','w');
% disp(['程序总运行时间：',num2str(etime(clock,t1))]);

