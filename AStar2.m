function Data=AStar2(map,start_row,start_col,end_row,end_col,speed,OSLength,Course,dire,Node_opti)
% 根据薛双飞的原A星算法改编，根据本程序的要求增加了若干函数，修改了部分参数
% 算法5:多向A*算法
%输入: 惩罚图像(矩阵)map,起点图像坐标(start_row,start_col),目标点图像坐标(destination_row, destination_col),船舶长度ShipLong,旋回半斤Rmin
%输出: 搜索过的点集open列表，被选为最优路径节点的点集close列表

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
end
