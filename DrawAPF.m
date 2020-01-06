function  APFmap= DrawAPF( OS, TS, map, draw )
%人工势场底图绘制函数，来自旧的人工势场程序

%% 地图设置
% map=[x1,x2,y1,y2],表示4个边界，单位海里
x1=-map(1)*1852;
x2=map(1)*1852;
y1=-map(2)*1852;
y2=map(2)*1852;

[APF_X,APF_Y]=meshgrid(x1:10:x2,y1:10:y2);
[m,n]=size(APF_X);
%% ==================================================
% 目标船舶参数设置
% 基本模型参考桥墩计算方法
% ==================================================
%当进入实际大小之后，可能会需要比较大一些的参数，不然几乎没有显示
Boat_Speed_Factor=1;           %速度方向势场衰减因子，取值越大在速度方向上影响越大
BoatRiskFieldPeakValue=100;    %风险最大值可以根据需要随意设置
Boat_eta=1;
Boat_alfa=0.1;
BoatCut=0;
RiskFieldValue=zeros(m,n);%wtf--Z=RiskFieldValue，即每一点的势场值。此处先将其归零为一个与X相同大小的0矩阵
APFmap=zeros(m,n);

for i=1:1:length(TS)
    %航标参数初始化
    Boat_x(i)=TS(i).pos(end,1);                   %第i个船x坐标
    Boat_y(i)=TS(i).pos(end,2);                   %第i个船y坐标
    Boat_theta(i)=-TS(i).Course(end,:)/180*pi;       %第i个船艏向角度
    Boat_Speed(i)=TS(i).speed(end,:);                %第i个船速度大小
    Boat_length(i)=TS(i).length;              %第i个船长
    Boat_width(i)=TS(i).width;                %第i个船宽
    %局部坐标系转换，把计算势场点坐标（X,Y）变换到船舶坐标系下点（BoatX{i},BoatY{i}）
    BoatX{i} = (APF_X-Boat_x(i))*cos(Boat_theta(i))+(APF_Y-Boat_y(i))*sin(Boat_theta(i));
    BoatY{i} = (APF_Y-Boat_y(i))*cos(Boat_theta(i))-(APF_X-Boat_x(i))*sin(Boat_theta(i));
    
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
APFmap=RiskFieldValue;
% APFValue=RiskFieldValue;
% newfield=RiskFieldValue/BoatRiskFieldPeakValue;
% APF.X=APF_X;
% APF.Y=APF_Y;
% APF.map0=APFValue;
% APF.map1=newfield;

%% 绘图程序，如果每一次都绘图的话，会太多，只在一些时刻画图，控制函数为draw
if draw==1  %确认绘图
    figure;
    mesh(APF_X,APF_Y,APFmap);
    hold on;
    plot(goal(1,1),goal(1,2),'ro','MarkerFaceColor','r');
    axis equal;
    axis off;
    figure
    contourf(APF_X,APF_Y,APFmap,'LevelStep',30);  %带填充颜色的等高线图
    hold on;
    plot(goal(1,1),goal(1,2),'ro','MarkerFaceColor','r');
    hold on;
    ship_icon(OS.pos(1),OS.pos(2),OS.length,OS.width, OS.Course,0 ); %这里使用ship_icon函数，显示出来的可能会有点小
    axis equal;
    axis off;
end
end

