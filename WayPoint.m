function  WayPoint = WayPoint( OS,TS)
%% 一对一的路径点计算
% OS眼中每一个TS相对于它的目标路径点，输出为船头+船尾
% 基本的CPA计算但是多了一个航向时间，用于Jinfen的算法中计算每一阶段的CPA
% 将本船和目标船的速度由(速度值+航行角)的极坐标形式转化为(Vx,Vy)的直角坐标形式
WayPoint=[];
time=1000;
for i=1:1:length(TS)
    course_target=TS(i).Course;
    TSlength=TS(i).length;
    
    CPA = computeCPA0( OS,TS(i),time);
    PosTS=CPA(1,3:4);
    x0=PosTS(1);
    y0=PosTS(2);
    x1=2*TSlength*sind(course_target)+x0;       %船头目标点x坐标，船头2倍船长处
    y1=2*TSlength*cosd(course_target)+y0;       %船头目标点y坐标，船头2倍船长处
    x2=x0-1.5*TSlength*sind(course_target);     %船尾目标点x坐标，船尾1.5倍船长处
    y2=y0-1.5*TSlength*cosd(course_target);     %船尾目标点y坐标，船尾1.5倍船长处
    
    WayPoint=[WayPoint;
              x1 y1 x2 y2];
end
end

