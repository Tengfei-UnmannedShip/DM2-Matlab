function  WayPoint = WayPoint1( OS,TS,time )
%% �ɱ�·�������
% OS����ÿһ��TS���������Ŀ��·���㣬���Ϊ��ͷ+��β
% ������CPA���㵫�Ƕ���һ������ʱ�䣬����Jinfen���㷨�м���ÿһ�׶ε�CPA
% ��������Ŀ�괬���ٶ���(�ٶ�ֵ+���н�)�ļ�������ʽת��Ϊ(Vx,Vy)��ֱ��������ʽ
WayPoint=[];


for i=1:1:length(TS)
    course_target=TS(i).Course;
    TSlength=TS(i).length;
    CPA = computeCPA0( OS,TS(i),time);
    dis=abs(OS.pos,
    

    PosTS=CPA(1,3:4);
    x0=PosTS(1);
    y0=PosTS(2);
    x1=a1*TSlength*sind(course_target)+x0;       %��ͷĿ���x���꣬��ͷa1��������
    y1=a1*TSlength*cosd(course_target)+y0;       %��ͷĿ���y���꣬��ͷa1��������
    x2=x0-a2*TSlength*sind(course_target);     %��βĿ���x���꣬��βa2��������
    y2=y0-a2*TSlength*cosd(course_target);     %��βĿ���y���꣬��βa2��������
    
    WayPoint=[WayPoint;
              x1 y1 x2 y2];
end
end

