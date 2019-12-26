function  WayPoint = WayPoint0( OS,TS,time )
%% OS����ÿһ��TS���������Ŀ��·���㣬���Ϊ��ͷ+��β
% ������CPA���㵫�Ƕ���һ������ʱ�䣬����Jinfen���㷨�м���ÿһ�׶ε�CPA
% ��������Ŀ�괬���ٶ���(�ٶ�ֵ+���н�)�ļ�������ʽת��Ϊ(Vx,Vy)��ֱ��������ʽ
WayPoint=[];
for i=1:1:length(TS)
    course_target=TS(i).Course;
    TSlength=TS(i).length;
    
    CPA = computeCPA0( OS,TS(i),time);
    PosTS=CPA(1,3:4);
    x0=PosTS(1);
    y0=PosTS(2);
    x1=2*TSlength*sind(course_target)+x0;       %��ͷĿ���x���꣬��ͷ2��������
    y1=2*TSlength*cosd(course_target)+y0;       %��ͷĿ���y���꣬��ͷ2��������
    x2=x0-1.5*TSlength*sind(course_target);     %��βĿ���x���꣬��β1.5��������
    y2=y0-1.5*TSlength*cosd(course_target);     %��βĿ���y���꣬��β1.5��������
    
    WayPoint=[WayPoint;
              x1 y1 x2 y2];
end
end

