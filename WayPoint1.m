function  WayPoint = WayPoint1( OS,TS )
%% �ɱ�·�������
% OS����ÿһ��TS���������Ŀ��·���㣬���Ϊ��ͷ+��β
% ������CPA���㵫�Ƕ���һ������ʱ�䣬����Jinfen���㷨�м���ÿһ�׶ε�CPA
% ��������Ŀ�괬���ٶ���(�ٶ�ֵ+���н�)�ļ�������ʽת��Ϊ(Vx,Vy)��ֱ��������ʽ
WayPoint=[];
time=1000;

for i=1:1:length(TS)
    course_target=TS(i).Course;
    TSlength=TS(i).length;
    CPA = computeCPA0( OS,TS(i),time);
    dis=norm(OS.pos(end,:)-TS(i).pos(end,:));
    if dis<200
        a1=200/TSlength;  %���������ʱ�򰴴�ͷ200�ף���β100�����
        a2=100/TSlength;
    elseif dis>200 && dis<500
        a1=500/TSlength;  %���������ʱ�򰴴�ͷ500�ף���β300�����
        a2=300/TSlength;
    elseif dis>500 && dis<1000
        a1=1;  %����ͷ1����������β0.75���������
        a2=0.75;
    else
        a1=2;  %����ͷ2����������β1.5���������
        a2=1.5;
    end
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

