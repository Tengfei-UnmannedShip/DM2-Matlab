function CPA = computeCPA0(OS,TS,time)
%% ������CPA���㵫�Ƕ���һ������ʱ�䣬����Jinfen���㷨�м���ÿһ�׶ε�CPA
%��������Ŀ�괬���ٶ���(�ٶ�ֵ+���н�)�ļ�������ʽת��Ϊ(Vx,Vy)��ֱ��������ʽ
v_own=OS.speed;
course_own=OS.Course;
pos_own=OS.pos;
v_target=TS.speed;
course_target=TS.Course;
pos_target=TS.pos;

V_x1 = v_own*sind(course_own);%WTF:sind���ԽǶ�Ϊ�Ա�����sinֵ��sin���Ի���Ϊ��λ�ģ�deg2rad���Ƕ�ת��Ϊ����
V_y1 = v_own*cosd(course_own);

V_x2 = v_target*sind(course_target);
V_y2 = v_target*cosd(course_target);
%WTF:����������ٶ�
V_x = V_x1-V_x2;
V_y = V_y1-V_y2;  %WTF:��������ʾ��Ŀ�괬Ϊ���������������ٶ�

pos = pos_target-pos_own;%WTF:������λ���������ɱ���ָ��Ŀ�괬,��ϸ�Ľ��ͼ������־

%WTF:��������̾�����ϸ�Ľ��ͼ������־
p_x = [V_y*(V_y*pos(1)-V_x*pos(2))/(V_x^2+V_y^2) -V_x*(V_y*pos(1)-V_x*pos(2))/(V_x^2+V_y^2)];

d = norm(p_x-pos,2);
%% WTF:λ���ж��㷨
if V_x*pos(1)+V_y*pos(2)<=0 %˵��������Զ��
%WTF:����pos=pos_target-pos_own,����ٶ�����v=(V_x,V_y),��˻�pos?v=V_x*pos(1)+V_y*pos(2)������н�(0,pi),
%WTF:���<=0,���������λ��������Ŀ�괬����ٶ������нǴ���90�ȣ���������Զ�룬��������Զ��ʱ����С����DCPA��Ϊ��ǰʱ�̵�ֵ
   t = 0;
else
    t = d/sqrt(V_x^2+V_y^2);
end
if t>time
    t=time;
end
%% WTF:�������
pos1=[pos_own(1)+v_own*sind(course_own)*t, pos_own(2)+v_own*cosd(course_own)*t];
pos2=[pos_target(1)+v_target*sind(course_target)*t, pos_target(2)+v_target*cosd(course_target)*t];

dist=norm(pos1-pos2,2);%WTF:norm�������ķ�������(x1^2+x2^2+x3^2)^(1/2)�����˴��������ķ���
CPA = [pos1,pos2,dist,t]; %�������:��������㴦OS����pos1��TS����pos2��DCPA��TCPA
end