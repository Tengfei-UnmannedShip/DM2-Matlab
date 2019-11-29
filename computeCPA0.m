function CPA = computeCPA0(OS,TS,time)
%% 基本的CPA计算但是多了一个航向时间，用于Jinfen的算法中计算每一阶段的CPA
%将本船和目标船的速度由(速度值+航行角)的极坐标形式转化为(Vx,Vy)的直角坐标形式
CPA=[];
for i=1:1:length(TS)
    if length(OS.pos)>1
        OS.pos=OS.pos(end,:);
    end
    if length(OS.Course)>1
        OS.Course=OS.Course(end,:);
    end
    if length(TS(i).pos)>1
        TS(i).pos=TS(i).pos(end,:);
    end
    if length(TS(i).Course)>1
        TS(i).Course=TS(i).Course(end,:);
    end

v_own=OS.speed;
course_own=OS.Course;
pos_own=OS.pos;
v_target=TS(i).speed;
course_target=TS(i).Course;
pos_target=TS(i).pos;

V_x1 = v_own*sind(course_own);%WTF:sind是以角度为自变量的sin值，sin是以弧度为单位的，deg2rad将角度转换为弧度
V_y1 = v_own*cosd(course_own);

V_x2 = v_target*sind(course_target);
V_y2 = v_target*cosd(course_target);
%WTF:两船的相对速度
V_x = V_x1-V_x2;
V_y = V_y1-V_y2;  %WTF:用向量表示以目标船为参照物，本船的相对速度

pos = pos_target-pos_own;%WTF:两船的位置向量，由本船指向目标船,详细的解释见编程日志

%WTF:两船的最短距离详细的解释见编程日志
p_x = [V_y*(V_y*pos(1)-V_x*pos(2))/(V_x^2+V_y^2) -V_x*(V_y*pos(1)-V_x*pos(2))/(V_x^2+V_y^2)];

d = norm(p_x-pos,2);
%% WTF:位置判断算法
if V_x*pos(1)+V_y*pos(2)<=0 %说明两船逐渐远离
%WTF:向量pos=pos_target-pos_own,相对速度向量v=(V_x,V_y),点乘积pos?v=V_x*pos(1)+V_y*pos(2)，点积夹角(0,pi),
%WTF:点积<=0,则两船相对位置向量与目标船相对速度向量夹角大于90度，两船正在远离，两船正在远离时，最小距离DCPA即为当前时刻的值
   t = 0;
else
    t = d/sqrt(V_x^2+V_y^2);
end
if t>time
    t=time;
end
%% WTF:距离计算
pos1=[pos_own(1)+v_own*sind(course_own)*t, pos_own(2)+v_own*cosd(course_own)*t];
pos2=[pos_target(1)+v_target*sind(course_target)*t, pos_target(2)+v_target*cosd(course_target)*t];

dist=norm(pos1-pos2,2);%WTF:norm求向量的范数，即(x1^2+x2^2+x3^2)^(1/2)，即此处的求距离的方程
CPA = [CPA;pos1,pos2,dist,t]; %函数输出:最近会遇点处OS坐标pos1，TS坐标pos2，DCPA和TCPA
end
end