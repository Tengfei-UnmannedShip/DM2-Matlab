% A*��APF��ϵĵ����棬APFʹ��������ship domainģ��

clear all;
clc;
tic;%tic1
t1=clock;
%% �������ã�������ʼ��
%�����������ã������޸�,�ֱ��ǣ�
% 1.compliance:�������������;2.inferLabbel:�Ƿ��Ʋ�
shipLabel=[
    0 0
    1 0
    1 0
    1 0];
% shipLabel=zeros(4,2);%����Ƿ����м佻��
%1~2λ��(�м��λ�ã�������ʼλ��)��3����(��)��4��ʼ����deg������Ϊ0����5��������ʱ����6��ⷶΧ��range��nm��
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
%�����������
ShipNum=4;
%ÿһ�Ҵ����Ļ��Χ
% map=[x1,x2,y1,y2],��ʾ4���߽磬��λ����
MapSize=[10,10];
GoalRange=MapSize-[1.5,1.5];
APFmapSize = [MapSize(1),MapSize(1),MapSize(2),MapSize(2)];
tMax=2500;
t1=0;
tt=2000;
for j=1:1:ShipNum
    
    ship(j).speeds = boat(j,3)/3600;%WTF��������ת��Ϊ�����
    ship(j).speed = boat(j,3);%WTF��ԭ���٣�����ship domain
    ship(j).ratio=1;             %���ٶȸı�ʱ�ı����
    ship(j).initialCourse = boat(j,4);
    ship(j).compliance= shipLabel(j,1); %compliance�ǶԱ�������ķ����ԣ�0��ֱ��ǰ�������ã�1�����أ�2�������أ�������
    ship(j).decisioncycle=boat(j,5);
    ship(j).range=boat(j,6)*1852;  %detect range
    ship(j).no=j;  %receive information from other ships
    ship(j).inferLabbel=shipLabel(j,2); %�����Ƿ��Ʋ��־��1Ϊ�Ʋ⣬0���Ʋ�
    ship(j).data=[];%WTF��ship.data��Ϊ�գ����ڴ��ÿһ������ʷ����
    ship(j).compliance_data=[];
    ship(j).decision_lable=0;
    ship(j).infer=[];
    ship(j).OSdecision=0;
    ship(j).Vratio=1;
    ship(j).courseAlter = 0; %��ʼ״̬�ĺ���Ǹñ���Ϊ0
    ship(j).Course = ship(j).initialCourse+ship(j).courseAlter;
    ship(j).Start_pos=[boat(j,1)-ship(j).speeds*sind(ship(j).initialCourse)*0.5*tMax, boat(j,2)-ship(j).speeds*cosd(ship(j).initialCourse)*0.5*tMax];
    ship(j).pos=[ship(j).Start_pos(1)+ship(j).speeds*sind(ship(j).Course)*t1, ship(j).Start_pos(1)+ship(j).speeds*cosd(ship(j).Course)*t1];
    ship(j).DCPA_Record = [];
    ship(j).TCPA_Record = [];
    ship(j).length=boatsize(j,1);
    ship(j).width=boatsize(j,2);
    ship(j).goalPiont0= Goal_point(ship(j).Start_pos(1),ship(j).Start_pos(2),ship(j).Course,GoalRange);
end

t=1;
%��������
%CAL�����У�ÿһ��Ϊ������ÿһ��ΪĿ�괬��
%CAL(j,k)��Ϊ����j��Ŀ�괬k�ı�����ͼ��0Ϊ�Ӵ�ͷ�ߣ�1Ϊ�Ӵ�β��2Ϊ�Լ����Լ�
%����1��ȫ������
CAL=[2 0 0 1
    1 2 0 1
    1 1 2 0
    0 0 1 2];

%% ÿ��ʱ�̵ľ���
%���Ƶ�ǰʱ�̵�APF��ͼ�������Ҵ���һ��ȫ�������֮��ÿ��ʹ�ö����ӣ�������������8�Ҵ���
for i=1:1:ShipNum
    
    APF(i).map= DrawAPF(ship(i),ship(i),APFmapSize,0);
    
end

for i=1:1:ShipNum
    %��ʼÿ�Ҵ��ľ���
    
    %% ���㱾����·�����Ŀ���
    %�ҵ���ǰʱ�̱�������ÿ��Ŀ�괬�ľ���·���㣨��ͷ�Ĵ�β�ģ�
    kk=1;
    WP_Pos=ship(i).pos(end,:);
    for ts=1:1:ShipNum
        Dis_temp0=ship(i).pos(end,:)-ship(ts).pos(end,:);
        ship(i).dis(t,ts)=norm(Dis_temp0);
        if ts~=i
            TS(kk)= ship(ts);
            kk=kk+1;
            WP_Pos=[WP_Pos;ship(i).pos(end,:)];
        end
    end
    ship(i).TSWayPoint0 = WayPoint0(ship(i),TS,1000);
    %����CALɸѡ·����
    kk=1;
    for ts=1:1:ShipNum
        if ts~=i
            if CAL(i,kk)==0
                ship(i).TSWayPoint(kk,:)=ship(i).TSWayPoint0(kk,1:2);
            elseif  CAL(i,kk)==1
                ship(i).TSWayPoint(kk,:)=ship(i).TSWayPoint0(kk,3:4);
            end
            kk=kk+1;
        end
    end
    %�������������ı������ۺ�·����
    ship(i).OSWayPoint=WP_allShips(WP_Pos,ship(i).TSWayPoint);
    
    %% �滮������·��
    
    %��㵽waypoint
    
    
    
    
    %waypoint���յ�
    
    
    
    
    %���ݴ洢
    
    
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp(['����������ʱ�䣺',num2str(etime(clock,t1))]);
