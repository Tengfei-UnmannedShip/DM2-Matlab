%% ���ղ��ԣ����鲽��3��
% �����3�������Ե�����SP��WP��GP

clear all;
clc;
tic;%tic1
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
    ship(j).speed1 = boat(j,3)*1852/3600;%WTF��ԭ���٣�����ship domain
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
    ship(j).pos=ship(j).Start_pos*1852;
    ship(j).DCPA_Record = [];
    ship(j).TCPA_Record = [];
    ship(j).length=boatsize(j,1);
    ship(j).width=boatsize(j,2);
    ship(j).goalPiont0= Goal_point(ship(j).Start_pos(1),ship(j).Start_pos(2),ship(j).Course,GoalRange);
    ship(j).posData=[];
    ship(j).courseData=[];
    
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
Re=100; %դ�񻯵ķֱ���
%% ÿ��ʱ�̵ľ���
%���Ƶ�ǰʱ�̵�APF��ͼ�������Ҵ���һ��ȫ�������֮��ÿ��ʹ�ö����ӣ�������������8�Ҵ���
for i=1:1:ShipNum
    
    APF(i).map= DrawAPF(ship(i),ship(i),APFmapSize,Re,0);
    
end
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% APFvalue=APF(i).map;
% for i=2:1:length(APF)
%     APFvalue=APFvalue+APF(i).map;
% end
%
% x1=-APFmapSize(1)*1852;
% x2=APFmapSize(1)*1852;
% y1=-APFmapSize(2)*1852;
% y2=APFmapSize(2)*1852;
%
% [APF_X,APF_Y]=meshgrid(x1:100:x2,y1:100:y2);
%
% figure
% mesh(APF_X,APF_Y,APFvalue);
% axis equal;
% axis off;
%
% figure
% contourf(APF_X,APF_Y,APFvalue,'LevelStep',30);  %�������ɫ�ĵȸ���ͼ
% axis equal;
% axis off;
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i=1:1:1
    %��ʼÿ�Ҵ��ľ���
    %% ���㱾����·�����Ŀ���
    %�ҵ���ǰʱ�̱�������ÿ��Ŀ�괬�ľ���·���㣨��ͷ�Ĵ�β�ģ�
    kk=1;
    WP_Pos=ship(i).pos(end,:); %WP_Pos��һ���Ǳ�����ǰλ��
    for ts=1:1:ShipNum
        Dis_temp0=ship(i).pos(end,:)-ship(ts).pos(end,:);
        ship(i).dis(t,ts)=norm(Dis_temp0);
        if ts~=i
            TS(kk).length = ship(ts).length;
            TS(kk).Course = ship(ts).Course(end,:);
            TS(kk).speed = ship(ts).speed(end,:);
            TS(kk).pos = ship(ts).pos(end,:);
            kk=kk+1;
            WP_Pos=[WP_Pos;ship(ts).pos(end,:)]; %WP_Pos֮��ÿ���ǰ�˳���Ŀ�괬��λ��
        end
    end
    ship(i).TSWayPoint0 = WayPoint0(ship(i),TS,1000);
    ship(i).APF=zeros(size(APF(ts).map));
    %����CALɸѡ·����
    kk=1;
    for ts=1:1:ShipNum
        if ts~=i
            ship(i).APF=ship(i).APF+APF(ts).map;
            if CAL(i,ts)==0
                ship(i).TSWayPoint(kk,:)=ship(i).TSWayPoint0(kk,1:2);
            elseif  CAL(i,ts)==1
                ship(i).TSWayPoint(kk,:)=ship(i).TSWayPoint0(kk,3:4);
            end
            kk=kk+1;
        end
    end
    %�������������ı������ۺ�·����
    ship(i).OSWayPoint=WP_allShips(WP_Pos,ship(i).TSWayPoint);
    
    %% ��㵽waypoint
    % function [SetClose,SetOpen]=CircleAStar(map,start_row,start_col,end_row,end_col)
    % �㷨5:����A*�㷨
    %����: �ͷ�ͼ��(����)map,���ͼ������(start_row,start_col),Ŀ���ͼ������(destination_row, destination_col),��������ShipLong,���ذ��Rmin
    %���: �������ĵ㼯open�б���ѡΪ����·���ڵ�ĵ㼯close�б�
    % line1. ���ó�ʼ��������
    
    background=ship(i).APF;
    map=ship(i).APF;
    
    start_row=round((ship(i).pos(1)+MapSize(1)*1852)/100);%APF��ͼ�ֱ�����100�ף���˳���100
    start_col=round((ship(i).pos(2)+MapSize(2)*1852)/100);
    
    end_row=round((ship(i).goalPiont0(1)+MapSize(1)*1852)/100);
    end_col=round((ship(i).goalPiont0(2)+MapSize(2)*1852)/100);
    OSLength=ship(i).length;
    speed=ship(i).speed;
    Course=ship(i).Course;
    dire=20; % direction������������n���A*
    Data=AStar2(ship(i).APF,start_row,start_col,end_row,end_col,speed,OSLength,Course,dire,0);

    ship(i).posData=Data(:,1:2);
    ship(i).courseData=Data(:,3);
    
    %% waypoint���յ�
    background=ship(i).APF;
    map=ship(i).APF;
    
    start_row=round((ship(i).OSWayPoint(1)+MapSize(1)*1852)/100);%APF��ͼ�ֱ�����100�ף���˳���100
    start_col=round((ship(i).OSWayPoint(2)+MapSize(2)*1852)/100);
    
    end_row=round((ship(i).OSWayPoint(1)+MapSize(1)*1852)/100);
    end_col=round((ship(i).OSWayPoint(2)+MapSize(2)*1852)/100);
    OSLength=ship(i).length;
    speed=ship(i).speed;
    Course=ship(i).Course;
    dire=20; % direction������������n���A*
    Data=AStar2(ship(i).APF,start_row,start_col,end_row,end_col,speed,OSLength,Course,dire,0);

    ship(i).posData=Data(:,1:2);
    ship(i).courseData=Data(:,3);    
    
    
    %���ݴ洢
    
    
end

toc
disp(['��������ʱ��: ',num2str(toc)]);

%��ÿ��λ��ռ�ĸ��ӻع鵽�㣬���м�����
for i=1:1:1
    for j=1:1:length(ship(i).posData)
        ship(i).pos(j+1,1)=(ship(i).posData(j,1)-0.5)*Re+(-MapSize(1)*1852);
        ship(i).pos(j+1,2)=(ship(i).posData(j,2)-0.5)*Re+(-MapSize(2)*1852);   
    end
end
%WTF:���������ĳ�ʼλ��
drawShip0(ship(1).pos(1,:),ship(1).Course(1),1,400);
drawShip0(ship(2).pos(1,:),ship(2).Course(1),2,400);
drawShip0(ship(3).pos(1,:),ship(3).Course(1),3,400);
drawShip0(ship(4).pos(1,:),ship(4).Course(1),4,400);

%WTF:���������ĺ���ͼ
plot(ship(1).pos(:,1),ship(1).pos(:,2),'r-');

%WTF:���������Ľ���λ��
drawShip0(ship(1).OSWayPoint(end,:),ship(1).courseData(end),1,400);



grid on;
xlabel('\it n miles', 'Fontname', 'Times New Roman');
ylabel('\it n miles', 'Fontname', 'Times New Roman');
% title(['t=',num2str(k),'s'], 'Fontname', 'Times New Roman');
box on;


