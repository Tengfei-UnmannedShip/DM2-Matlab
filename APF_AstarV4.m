%% ���ղ��ԣ����鲽��1������2��
% ���Ե�1������׼WP
% ���Ե�2�������Ƶ�����SP��WP��·����
% Ҫ��1.������ÿһ����·�ߣ�
%      2.·��������Ȼ
%      3.����Ч������㾫�ȵĵ��ԣ�

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
    %     Data=AStar2(ship(i).APF,start_row,start_col,end_row,end_col,speed,OSLength,Course,dire,0);
    background=map;
    
    start_point.row=start_row;
    start_point.col=start_col;
    
    destination.row=end_row;
    destination.col=end_col;
    Movelength=round((speed*1852/60)/100);
    ShipLong=round(OSLength/100);
    SurroundPointsNum=dire; % direction������������n���A*
    RudderAngle=2*pi/SurroundPointsNum;
    Rmin=2*Movelength/3; %ת��뾶
    valueAPF=10;  %APF�Ƴ��ļ�ֵ����
    %     Rmin=0;
    
    %��ʼ����
    % line2. ��ʼ׼����
    %�������λ���ڵ�ͼ��Χ֮����ߴ���״̬����ȫ���㷨��������ʾ�ް�ȫ·����
    %���������ʼ�ڵ������(���ꡢ���򡢳ͷ�ֵ���ƶ�����G����Ŀ����Ԥ�ƴ���H���ܴ���F����һ���ƶ�����r�����ڵ㡢�ӽڵ��)
    %�����ýڵ�ŵ�open����,��ʼ��close�б�Ϊ�գ�
    if (0<start_point.col<length(background(1,:))&&0<start_point.row<length(background(:,1)))
        start_point.G=0; %�ƶ����� G
        start_point.H=sqrt((destination.col-start_point.col)^2+(destination.row-start_point.row)^2);  %��Ŀ����Ԥ�ƴ���H
        start_point.F=start_point.G+start_point.H; %�ܴ���F
        start_point.R= Movelength; %��һ���ƶ�����r
        %����û�ж�������ע���޸�
        start_point.Dir=Course*pi/180;  %��ʼ������
        
        Open(1)=start_point; %��ʼ������
        Open(1).father=nan; %���ڵ�
        Close(1)=Open(1); %�����ýڵ�ŵ�open����,��ʼ��close�б�Ϊ�գ�
    end
    % ��ʼ����
    while  ~isempty(Open)  %line3.While: open �б�Ϊ��
        for ii=2:length(Open)  %line4.Ѱ��open�б���Fֵ��С�Ľڵ㣬��ΪFMin��
            if Open(ii).F < Open(1).F
                OpenTemp=Open(ii);
                Open(ii)=Open(1);
                Open(1)=OpenTemp;
            end
        end
        Close=[Close;Open(1)]; %line5-1.��FMin����close�б�,����FMin����SetClose(end),ͬʱ��open�б���ɾ���õ㣻
        Open(1)=[]; %line5-2.��FMin����close�б�ͬʱ��open�б���ɾ���õ㣻
        Surround=[];
        if Node_opti==1 %���ѡ��ڵ��Ż�optimization
            % �㷨4���ڽ��ڵ���ѡ
            %���룺A*�㷨�е�close�б�
            %������Ż����close�б�
            %%%��������%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
        %�䲽������%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %line7. ���� FMin �ڵ�ĳͷ�ֵ��С������һ��Ӧ���ƶ��Ĳ��� ShipSpeed��
        ShipSpeed=Movelength * (1-map(Close(end).row,Close(end).col));
        if ShipSpeed<1
            ShipSpeed=1;
        end
        %          ShipSpeed=Movelength;
        %line8.���㴬���ƶ�һ���ľ������movecost��Ӧ��չ������ڵ���Num��
        movecost=10; %���Ϊ���ٵ�A*������movecost�������ı�
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        for ii=1:SurroundPointsNum  %line9. For ���ɵ�ÿһ������ڵ�Surround(i)��
            Surround(ii).row=floor(Close(end).row-ShipSpeed*sin((ii-1)*RudderAngle));
            Surround(ii).col=floor(Close(end).col+ShipSpeed*cos((ii-1)*RudderAngle));
            Surround(ii).R= ShipSpeed;
            Surround(ii).Dir = ShipDirection(Close(end).row,Close(end).col,Surround(ii).row,Surround(ii).col);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%%���ټ������ڵ������
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
            % line10.If(Sourround(i)��Ŀ������Χ֮�ڣ���Sourround(i)�������ϰ����Sourround(i)����close�б���,�Ҵ�FMin�ƶ���Sourround(i)�����д�����ȫ,���˶����̲��ܴ����˶���������)
            if (0>=Surround(ii).col||Surround(ii).col>=length(background(1,:))||0>=Surround(ii).row||Surround(ii).row>=length(background(:,1))...
                    || background(Surround(ii).row,Surround(ii).col)==1 ||alreadyexist(Surround(ii),Close)==1 ...
                    ||ObstacleInMove(background,Close(end).row,Close(end).col,Surround(ii).row,Surround(ii).col,ShipLong/2)==0 ...
                    ||ObstacleInDomain(background,Surround(ii).row,Surround(ii).col,ShipLong/2)==0)...
                    ||PointsCanReach(ShipSpeed,Rmin,Surround(ii).Dir,Close(end).Dir)==0
            else
                %line11. ����Sourround(i)��G��H��Fֵ,����FMinΪSourround(i)�ĸ��ڵ㣻
                Surround(ii).H=sqrt((destination.col-Surround(ii).col)^2+(destination.row-Surround(ii).row)^2);
                Surround(ii).G=Close(end).G+movecost+valueAPF*movecost*map(Surround(ii).row,Surround(ii).col);%movecost���ڵ����Ƴ�����ֵ
                Surround(ii).F=Surround(ii).G+Surround(ii).H;
                Surround(ii).father=Close(end); %����FMinΪSourround(i)�ĸ��ڵ㣻
                
                if alreadyexist(Surround(ii),Open)==0 %line12. If(Sourround(i)�������겻ͬ��open�б������������)
                    Open=[Open;Surround(ii)]; %line13. ��Sourround(i)����open�б�
                else %line14
                    % line15.�Ƚ�Sourround(i)��open�б��о�����ͬ����ڵ��Gֵ�����ý�С�ߵĸ��ڵ�ΪFMin��
                    for kk=1:length(Open)
                        %                         if abs(Surround(ii).row - SetOpen(kk).row)<=1/4*ShipLong && abs(Surround(ii).col-SetOpen(kk).col)<=1/4*ShipLong
                        if (Surround(ii).row == Open(kk).row && Surround(ii).col==Open(kk).col)
                            rember=kk;                       %�ҵ�Sourround(i)��open�б��о�����ͬ����Ľڵ�
                        end
                    end
                    if Surround(ii).G < Open(rember).G     %�Ƚ�Gֵ
                        Open(rember).father=Close(end); %���ý�С�ߵĸ��ڵ�ΪFMin��
                    end
                end %line16.
            end     %line17.
        end         %line18.
        if Close(end).H < ShipSpeed %line19. ���FMin��Ŀ���ľ���С���ƶ��������㷨������
            break;
        end
    end
    destination.father=Close(end);
    destination.Dir=ShipDirection(Close(end).row,Close(end).col,end_row,end_col);
    
    %ԭ���������Ƕ�׵ķ�ʽ�������Ҫ�Ѿ��ߵ����ݣ�ÿһ����λ�á�����ȡ����
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
    ship(i).posData=Data(:,1:2);
    ship(i).courseData=Data(:,3);
    
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


