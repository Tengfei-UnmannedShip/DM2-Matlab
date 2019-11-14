%% A*��APF��ϵĵڶ��棬A*�м������м�㣬APF��Ϊһ������
% ԭ������Ѧ˫��
% ��һ��������⣺0��û���м��??�м��Ӧ������APF��ͼ����֮��ģ����ԣ�APFӦ��Ҳ��һ������
% 1.��ͼ�Ƿ���??����ת��
% 2.��ɫΪ�ڰ�-�Ƿ��·��ȡ�����ŵ��ȸ���ͼ����ʾ���ο�classicAPF?new-main��ķ���

clear all;
clc;
tic;%tic1
t1=clock;

[X,Y]=meshgrid(-300:1:300,-300:1:300);
[m,n]=size(X);
%%==================================================
%����������������
% ����ģ�Ͳο��Ŷռ��㷽��
% ==================================================
Boat_Speed_Factor=1;        %�ٶȷ����Ƴ�˥�����ӣ�ȡֵԽ�����ٶȷ�����Ӱ��Խ��
BoatRiskFieldPeakValue=100;   %�������ֵ���Ը�����Ҫ��������
Boat_eta=1;  %�ı�Ӱ�췶Χ���������½��¶ȣ�Խ��ΧԽС
Boat_alfa=0.1; %�ı�Ӱ�췶Χ���������½��¶ȣ�Խ��ΧԽС
BoatCut=0;
RiskFieldValue=zeros(m,n);%wtf--Z=RiskFieldValue����ÿһ����Ƴ�ֵ���˴��Ƚ������Ϊһ����X��ͬ��С��0����

U_att=zeros(m,n);
APFValue=zeros(m,n);

Boat_Num=3;%��������
%�۲���λ��
Boat.State(1,:)=[   0   220   180     5  30  10];  %����״̬���꣨x,y,ang,v,l,w��angΪ�������� vΪ�ٶȴ�С,lΪ������wΪ����
Boat.State(2,:)=[-200    50   120     5  20   6];  %����״̬���꣨x,y,ang,v��angΪ��������
Boat.State(3,:)=[ 150    70  -120     8  10   3];
Boat.State(4,:)=[ 120  -150   -45    10  20   5]; %ship4�Ǳ���
%Boat.State(5,:)=[-70   -15    90    10  20   5];
goal=[-280  250];%��ʱ�����ڣ�120��-150���Ƕ�-45
k_near=10;%����Ŀ��㸽��������Ҫ������ϵ��
k_far=10; %���������ż�֮�������㶨ʱ������ϵ����һ����k_near��ͬ���������ż��������������ͻ��
Po_att=100;%�����ż��뾶
k=0.3;%����ϵ��
d_goal=sqrt((X-goal(1,1)).^2+(Y-goal(1,2)).^2);
% U_att=0.5*k_near*d_obs_APF.^2;%���������ż���

% U_att=0.5*k_near*d_goal.^2.*(d_goal<=Po_att)+k_far*Po_att*d_goal.*(d_goal>Po_att);%�������ż���
% U_att=d_goal.*k ;  %����Ϊ�����ģ��������Ǿ����һ�κ���

for i=1:1:Boat_Num
    
    %���������ʼ��
    Boat_x(i)=Boat.State(i,1);                  %��i����x����
    Boat_y(i)=Boat.State(i,2);                  %��i����y����
    Boat_theta(i)=-Boat.State(i,3)/180*pi;       %��i��������Ƕ�
    Boat_Speed(i)=Boat.State(i,4);              %��i�����ٶȴ�С
    Boat_length(i)=Boat.State(i,5);             %��i������
    Boat_width(i)=Boat.State(i,6);              %��i������
    
    %�Ѽ����Ƴ������꣨X,Y���任����������ϵ�µ㣨BoatX{i},BoatY{i}��
    BoatX{i} = (X-Boat_x(i))*cos(Boat_theta(i))+(Y-Boat_y(i))*sin(Boat_theta(i));
    BoatY{i} = (Y-Boat_y(i))*cos(Boat_theta(i))-(X-Boat_x(i))*sin(Boat_theta(i));
    
    BoatSpeedFactor{i}=Boat_Speed_Factor*Boat_Speed(i);
    
    %����ռ��е㵽��i�������ؾ���Dis{i}
    Dis{i}=zeros(m,n);
    Dis{i}=sqrt(BoatX{i}.^2+BoatY{i}.^2).*(BoatY{i}<=0)+sqrt((BoatY{i}/(Boat_Speed(i)+1)).^2+BoatX{i}.^2).*(BoatY{i}>0);
    
    %�����i�������ճ�
    BoatRiskField{i}=BoatRiskFieldPeakValue.*(exp(-Boat_eta*Boat_alfa*Dis{i})./(Boat_alfa*Dis{i}));
    
    if  BoatRiskField{i}>BoatRiskFieldPeakValue
        BoatRiskField{i}=BoatRiskFieldPeakValue;
    end
    BoatRiskField{i}(BoatRiskField{i}>BoatRiskFieldPeakValue)=BoatRiskFieldPeakValue;
    
    if BoatCut==1
        BoatRiskField{i}=BoatRiskField{i}.*(BoatX{i}>=0)+0.*(BoatX{i}<0);
    end
    
    %����ÿ�����ڲ�ͬ���µĳ�֮���ݲ��ü򵥼Ӻ�
    RiskFieldValue=RiskFieldValue+BoatRiskField{i};
end
% APFValue=max(U_att,RiskFieldValue);
APFValue=RiskFieldValue;
newfield=RiskFieldValue/BoatRiskFieldPeakValue;
figure;
mesh(X,Y,APFValue);
hold on;
plot(goal(1,1),goal(1,2),'ro','MarkerFaceColor','r');
%     hold on;
%     ship_icon(Boat.State(4,1),Boat.State(4,2),Boat.State(4,5), Boat.State(4,6), Boat.State(4,3),0 );
axis equal;
axis off;
%     surf(X,Y,APFValue);
figure
contourf(X,Y,APFValue,'LevelStep',30);  %�������ɫ�ĵȸ���ͼ
hold on;
plot(goal(1,1),goal(1,2),'ro','MarkerFaceColor','r');
hold on;
ship_icon(Boat.State(4,1),Boat.State(4,2),Boat.State(4,5), Boat.State(4,6), Boat.State(4,3),0 );

%     plot(goal(1,1),goal(1,2),'rx');
axis equal;
axis off;

% newfield1=zeros(size(newfield));
%% A*�㷨
% [SetClose,SetOpen]=CircleAStar(newfield,300,500,500,100);%ֱ�ӵ��ú�����% ԭ������Ѧ˫��

% function [SetClose,SetOpen]=CircleAStar(map,start_row,start_col,end_row,end_col)
% �㷨5:����A*�㷨
%����: �ͷ�ͼ��(����)map,���ͼ������(start_row,start_col),Ŀ���ͼ������(destination_row, destination_col),��������ShipLong,���ذ��Rmin
%���: �������ĵ㼯open�б���ѡΪ����·���ڵ�ĵ㼯close�б�
% line1. ���ó�ʼ��������
background=newfield;
map=newfield;
MapSize=[300,300];
% start_row=300; 
% start_col=500; 
start_row=round(Boat.State(4,1)+MapSize(1)); 
start_col=round(Boat.State(4,2)+MapSize(2)); 
start_point.row=start_row;
start_point.col=start_col;
GoalRange=MapSize-[20,20];
%     ship(j).goalPiont= Goal_point(ship(j).pos(1),ship(j).pos(2),ship(j).Course,GoalRange);
goalPiont=Goal_point1(Boat.State(4,1),Boat.State(4,2),Boat.State(4,3),GoalRange);
% end_row=500; 
% end_col=100; 
end_row=round(goalPiont(1)+MapSize(1)); 
end_col=round(goalPiont(2)+MapSize(2)); 
destination.row=end_row;
destination.col=end_col;
ShipLong=4;
Movelength=20;  %����
SurroundPointsNum=20; %������������n���A*
RudderAngle=2*pi/SurroundPointsNum;
Rmin=2*Movelength/3; %ת��뾶
valueAPF=2;  %APF�Ƴ��ļ�ֵ����
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
    start_point.Dir=pi/2;  %��ʼ������
    SetOpen(1)=start_point; %��ʼ������
    SetOpen(1).father=nan; %���ڵ�
    SetClose(1)=SetOpen(1); %�����ýڵ�ŵ�open����,��ʼ��close�б�Ϊ�գ�
end
% ��ʼ����
while  ~isempty(SetOpen)  %line3.While: open �б�Ϊ��
    for ii=2:length(SetOpen)  %line4.Ѱ��open�б���Fֵ��С�Ľڵ㣬��ΪFMin��
        if SetOpen(ii).F < SetOpen(1).F
            a=SetOpen(ii);
            SetOpen(ii)=SetOpen(1);
            SetOpen(1)=a;
        end
    end
    SetClose=[SetClose;SetOpen(1)]; %line5-1.��FMin����close�б�,����FMin����SetClose(end),ͬʱ��open�б���ɾ���õ㣻
    SetOpen(1)=[]; %line5-2.��FMin����close�б�ͬʱ��open�б���ɾ���õ㣻
    Surround=[];
    %         %% �㷨4���ڽ��ڵ���ѡ
    %         %���룺A*�㷨�е�close�б�
    %         %������Ż����close�б�
    %         %%%��������%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %         L_Close=length(SetClose);
    %         ComPoint=[];
    %         if L_Close>2
    %             ComPoint=SetClose(end).father;
    %             while ~(ComPoint.row==start_row && ComPoint.col==start_col)
    %                 if ((SetClose(end).row-ComPoint.row)^2+(SetClose(end).col-ComPoint.col)^2)<(ComPoint.R)^2
    %                     SetClose(end).father=ComPoint;
    %                     SetClose(end).G=ComPoint.G+movecost+movecost*map(ComPoint.row,ComPoint.col);
    %                 end
    %                 ComPoint=ComPoint.father;
    %             end
    %         end
    %         SetClose(end).father=ComPoint;
    
    %�䲽������%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %line7. ���� FMin �ڵ�ĳͷ�ֵ��С������һ��Ӧ���ƶ��Ĳ��� ShipSpeed��
    ShipSpeed=Movelength * (1-map(SetClose(end).row,SetClose(end).col));
    if ShipSpeed<1
        ShipSpeed=1;
    end
    %          ShipSpeed=Movelength;
    %line8.���㴬���ƶ�һ���ľ������movecost��Ӧ��չ������ڵ���Num��
    movecost=10; %���Ϊ���ٵ�A*������movecost�������ı�
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for ii=1:SurroundPointsNum  %line9. For ���ɵ�ÿһ������ڵ�Surround(i)��
        Surround(ii).row=floor(SetClose(end).row-ShipSpeed*sin((ii-1)*RudderAngle));
        Surround(ii).col=floor(SetClose(end).col+ShipSpeed*cos((ii-1)*RudderAngle));
        Surround(ii).R= ShipSpeed;
        Surround(ii).Dir = ShipDirection(SetClose(end).row,SetClose(end).col,Surround(ii).row,Surround(ii).col);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%���ټ������ڵ������
        if ~isempty( SetOpen)
            openitear=1;
            mindis = 1000;
            while (openitear<length(SetOpen))
                dis=sqrt((Surround(ii).row -SetOpen(openitear).row)^2+(Surround(ii).col-SetOpen(openitear).col)^2);
                if(dis<mindis)
                    mindis=dis;
                    replace=openitear;
                end
                openitear=openitear+1;
            end
            if (mindis<Movelength/4 && ObstacleInMove(background,Surround(ii).row,Surround(ii).col,SetOpen(replace).row,SetOpen(replace).col,ShipLong/2)==1)
                %                         if (mindis<6)
                Surround(ii).row=SetOpen(replace).row;
                Surround(ii).col=SetOpen(replace).col;
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % line10.If(Sourround(i)��Ŀ������Χ֮�ڣ���Sourround(i)�������ϰ����Sourround(i)����close�б���,�Ҵ�FMin�ƶ���Sourround(i)�����д�����ȫ,���˶����̲��ܴ����˶���������)
        if (0>=Surround(ii).col||Surround(ii).col>=length(background(1,:))||0>=Surround(ii).row||Surround(ii).row>=length(background(:,1))...
                || background(Surround(ii).row,Surround(ii).col)==1 ||alreadyexist(Surround(ii),SetClose)==1 ...
                ||ObstacleInMove(background,SetClose(end).row,SetClose(end).col,Surround(ii).row,Surround(ii).col,ShipLong/2)==0 ...
                ||ObstacleInDomain(background,Surround(ii).row,Surround(ii).col,ShipLong/2)==0)...
                ||PointsCanReach(ShipSpeed,Rmin,Surround(ii).Dir,SetClose(end).Dir)==0
        else
            %line11. ����Sourround(i)��G��H��Fֵ,����FMinΪSourround(i)�ĸ��ڵ㣻
            Surround(ii).H=sqrt((destination.col-Surround(ii).col)^2+(destination.row-Surround(ii).row)^2);
            Surround(ii).G=SetClose(end).G+movecost+valueAPF*movecost*map(Surround(ii).row,Surround(ii).col);%movecost���ڵ����Ƴ�����ֵ
            Surround(ii).F=Surround(ii).G+Surround(ii).H;
            Surround(ii).father=SetClose(end); %����FMinΪSourround(i)�ĸ��ڵ㣻
            
            if alreadyexist(Surround(ii),SetOpen)==0 %line12. If(Sourround(i)�������겻ͬ��open�б������������)
                SetOpen=[SetOpen;Surround(ii)]; %line13. ��Sourround(i)����open�б�
            else %line14
                % line15.�Ƚ�Sourround(i)��open�б��о�����ͬ����ڵ��Gֵ�����ý�С�ߵĸ��ڵ�ΪFMin��
                for kk=1:length(SetOpen)
                    %                         if abs(Surround(ii).row - SetOpen(kk).row)<=1/4*ShipLong && abs(Surround(ii).col-SetOpen(kk).col)<=1/4*ShipLong
                    if (Surround(ii).row == SetOpen(kk).row && Surround(ii).col==SetOpen(kk).col)
                        rember=kk;                       %�ҵ�Sourround(i)��open�б��о�����ͬ����Ľڵ�
                    end
                end
                if Surround(ii).G < SetOpen(rember).G     %�Ƚ�Gֵ
                    SetOpen(rember).father=SetClose(end); %���ý�С�ߵĸ��ڵ�ΪFMin��
                end
            end %line16.
        end     %line17.
    end         %line18.
    if SetClose(end).H < ShipSpeed %line19. ���FMin��Ŀ���ľ���С���ƶ��������㷨������
        break;
    end
end
destination.father=SetClose(end);
destination.Dir=ShipDirection(SetClose(end).row,SetClose(end).col,end_row,end_col);
%����·��
figure
%     background=K;
imshow(background);
rectangle('position',[1 1 size(background)-1],'edgecolor','k')%����ͼƬ�߿��С����ɫ
t=1;
M(t)=getframe;
t=t+1;

background2(:,:,1)=background;background2(:,:,2)=background;background2(:,:,3)=background;
CurrentPoint=destination;

while ~(CurrentPoint.row==start_point.row && CurrentPoint.col==start_point.col)
    position=[CurrentPoint.row CurrentPoint.col];
    %         if (CurrentPoint.col>=CurrentPoint.father.col)
    %             ShipDirection=atan((CurrentPoint.row-CurrentPoint.father.row)/(CurrentPoint.col-CurrentPoint.father.col));
    %         else
    %             ShipDirection=pi+atan((CurrentPoint.row-CurrentPoint.father.row)/(CurrentPoint.col-CurrentPoint.father.col));
    %         end
    %         plotShip(position,ShipDirection,ShipLong/2);
    plotShip(position,CurrentPoint.Dir,ShipLong/2);
    CurrentPoint=CurrentPoint.father;
    M(t)=getframe;t=t+1;
end
line([start_point.col-3;start_point.col+3;start_point.col+3;start_point.col-3;start_point.col-3],[start_point.row-3;start_point.row-3;start_point.row+3;start_point.row+3;start_point.row-3],'color','g','LineWidth',5);
line([destination.col-3;destination.col+3;destination.col+3;destination.col-3;destination.col-3],[destination.row-3;destination.row-3;destination.row+3;destination.row+3;destination.row-3],'color','b','LineWidth',5);
disp(['����������ʱ�䣺',num2str(etime(clock,t1))]);


