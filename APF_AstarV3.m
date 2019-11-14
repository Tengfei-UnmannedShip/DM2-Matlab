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
MapSize=[10,10];  %ÿһ�Ҵ����Ļ��Χ
t=2500;
t1=0;
tt=2000;
for j=1:1:ShipNum
    
    ship(j).speeds = boat(j,3)/3600;%WTF��������ת��Ϊ�����
    ship(j).speed0 = boat(j,3);%WTF��ԭ���٣�����ship domain
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
    ship(j).Start_pos=[boat(j,1)-ship(j).speeds*sind(ship(j).initialCourse)*0.5*t, boat(j,2)-ship(j).speeds*cosd(ship(j).initialCourse)*0.5*t];
    ship(j).pos1=[ship(j).Start_pos(1)+ship(j).speeds*sind(ship(j).Course)*t1, ship(j).Start_pos(1)+ship(j).speeds*cosd(ship(j).Course)*t1];
    ship(j).DCPA_Record = [];
    ship(j).TCPA_Record = [];
    ship(j).length=boatsize(j,1);
    ship(j).width=boatsize(j,2);
    GoalRange=MapSize-[1.5,1.5];
    ship(j).goalPiont0= Goal_point(ship(j).Start_pos(1),ship(j).Start_pos(2),ship(j).Course,GoalRange);
end

%% APF��ͼ����
[X,Y]=meshgrid(-MapSize(1):0.01:MapSize(1),-MapSize(2):0.01:MapSize(2));
[m,n]=size(X);
%%==================================================
%����������������
% ����ģ�Ͳο��Ŷռ��㷽��
% ==================================================

ShipDomainMap=zeros(m,n);
SCR=zeros(m,n);
Boat_Num=4;%��������

for i=1:1:Boat_Num
    
    %���������ʼ��
    Boat_x(i)=ship(i).Start_pos(1);                  %��i����x����
    Boat_y(i)=ship(i).Start_pos(2);                 %��i����y����
    Boat_theta0(i)=ship(i).Course;              %ת��ǰ��i��������Ƕ�
    Boat_theta(i)=-ship(i).Course/180*pi;       %��i��������Ƕ�
    Boat_Speed(i)=ship(i).speeds*ship(i).ratio;  %��i�����ٶȴ�С
    Boat_length(i)=ship(i).length;              %��i������
    Boat_width(i)=ship(i).width;                %��i������
    
    %�Ѽ����Ƴ������꣨X,Y���任����������ϵ�µ㣨BoatX{i},BoatY{i}��
    BoatX{i} = (X-Boat_x(i))*cos(Boat_theta(i))+(Y-Boat_y(i))*sin(Boat_theta(i));
    BoatY{i} = (Y-Boat_y(i))*cos(Boat_theta(i))-(X-Boat_x(i))*sin(Boat_theta(i));
    
    r0 = 0.5;
    L = Boat_length(i)/1852;
    k=2;
    Vos = ship(i).speed0;
    k_AD = 10^(0.3591*log10(Vos)+0.0952);
    k_DT = 10^(0.5441*log10(Vos)-0.0795);
    
    Rfore = (1+1.34*sqrt(k_AD^2+0.25*k_DT^2))*L;
    Raft = (1+0.67*sqrt(k_AD^2+0.25*k_DT^2))*L;
    Rstarb = (0.2+k_DT)*L;
    Rport = (0.2+0.75*k_DT)*L;
    R = [Rfore Raft Rstarb Rport];
    Sigma = R/((log(1/r0))^(1/k));
    a=Sigma(1);
    
    SCR= exp(-(2*BoatY{i}./((1+sign(BoatY{i}))*Sigma(1)+(1-sign(BoatY{i}))*Sigma(2))).^k-(2*BoatX{i}./((1+sign(BoatX{i}))*Sigma(3)+(1-sign(BoatX{i}))*Sigma(4))).^k);
    ShipDomainMap=ShipDomainMap+SCR;
end

figure
mesh(X,Y,ShipDomainMap);
% axis on;
axis equal;
xlabel('\it n miles', 'Fontname', 'Times New Roman');
ylabel('\it n miles', 'Fontname', 'Times New Roman');

figure
mesh(X,Y,ShipDomainMap);
hold on
surfc(X,Y,-5+0*ShipDomainMap,ShipDomainMap,'edgecolor','none','facecolor','interp') %���������ɫ�ĵȸ���ͼ
axis equal
xlabel('\it n miles', 'Fontname', 'Times New Roman');
ylabel('\it n miles', 'Fontname', 'Times New Roman');

figure
contourf(X,Y,ShipDomainMap,'LineStyle','none');  %�������ɫ�ĵȸ���ͼ
axis equal;
xlabel('\it n miles', 'Fontname', 'Times New Roman');
ylabel('\it n miles', 'Fontname', 'Times New Roman');

% figure
% contourf(X,Y,ShipDomainMap,'LineStyle','none');  %�������ɫ�ĵȸ���ͼ
% % for i=1:1:4
% %     hold on;
% %     plot(ship(1).goalPiont0(1),ship(1).goalPiont0(2),'ro','MarkerFaceColor','r');
% %     hold on;
% %     ship_icon(ship(1).Start_pos(1),ship(1).Start_pos(2),ship(1).length, ship(1).width, ship(1).Course,0 );
% %     plot(goal(1,1),goal(1,2),'rx');
% % end
% % axis on;
% axis equal;

% % newfield1=zeros(size(ShipDomainMap));
% %% A*�㷨
% % [SetClose,SetOpen]=CircleAStar(newfield,300,500,500,100);%ֱ�ӵ��ú�����% ԭ������Ѧ˫��
% 
% % function [SetClose,SetOpen]=CircleAStar(map,start_row,start_col,end_row,end_col)
% % �㷨5:����A*�㷨
% %����: �ͷ�ͼ��(����)map,���ͼ������(start_row,start_col),Ŀ���ͼ������(destination_row, destination_col),��������ShipLong,���ذ��Rmin
% %���: �������ĵ㼯open�б���ѡΪ����·���ڵ�ĵ㼯close�б�
% % line1. ���ó�ʼ��������
% background=ShipDomainMap;
% map=ShipDomainMap;
% start_row=round((ship(4).Start_pos(1)+MapSize(1))*100);
% start_col=round((ship(4).Start_pos(2)+MapSize(2))*100);
% start_point.row=start_row;
% start_point.col=start_col;
% end_row=round((ship(4).goalPiont0(1)+MapSize(1))*100);
% end_col=round((ship(4).goalPiont0(2)+MapSize(2))*100);
% 
% destination.row=end_row;
% destination.col=end_col;
% Movelength=round(ship(4).speed0/10);
% ShipLong=round(ship(4).length);
% SurroundPointsNum=20; %������������n���A*
% RudderAngle=2*pi/SurroundPointsNum;
% Rmin=2*Movelength/3; %ת��뾶
% valueAPF=10;  %APF�Ƴ��ļ�ֵ����
% %     Rmin=0;
% 
% %��ʼ����
% % line2. ��ʼ׼����
% %�������λ���ڵ�ͼ��Χ֮����ߴ���״̬����ȫ���㷨��������ʾ�ް�ȫ·����
% %���������ʼ�ڵ������(���ꡢ���򡢳ͷ�ֵ���ƶ�����G����Ŀ����Ԥ�ƴ���H���ܴ���F����һ���ƶ�����r�����ڵ㡢�ӽڵ��)
% %�����ýڵ�ŵ�open����,��ʼ��close�б�Ϊ�գ�
% if (0<start_point.col<length(background(1,:))&&0<start_point.row<length(background(:,1)))
%     start_point.G=0; %�ƶ����� G
%     start_point.H=sqrt((destination.col-start_point.col)^2+(destination.row-start_point.row)^2);  %��Ŀ����Ԥ�ƴ���H
%     start_point.F=start_point.G+start_point.H; %�ܴ���F
%     start_point.R= Movelength; %��һ���ƶ�����r
%     start_point.Dir=pi/2;  %��ʼ������
%     SetOpen(1)=start_point; %��ʼ������
%     SetOpen(1).father=nan; %���ڵ�
%     SetClose(1)=SetOpen(1); %�����ýڵ�ŵ�open����,��ʼ��close�б�Ϊ�գ�
% end
% % ��ʼ����
% while  ~isempty(SetOpen)  %line3.While: open �б�Ϊ��
%     for ii=2:length(SetOpen)  %line4.Ѱ��open�б���Fֵ��С�Ľڵ㣬��ΪFMin��
%         if SetOpen(ii).F < SetOpen(1).F
%             a=SetOpen(ii);
%             SetOpen(ii)=SetOpen(1);
%             SetOpen(1)=a;
%         end
%     end
%     SetClose=[SetClose;SetOpen(1)]; %line5-1.��FMin����close�б�,����FMin����SetClose(end),ͬʱ��open�б���ɾ���õ㣻
%     SetOpen(1)=[]; %line5-2.��FMin����close�б�ͬʱ��open�б���ɾ���õ㣻
%     Surround=[];
%     %         %% �㷨4���ڽ��ڵ���ѡ
%     %         %���룺A*�㷨�е�close�б�
%     %         %������Ż����close�б�
%     %         %%%��������%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     %         L_Close=length(SetClose);
%     %         ComPoint=[];
%     %         if L_Close>2
%     %             ComPoint=SetClose(end).father;
%     %             while ~(ComPoint.row==start_row && ComPoint.col==start_col)
%     %                 if ((SetClose(end).row-ComPoint.row)^2+(SetClose(end).col-ComPoint.col)^2)<(ComPoint.R)^2
%     %                     SetClose(end).father=ComPoint;
%     %                     SetClose(end).G=ComPoint.G+movecost+movecost*map(ComPoint.row,ComPoint.col);
%     %                 end
%     %                 ComPoint=ComPoint.father;
%     %             end
%     %         end
%     %         SetClose(end).father=ComPoint;
% 
%     %�䲽������%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     %line7. ���� FMin �ڵ�ĳͷ�ֵ��С������һ��Ӧ���ƶ��Ĳ��� ShipSpeed��
%     ShipSpeed=Movelength * (1-map(SetClose(end).row,SetClose(end).col));
%     if ShipSpeed<1
%         ShipSpeed=1;
%     end
%     %          ShipSpeed=Movelength;
%     %line8.���㴬���ƶ�һ���ľ������movecost��Ӧ��չ������ڵ���Num��
%     movecost=10; %���Ϊ���ٵ�A*������movecost�������ı�
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     for ii=1:SurroundPointsNum  %line9. For ���ɵ�ÿһ������ڵ�Surround(i)��
%         Surround(ii).row=floor(SetClose(end).row-ShipSpeed*sin((ii-1)*RudderAngle));
%         Surround(ii).col=floor(SetClose(end).col+ShipSpeed*cos((ii-1)*RudderAngle));
%         Surround(ii).R= ShipSpeed;
%         Surround(ii).Dir = ShipDirection(SetClose(end).row,SetClose(end).col,Surround(ii).row,Surround(ii).col);
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         %%%���ټ������ڵ������
%         if ~isempty( SetOpen)
%             openitear=1;
%             mindis = 1000;
%             while (openitear<length(SetOpen))
%                 dis=sqrt((Surround(ii).row -SetOpen(openitear).row)^2+(Surround(ii).col-SetOpen(openitear).col)^2);
%                 if(dis<mindis)
%                     mindis=dis;
%                     replace=openitear;
%                 end
%                 openitear=openitear+1;
%             end
%             if (mindis<Movelength/4 && ObstacleInMove(background,Surround(ii).row,Surround(ii).col,SetOpen(replace).row,SetOpen(replace).col,ShipLong/2)==1)
%                 %                         if (mindis<6)
%                 Surround(ii).row=SetOpen(replace).row;
%                 Surround(ii).col=SetOpen(replace).col;
%             end
%         end
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         % line10.If(Sourround(i)��Ŀ������Χ֮�ڣ���Sourround(i)�������ϰ����Sourround(i)����close�б���,�Ҵ�FMin�ƶ���Sourround(i)�����д�����ȫ,���˶����̲��ܴ����˶���������)
%         if (0>=Surround(ii).col||Surround(ii).col>=length(background(1,:))||0>=Surround(ii).row||Surround(ii).row>=length(background(:,1))...
%                 || background(Surround(ii).row,Surround(ii).col)==1 ||alreadyexist(Surround(ii),SetClose)==1 ...
%                 ||ObstacleInMove(background,SetClose(end).row,SetClose(end).col,Surround(ii).row,Surround(ii).col,ShipLong/2)==0 ...
%                 ||ObstacleInDomain(background,Surround(ii).row,Surround(ii).col,ShipLong/2)==0)...
%                 ||PointsCanReach(ShipSpeed,Rmin,Surround(ii).Dir,SetClose(end).Dir)==0
%         else
%             %line11. ����Sourround(i)��G��H��Fֵ,����FMinΪSourround(i)�ĸ��ڵ㣻
%             Surround(ii).H=sqrt((destination.col-Surround(ii).col)^2+(destination.row-Surround(ii).row)^2);
%             Surround(ii).G=SetClose(end).G+movecost+valueAPF*movecost*map(Surround(ii).row,Surround(ii).col);%movecost���ڵ����Ƴ�����ֵ
%             Surround(ii).F=Surround(ii).G+Surround(ii).H;
%             Surround(ii).father=SetClose(end); %����FMinΪSourround(i)�ĸ��ڵ㣻
% 
%             if alreadyexist(Surround(ii),SetOpen)==0 %line12. If(Sourround(i)�������겻ͬ��open�б������������)
%                 SetOpen=[SetOpen;Surround(ii)]; %line13. ��Sourround(i)����open�б�
%             else %line14
%                 % line15.�Ƚ�Sourround(i)��open�б��о�����ͬ����ڵ��Gֵ�����ý�С�ߵĸ��ڵ�ΪFMin��
%                 for kk=1:length(SetOpen)
%                     %                         if abs(Surround(ii).row - SetOpen(kk).row)<=1/4*ShipLong && abs(Surround(ii).col-SetOpen(kk).col)<=1/4*ShipLong
%                     if (Surround(ii).row == SetOpen(kk).row && Surround(ii).col==SetOpen(kk).col)
%                         rember=kk;                       %�ҵ�Sourround(i)��open�б��о�����ͬ����Ľڵ�
%                     end
%                 end
%                 if Surround(ii).G < SetOpen(rember).G     %�Ƚ�Gֵ
%                     SetOpen(rember).father=SetClose(end); %���ý�С�ߵĸ��ڵ�ΪFMin��
%                 end
%             end %line16.
%         end     %line17.
%     end         %line18.
%     if SetClose(end).H < ShipSpeed %line19. ���FMin��Ŀ���ľ���С���ƶ��������㷨������
%         break;
%     end
% end
% destination.father=SetClose(end);
% destination.Dir=ShipDirection(SetClose(end).row,SetClose(end).col,end_row,end_col);
% CurrentPoint=destination;
% decision=[];
% while ~(CurrentPoint.row==start_point.row && CurrentPoint.col==start_point.col)
%     Dec_pos=[CurrentPoint.row CurrentPoint.col];
%     Dec_course=CurrentPoint.Dir;
%     decision=[decision;Dec_pos,Dec_course];
% end
% 
% % %����·��
% % figure
% % %     background=K;
% % imshow(background);
% % rectangle('position',[1 1 size(background)-1],'edgecolor','k')%����ͼƬ�߿��С����ɫ
% % t=1;
% % M(t)=getframe;
% % t=t+1;
% % 
% % background2(:,:,1)=background;
% % background2(:,:,2)=background;
% % background2(:,:,3)=background;
% % CurrentPoint=destination;
% % 
% % while ~(CurrentPoint.row==start_point.row && CurrentPoint.col==start_point.col)
% %     position=[CurrentPoint.row CurrentPoint.col];
% %     plot(position,'o','Color','y');
% %     CurrentPoint=CurrentPoint.father;
% %     M(t)=getframe;
% %     t=t+1;
% % end
% % plot([start_point.col,start_point.row],'*','Color','w');
% % plot([destination.col,destination.row],'*','Color','w');
% disp(['����������ʱ�䣺',num2str(etime(clock,t1))]);

