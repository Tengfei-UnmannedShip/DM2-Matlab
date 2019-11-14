clc
clear
load Test1

map = APF.map0;
% ���е���ʼ���Ŀ��㣬��Ҫȡ����

start_row = round((ship(OS).pos(1)+MapSize(1)*1852)/10); 
start_col = round((ship(OS).pos(2)+MapSize(2)*1852)/10);
end_row = round((ship(OS).goalPiont(1)+MapSize(1)*1852)/10);
end_col = round((ship(OS).goalPiont(2)+MapSize(2)*1852)/10);
drawPath = 1;

%% ����Ѧ˫�ɵ��㷨�ı�

% �㷨5:����A*�㷨
%����: �ͷ�ͼ��(����)map,���ͼ������(start_row,start_col),Ŀ���ͼ������(destination_row, destination_col),��������ShipLong,���ذ��Rmin
%���: �������ĵ㼯open�б���ѡΪ����·���ڵ�ĵ㼯close�б�
%% line1. ���ó�ʼ��������
background=map;
start_point.row=start_row;
start_point.col=start_col;
destination.row=end_row;
destination.col=end_col;
ShipLong=4;
Movelength=round(ship(OS).speed/10); %����
SurroundPointsNum=20; %������������n���A*
RudderAngle=2*pi/SurroundPointsNum;
Rmin=2*Movelength/3; %ת��뾶
valueAPF=2;  %APF�Ƴ��ļ�ֵ����
%     Rmin=0;

%��ʼ����
%% line2. ��ʼ׼����
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
%% ��ʼ����
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
    
    %% %%�䲽������%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
                %% line15.�Ƚ�Sourround(i)��open�б��о�����ͬ����ڵ��Gֵ�����ý�С�ߵĸ��ڵ�ΪFMin��
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
%% ����·������
if drawPath==1
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
end

