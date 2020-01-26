function Data=AStar2(map,start_row,start_col,end_row,end_col,speed,OSLength,Course,dire,Node_opti)
% ����Ѧ˫�ɵ�ԭA���㷨�ı࣬���ݱ������Ҫ�����������ɺ������޸��˲��ֲ���
% �㷨5:����A*�㷨
%����: �ͷ�ͼ��(����)map,���ͼ������(start_row,start_col),Ŀ���ͼ������(destination_row, destination_col),��������ShipLong,���ذ��Rmin
%���: �������ĵ㼯open�б���ѡΪ����·���ڵ�ĵ㼯close�б�

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
end
