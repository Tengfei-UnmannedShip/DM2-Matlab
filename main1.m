%% DecisionMaking-2������������
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 1.2019��10��20�յ�һ��
% 2.���԰汾���㷨���Ϊ��
% ��1��׼������
% ��2���жϵ�ǰ״̬����·��ֱ����״̬��Fun1��
% ��3���ж�CPA������;��ͷ
% ��4��
% 3.��ǰ��Ԥ�⣬ֻ����
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
clc;
% close all;
tic;%tic1
t1=clock;
%% WTF:������ʼ��
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
drawAPFtime=[1,500,1000,1500,2500];
tt=2000;
for j=1:1:ShipNum
    
    ship(j).speed = boat(j,3)*1852/3600;%WTF������Ϊ18����/Сʱ��1����=1.852������1����/Сʱ=1.852���Сʱ=1852/3600��/��
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
    ship(j).pos=[boat(j,1)-ship(j).speed*sind(ship(j).initialCourse)*0.5*t, boat(j,2)-ship(j).speed*cosd(ship(j).initialCourse)*0.5*t];
    %   ship(i).pos = boat(i,1:2)*1852;
    ship(j).courseAlter = 0; %��ʼ״̬�ĺ���Ǹñ���Ϊ0
    ship(j).Course = ship(j).initialCourse+ship(j).courseAlter;
    ship(j).DCPA_Record = [];
    ship(j).TCPA_Record = [];
    ship(j).length=boatsize(j,1);
    ship(j).width=boatsize(j,2);
    GoalRange=MapSize-[1.5,1.5];
    ship(j).goalPiont= Goal_point(ship(j).pos(1),ship(j).pos(2),ship(j).Course,GoalRange);
end

pos1=zeros(t,2);
pos1(1,:)=ship(1).pos;%WTF������λ�þ����һ�У�����t��ֵ��δȷ����%%�²⴬��ԭ�����λ��
pos2=zeros(t,2);
pos2(1,:)=ship(2).pos;
pos3=zeros(t,2);
pos3(1,:)=ship(3).pos;
pos4=zeros(t,2);
pos4(1,:)=ship(4).pos;

OSdecision=[0 0 0 0];
OSdecision_time=[ ];
OSdecConut=zeros(t,4);
s=0;
%% WTF:��ʼִ�зֲ�ʽ����
for i=2:t
    %% ȫ�ֻ���
    tic ;%tic2
    t2=clock;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Update%WTF��������Ϣ����
    for j=1:ShipNum
        if ship(j).decision_lable~=0 && OSdecision(j)~=0 && ship(j).compliance~=0 %����ָ�겻Ϊ0,���Ѿ����߹����������������ߣ�λ�ü���������һ����A*���߽����ֱ����һ��
            ship(j).pos=ship(j).DM_pos(i-ship(j).decision_lable,2*j-1:2*j);
            ship(j).Course=ship(j).DM_c(i-ship(j).decision_lable,j);
            ship(j).courseAlter=ship(j).Course-ship(j).initialCourse;
            ship(j).Vratio=ship(j).DM_r(i-ship(j).decision_lable,j);
        else                           %������������Ϊ0��˵���������ߣ����򣬱������ճ���״̬����
            ship(j).Course=ship(j).initialCourse+ship(j).courseAlter;
            speed_now=ship(j).Vratio*ship(j).speed;
            %WTF�������ߵĻ�����ǰ�Ĵ���λ��Ϊ��ԭλ��x+1*��ǰ�ٶ�v*sin��ԭ�����alpha����ԭλ��y+1*��ǰ�ٶ�v*cos��ԭ�����alpha����
            ship(j).pos =[ship(j).pos(1)+speed_now*sind(ship(j).Course),ship(j).pos(2)+speed_now*cosd(ship(j).Course)];
        end
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    pos1(i,:)=ship(1).pos;%WTF�����䴬��λ�þ���ÿһһ��ѭ��Ϊһ�룬ÿһ�����λ�þ����¼�һ�У�������1500�С�
    pos2(i,:)=ship(2).pos;
    pos3(i,:)=ship(3).pos;
    pos4(i,:)=ship(4).pos;
    
    c1(i,:)=ship(1).Course;%WTF�����䴬���������ÿ��ѭ�����´�����ǰ����ԭ�����alpha+��ǰ��ƫת��beta����������1500�С�
    c2(i,:)=ship(2).Course;
    c3(i,:)=ship(3).Course;
    c4(i,:)=ship(4).Course;
    
    %% ������ʼ����%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for OS=1:1:ShipNum  %�ӵ�һ�Ҵ��������Ҵ��ı���
        if decisioncycle(OS,i,ship)&& ship(OS).compliance~=0%�жϵ�ǰiʱ������j���ľ�����������compliance==1����������
            %compliance=0 ˵����ȫ���䣬��ʱ�����߾ͺã�compliance=2 ������
            risk_neighbor_temp1=SearchNeighbor(ship(OS),ship,ship(OS).range);  %���뱾����ⷶΧ�Ĵ�
            if ~isempty(risk_neighbor_temp1) && CollisionRisk0(ship(OS),risk_neighbor_temp1) %WTF:�������ٴ�����ײ����ʱ
                % �������ߴ�������
                TS=risk_neighbor_temp1; %����TS��˳����ship��˳��ȥ��OS֮�����
                OSdecConut(i,OS)=1;
                OSdecision(OS)=OSdecision(OS)+1;
                OSdecision_time(OS,OSdecision(OS))=i;
                disp(['t=',num2str(i),',  ','��',num2str(OS),'�Ҵ��ĵ�',num2str(OSdecision(OS)),'�ξ���']);
                % STEP 0.���������ǰ��ʶ�𵽷��մ������ڵ�APFͼ����Ϊ���յ�ͼ
                if i==ismember(drawAPFtime,1)   %ȷ���Ƿ�APF��ͼ
                    drawAPF=1;
                else
                    drawAPF=0;
                end
                APF  = DrawAPF(ship(OS),TS, MapSize, drawAPF );
                % STEP 1.�ж�TSi����·������ֱ����
                TS_GSR = GSR(ship(OS), TS);
                % STEP 2.�ҵ�CPA��������ǰ1.5������Ϊtheta1�����1������Ϊtheta2
                WayPointTS = WayPoint(ship(OS),TS,t);     %OS���и���TS��WayPoint
                for i_wp=1:1:length(TS)                    %����TS����OS��WayPoint
                    WayPointOS = WayPoint(TS(i_wp),ship(OS),t);
                end
                %% inferLabbel==0 ʱ�������Ʋ�������
                if ship(OS).inferLabbel==0 %�����������������Ʋ⣻
                    %���Ʋ��ʱ��ֱ���ڷ��յ�ͼ�Ͼ���
                    [SetClose,SetOpen]=AStar1(APF.map0,ship(OS).pos(1),ship(OS).pos(2),ship(OS).goalPiont(1),...
                        ship(OS).goalPiont(2),ship(OS).speed,ship(OS).length,MapSize,1);%ֱ�ӵ��ú�����% ԭ������Ѧ˫��
                else
                    %% inferLabbel==1 �����������ߣ���֤�������Ƶ�����ǰ��CAL����
                    % neighbor_temp=SearchNeighbor(ship(j),ship,ship(j).range);%��j�Ҵ����ڴ����������ⷶΧ�����д�
                    % ���������д���Ҫ���ǵ������Բ����жϷ�Χ��
                    % ����ǵ�һ����������
                    if OSdecision(OS)==1  %�ڵ�һ����������
                        %���Ʋ��ʱ��ֱ���ڷ��յ�ͼ�Ͼ���
                        [SetClose,SetOpen]=AStar1(APF.map0,ship(OS).pos(1),ship(OS).pos(2),ship(OS).goalPiont(1),...
                            ship(OS).goalPiont(2),ship(OS).speed,ship(OS).length,MapSize,0);%ֱ�ӵ��ú�����% ԭ������Ѧ˫��
                        
                        
                    else  %�ǵ�һ����������
                        %% 3.�ñ�Ҷ˹�Ʋ�ĳ������ɷ���ͼ
                        % BAYESIANINTENTIONPRED ���ڴ�����ͼԤ��
                        % �ο����ģ�Bayesian Intention Inference for Trajectory Prediction with an Unknown Goal Destination
                        % inputs��
                        %    OtherTrack:n*2���飬�����켣(n>=2)
                        %    likelihood: 2*2���飬likelihood(1,1)�����²������ӱ�����ͷ��������Ȼ��
                        %                        likelihood(1,2)�����²������ӱ�����β��������Ȼ��
                        %                        likelihood(2,1)�����²��������²Ȿ����������ͷ��������Ȼ��
                        %                        likelihood(2,2)�����²��������²Ȿ����������β��������Ȼ��
                        %                likelihoodΪ�Գƾ�������[0.3, 0.7; 0.7, 0.3]
                        %    pointOfPass: 2*2���飬pointOfPass(1,:)�����²������ӱ�����ͷ�����ĵ�
                        %                          pointOfPass(2,:)�����²������ӱ�����β�����ĵ�
                        [Astarmap] = BayesianIntentionPred(OtherTrack, pointOfPass, likelihood, map);
                        %% 4.��A*�㷨���ɱ���·��
                        
                        [SetClose,SetOpen]=AStar1(APF.map0,ship(OS).pos(1),ship(OS).pos(2),ship(OS).goalPiont(1),...
                            ship(OS).goalPiont(2),ship(OS).speed,ship(OS).length,MapSize,1);%ֱ�ӵ��ú�����% ԭ������Ѧ˫��
                        
                    end
                end
            end
        end
    end
end
