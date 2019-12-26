
clear
tic;%tic1
t1=clock;

load pos.mat

%�˴���pos1��������pos2��Ŀ�괬����ǰΪ��21s
boatsize = [ 250, 30
    250, 30
    250, 30
    250, 30
    ];

%��ǰ������Ŀ�괬��ϵ
% �磺��һ�Ҵ���2��3����β������4�Ŵ�ͷ��
CAL=[2 0 0 1
    1 2 1 0
    1 0 2 0
    0 1 1 2];

% [X,Y]=meshgrid(0:0.01:6,0:0.01:6);      % Pnew = (Pold - Pref)*scale,����Pref�ǣ�0,0����������ʼλ��ֻ���ǣ�0,0��,15 Nov 2019, by WS
[X,Y]=meshgrid(-10:0.01:10,-10:0.01:10);  % ע���ͼ�ĳߴ磬ÿһ��������0.01��������ģ������Ҫ��������ӵļ��������µİ��ţ�����˼������õ�������
[m0,n0]=size(X);


for figNO=1:1:1
    if figNO>1
        time=300*(figNO-1); %ѡȡʱ�̷ֱ�Ϊ50��300*(figNO-1)
    else
        time=50;
    end
    for i=1:4
        ship(i).speed=18/3600;
        ship(i).length=boatsize(1,1)/1852;
        ship(i).pos=pos1(1:time+1,:);
        ship(i).Course=c1(1:time+1,:);
        if i==1
            OS=ship(i);
        else  
            TS(i-1)=ship(i);
        end
    end
    
    for TSi=1:1:1
        
        IntentionMap2=zeros(m0,n0);
        IntentionMap3=zeros(m0,n0);
        IntentionMap4=zeros(m0,n0);
        
        CPA_temp = computeCPA0(OS,TS(TSi),1500);
        CPA(figNO).ship(TSi,:)=CPA_temp;
        CR=CollisionRisk0(OS,TS(TSi),1852);
         if CollisionRisk0(OS,TS(TSi),1852) %�޷���Ϊ0���з���Ϊ1�����з���ʱ��ִ��
            TS(TSi).infer=1; %����ʼԤ��
            disp(['��ǰ������Ŀ�괬',num2str(TSi)+1,'����ײ���գ�DCPA=',num2str(CPA_temp(1,5)),'��TCPA= ',num2str(CPA_temp(1,6))]);
            %�з��գ�ִ��Ԥ��
            
            map=zeros(m0,n0);
            IntentionMap0=zeros(m0,n0);
            
            WP0_real = Find_WP( ship,TSi+1,CAL );
            [Cal_wp,n,data]=WP_allShips(wp_pos,WP0_real);
            %��ȡ data1 �е�һ���������� ��������
            subplot(2,1,1)
            plot(data(1,:))
            title('x1 �ڵ����еı仯')
            %��ȡ data �еĵڶ����������� ������仯����
            subplot(2,1,2)
            %���ڿ��������᷶Χ ʹͼ�������
            plot(data(2,:))
            title('x2 �ڵ����еı仯')
            % BAYESIANINTENTIONPRED ���ڴ�����ͼԤ��
            % �ο����ģ�Bayesian Intention Inference for Trajectory Prediction with an Unknown Goal Destination
            % inputs��
            %    OtherTrack: n*2���飬�����켣(n>=2)
            %                �˴���pos1��������pos2��Ŀ�괬��ǰ100s�켣��Ԥ��
         else
            disp(['��ǰ������Ŀ�괬',num2str(TSi)+1,'����ײ����']);
            
        end
    end
end
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%�����ʱ��tic2��ʱ�䣬�������һ������tic����forѭ����i=3ʱ�����Լ���������һ��ѭ����ʱ��
% disp(['���һ��ѭ������ʱ��',num2str(toc)])
disp(['����������ʱ�䣺',num2str(etime(clock,t1))]);