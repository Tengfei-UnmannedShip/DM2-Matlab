
clear
tic;%tic1
t1=clock;

load pos.mat

%此处用pos1做本船，pos2做目标船，当前为第21s
boatsize = [ 250, 30
    250, 30
    250, 30
    250, 30
    ];

%当前本船与目标船关系
% 如：第一艘船从2，3船船尾过，从4号船头过
CAL=[2 0 0 1
    1 2 1 0
    1 0 2 0
    0 1 1 2];

% [X,Y]=meshgrid(0:0.01:6,0:0.01:6);      % Pnew = (Pold - Pref)*scale,这里Pref是（0,0），所以起始位置只能是（0,0）,15 Nov 2019, by WS
[X,Y]=meshgrid(-10:0.01:10,-10:0.01:10);  % 注意地图的尺寸，每一个格子是0.01海里见方的，因此需要对下面格子的检索进行新的安排，并且思考如何用到函数中
[m0,n0]=size(X);


for figNO=1:1:1
    if figNO>1
        time=300*(figNO-1); %选取时刻分别为50，300*(figNO-1)
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
         if CollisionRisk0(OS,TS(TSi),1852) %无风险为0，有风险为1，即有风险时才执行
            TS(TSi).infer=1; %即开始预测
            disp(['当前本船与目标船',num2str(TSi)+1,'有碰撞风险，DCPA=',num2str(CPA_temp(1,5)),'，TCPA= ',num2str(CPA_temp(1,6))]);
            %有风险，执行预测
            
            map=zeros(m0,n0);
            IntentionMap0=zeros(m0,n0);
            
            WP0_real = Find_WP( ship,TSi+1,CAL );
            [Cal_wp,n,data]=WP_allShips(wp_pos,WP0_real);
            %抽取 data1 中第一个变量数据 画出曲线
            subplot(2,1,1)
            plot(data(1,:))
            title('x1 在迭代中的变化')
            %抽取 data 中的第二个变量数据 画出其变化曲线
            subplot(2,1,2)
            %用于控制坐标轴范围 使图象更清晰
            plot(data(2,:))
            title('x2 在迭代中的变化')
            % BAYESIANINTENTIONPRED 用于船舶意图预测
            % 参考论文：Bayesian Intention Inference for Trajectory Prediction with an Unknown Goal Destination
            % inputs：
            %    OtherTrack: n*2数组，他船轨迹(n>=2)
            %                此处用pos1做本船，pos2做目标船，前100s轨迹做预测
         else
            disp(['当前本船与目标船',num2str(TSi)+1,'无碰撞风险']);
            
        end
    end
end
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%计算此时到tic2的时间，由于最后一次遇到tic是在for循环的i=3时，所以计算的是最后一次循环的时间
% disp(['最后一次循环运行时间',num2str(toc)])
disp(['程序总运行时间：',num2str(etime(clock,t1))]);