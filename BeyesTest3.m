%% ��Ҷ˹�ƶϲ���
%���������ڲ��Ա�Ҷ˹�ƶϺ���
%�ؼ��ǿ��ǵ�ͼ��ͳһ
clear
load pos.mat
%�˴���pos1��������pos2��Ŀ�괬����ǰΪ��21s
boatsize = [ 250, 30
    250, 30
    250, 30
    250, 30
    ];

%��ǰ������Ŀ�괬��ϵ
CAL=[2 0 0 1
    1 2 1 0
    1 0 2 0
    0 1 1 2];

for figNO=1:1:9
    if figNO>1
        time=300*(figNO-1); %ѡȡʱ�̷ֱ�Ϊ50��300*(figNO-1)
    else
        time=50;
    end
    OS.speed=18/3600;
    TS(1).speed=18/3600;
    TS(2).speed=18/3600;
    TS(3).speed=18/3600;
    
    OS.length=boatsize(1,1)/1852;
    TS(1).length=boatsize(2,1)/1852;
    TS(2).length=boatsize(3,1)/1852;
    TS(3).length=boatsize(4,1)/1852;
    
    OS.pos=pos1(1:time+1,:);
    OS.Course=c1(1:time+1,:);
    
    TS(1).pos=pos2(1:time+1,:);
    TS(2).pos=pos3(1:time+1,:);
    TS(3).pos=pos4(1:time+1,:);
    
    TS(1).Course=c2(1:time+1,:);
    TS(2).Course=c3(1:time+1,:);
    TS(3).Course=c4(1:time+1,:);
    
    % [X,Y]=meshgrid(0:0.01:6,0:0.01:6);      % Pnew = (Pold - Pref)*scale,����Pref�ǣ�0,0����������ʼλ��ֻ���ǣ�0,0��,15 Nov 2019, by WS
    [X,Y]=meshgrid(-10:0.01:10,-10:0.01:10);  % ע���ͼ�ĳߴ磬ÿһ��������0.01��������ģ������Ҫ��������ӵļ��������µİ��ţ�����˼������õ�������
    [m0,n0]=size(X);
    map=zeros(m0,n0);
    %     TotalIntentionMap0(figNO)=zeros(m0,n0);
    %     TotalIntentionMap(figNO)=zeros(m0,n0);
    
    for TSi=1:1:length(TS)
        IntentionMap0(:,:)=zeros(m0,n0);
        IntentionMap(:,:)=zeros(m0,n0);
        
        OS_waypoint.pos=OS.pos(end,:);
        OS_waypoint.Course=OS.Course(end,:);
        OS_waypoint.speed=OS.speed;
        OS_waypoint.length=OS.length;
        TS_waypoint.pos=TS(TSi).pos(end,:);
        TS_waypoint.Course=TS(TSi).Course(end,:);
        TS_waypoint.speed=TS.speed;
        TS_waypoint.length=TS.length;
        
        WayPointOT0 = WayPoint(OS_waypoint,TS_waypoint,1500);
        WayPointTO0 = WayPoint(TS_waypoint,OS_waypoint,1500);
        
        WayPointOT = floor((WayPointOT0+10*ones(size(WayPointOT0)))*100); %���ڵ�ͼ��-10:0.01:10,���ԣ�ÿһ������Ϊ��λ��Ҫ�Ŵ�100��ȡ����������ĳһ������
        WayPointTO = floor((WayPointTO0+10*ones(size(WayPointTO0)))*100);
        % BAYESIANINTENTIONPRED ���ڴ�����ͼԤ��
        % �ο����ģ�Bayesian Intention Inference for Trajectory Prediction with an Unknown Goal Destination
        % inputs��
        %    OtherTrack: n*2���飬�����켣(n>=2)
        %                �˴���pos1��������pos2��Ŀ�괬��ǰ100s�켣��Ԥ��
        %         OtherTrack0=pos2(1:50,:);
        OtherTrack0=TS(TSi).pos(1:time,:);
        OtherTrack=floor((OtherTrack0+10*ones(size(OtherTrack0)))*100);
        %    likelihood: 2*2����,likelihood(1,1)�����²������ӱ�����ͷ��������Ȼ��
        %                        likelihood(1,2)�����²������ӱ�����β��������Ȼ��
        %                        likelihood(2,1)�����²��������²Ȿ����������ͷ��������Ȼ��
        %                        likelihood(2,2)�����²��������²Ȿ����������β��������Ȼ��
        %                likelihoodΪ�Գƾ�������[0.3, 0.7; 0.7, 0.3]
        likelihood=[0.3, 0.7;
            0.7, 0.3];
        if CAL(1,TSi+1)==1
            likelihood=[0.05, 0.95;
                0.95, 0.05];
        else
            likelihood=[0.95, 0.05;
                0.05, 0.95];
        end
        %    pointOfPass: 2*2���飬pointOfPass(1,:)�����²������ӱ�����ͷ�����ĵ�
        %                          pointOfPass(2,:)�����²������ӱ�����β�����ĵ�
        
        pointOfPass=[WayPointTO(1:2)
            WayPointTO(3:4)] ;
        
        %     IntentionMap0=BayesianIntentionPred(OtherTrack, pointOfPass, likelihood, map);
        %% ��Ҷ˹�ƶ�
        alpha = 1;
        N = 100; % ���ؿ���ȡ������
        k = 500; % Ԥ�ⲽ��
        
        n = size(OtherTrack, 1);
        m = size(likelihood, 2);
        goalPoints = pointOfPass; %��ͷ��β��Ŀ���
        PrXTheta = zeros(n-1, m); % �����й�ʽ(1)
        for i=1:n-1
            for j=1:m
                tempX = zeros(3, 2);
                tempP = zeros(1, 3);
                tempX(1, :) = OtherTrack(i+1, :);
                tempArrow = OtherTrack(i+1, :) - OtherTrack(i, :);
                tempSpeed = sqrt(sum(tempArrow.^2));
                tempAlpha = atan2d(tempArrow(2), tempArrow(1));
                tempX(2, :) = OtherTrack(i, :) + tempSpeed * [cosd(tempAlpha+45), sind(tempAlpha+45)];
                tempX(3, :) = OtherTrack(i, :) + tempSpeed * [cosd(tempAlpha-45), sind(tempAlpha-45)];
                for jj=1:3
                    tempDistans1 = sqrt(sum((OtherTrack(i, :)-tempX(jj, :)).^2));
                    tempDistans2 = sqrt(sum((goalPoints(j, :)-tempX(jj, :)).^2));
                    tempDistans3 = sqrt(sum((goalPoints(j, :)-OtherTrack(i, :)).^2));
                    tempP(jj) = exp(-alpha*(tempDistans1+tempDistans2-tempDistans3));
                end
                PrXTheta(i, j) = tempP(1)/sum(tempP); %PrXTheta�ǹ�ʽ��1���Ľ��
            end
        end
        
        PrTheta = zeros(n, m);
        PrTheta(1, :) = likelihood(1, :);
        for i=1:n-1     % �����й�ʽ(2)
            PrTheta(i+1, :) = PrTheta(i, :).*PrXTheta(i, :);
            PrTheta(i+1, :) = PrTheta(i+1, :)./sum(PrTheta(i+1, :)); %��һ��
        end
        
        for i=1:1:N      %��ʼ���ؿ������
            Bpos1 = OtherTrack(n-1, :); %��ǰʱ�̺���һ��ʱ�̵�λ�ã����ڽ��б�Ҷ˹�ƶϣ�������ʱ�̵ļ������˲���
            Bpos2 = OtherTrack(n, :);
            for j=1:k
                PrX = zeros(1, 3);
                tempX = zeros(3, 2);
                tempP = zeros(3, m);
                tempArrow = Bpos2 - Bpos1;
                tempSpeed = sqrt(sum(tempArrow.^2));  %��ǰ���ٶȣ�������������λ��֮���ʵ�ʾ���
                tempAlpha = atan2d(tempArrow(2), tempArrow(1));   %��ǰ�ĺ����
                tempX(1, :) = Bpos2 + tempSpeed * [cosd(tempAlpha), sind(tempAlpha)];   %��ǰλ�ò��䷽��
                tempX(2, :) = Bpos2 + tempSpeed * [cosd(tempAlpha+45), sind(tempAlpha+45)];   %+45��
                tempX(3, :) = Bpos2 + tempSpeed * [cosd(tempAlpha-45), sind(tempAlpha-45)];   %-45��
                for ii=1:m       %����һ����PrXTheta
                    for jj=1:3
                        tempDistans1 = sqrt(sum((Bpos2-tempX(jj, :)).^2));
                        tempDistans2 = sqrt(sum((goalPoints(ii, :)-tempX(jj, :)).^2));
                        tempDistans3 = sqrt(sum((goalPoints(ii, :)-Bpos2).^2));
                        tempP(jj, ii) = exp(-alpha*(tempDistans1+tempDistans2-tempDistans3)); %��ʽ��3����һ����
                    end
                    tempP(:, ii) = tempP(:, ii)/sum(tempP(:, ii));
                end
                for jj=1:3
                    for ii=1:m
                        PrX(jj) = PrX(jj)+tempP(jj, ii)*PrTheta(n, ii);
                    end
                end
                select = rand();
                upper = [PrX(1), PrX(1)+PrX(2), 1];
                for jj=1:3
                    if select < upper(jj)
                        break;
                    end
                end
                Bpos1 = Bpos2;
                Bpos2 = tempX(jj, :);
                point0= Bpos2;
                point = floor(Bpos2);
                map(point(2), point(1)) = map(point(2), point(1)) + 1;
            end
        end
        IntentionMap0(:,:)=map;
        
        [row,col]=find(IntentionMap0(:,:)~=0);%���ؾ���B�з���Ԫ�ض�Ӧ���к���
        TS(TSi).pos(1)=row(1);
        TS(TSi).pos(2)=col(1);
        WayPointTO = WayPoint( TS_waypoint,OS_waypoint,1500 );
        pointOfPass=[WayPointTO(1:2);WayPointTO(3:4)] ;
        IntentionMap(:,:)=IntentionMap(:,:)+IntentionMap0(:,:);
        %�洢����
        if TSi==1
            IntentionMap2(:,:)=IntentionMap(:,:);
        elseif TSi==2
            IntentionMap3(:,:)=IntentionMap(:,:);
        else
            IntentionMap4(:,:)=IntentionMap(:,:);
        end
    end
    TotalMap(figNO).Ship2(:,:)=IntentionMap2(:,:,:);
    TotalMap(figNO).Ship3(:,:)=IntentionMap3(:,:,:);
    TotalMap(figNO).Ship4(:,:)=IntentionMap4(:,:,:);
end

%% ��������
save('IntentionMapData','TotalMap','-append');

%% ��ͼ����

figure
ha = MarginEdit(3,3,[.05  0.05],[.05  0.05],[0.05  0.05],1);
for figNO=1:1:9
    if figNO>1
        k=300*(figNO-1); %ѡȡʱ�̷ֱ�Ϊ50��300*(figNO-1)
    else
        k=50;
    end
    
%     subplot(3,2,fig);
%     hold on;
    axes(ha(fig)); 
    %WTF:���������ĳ�ʼλ��
    drawShip0(pos1(1,:),c1(1),1,400);
    drawShip0(pos2(1,:),c2(1),2,400);
    drawShip0(pos3(1,:),c3(1),3,400);
    drawShip0(pos4(1,:),c4(1),4,400);
    
    %WTF:���������Ľ���λ��
    drawShip(pos1(k,:),c1(k),1,400);
    drawShip(pos2(k,:),c2(k),2,400);
    drawShip(pos3(k,:),c3(k),3,400);
    drawShip(pos4(k,:),c4(k),4,400);
    
    %WTF:���������ĺ���ͼ
    plot(pos1(1:k,1),pos1(1:k,2),'r-');
    plot(pos2(1:k,1),pos2(1:k,2),'g-');
    plot(pos3(1:k,1),pos3(1:k,2),'b-');
    plot(pos4(1:k,1),pos4(1:k,2),'k-');
     
%     %WTF:������ͷ��Բ���ڱ�ʾ��ȫ��Χ
%     circle(pos1(k,:),900/1852,1);
%     circle(pos2(k,:),900/1852,2);
%     circle(pos3(k,:),900/1852,3);
%     circle(pos4(k,:),900/1852,4);
    
    %WTF:����2~4�Ŵ���ͷ��Ԥ��켣
    
    
    
    
    
    
    grid on;
    axis on
    xlabel('\it n miles', 'Fontname', 'Times New Roman','FontSize',15);
    ylabel('\it n miles', 'Fontname', 'Times New Roman','FontSize',15);
    title(['t=',num2str(k),'s'], 'Fontname', 'Times New Roman','FontSize',15);
    box on;
end




