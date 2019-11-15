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
OS.speed=18/3600;
TS.speed=18/3600;
OS.length=boatsize(1,1)/1852;
TS.length=boatsize(2,1)/1852;
OS.pos=pos1(51,:);
OS.Course=c1(51,:);
TS.pos=pos2(51,:);
TS.Course=c1(51,:);

WayPointOT = WayPoint( OS,TS,1500 );
WayPointTO = WayPoint( TS,OS,1500 );
% BAYESIANINTENTIONPRED ���ڴ�����ͼԤ��
% �ο����ģ�Bayesian Intention Inference for Trajectory Prediction with an Unknown Goal Destination
% inputs��
%    OtherTrack: n*2���飬�����켣(n>=2)
%                �˴���pos1��������pos2��Ŀ�괬��ǰ100s�켣��Ԥ��

OtherTrack=pos2(1:10:50*10,:);
%    likelihood: 2*2����,likelihood(1,1)�����²������ӱ�����ͷ��������Ȼ��
%                        likelihood(1,2)�����²������ӱ�����β��������Ȼ��
%                        likelihood(2,1)�����²��������²Ȿ����������ͷ��������Ȼ��
%                        likelihood(2,2)�����²��������²Ȿ����������β��������Ȼ��
%                likelihoodΪ�Գƾ�������[0.3, 0.7; 0.7, 0.3]
likelihood=[0.3, 0.7; 0.7, 0.3];

%    pointOfPass: 2*2���飬pointOfPass(1,:)�����²������ӱ�����ͷ�����ĵ�
%                          pointOfPass(2,:)�����²������ӱ�����β�����ĵ�


pointOfPass=[WayPointTO(1:2)
    WayPointTO(3:4)] ;

[X,Y]=meshgrid(-10:0.01:10,-10:0.01:10);    %ע���ͼ�ĳߴ磬ÿһ��������0.01��������ģ������Ҫ��������ӵļ��������µİ��ţ�����˼������õ�������
[m,n]=size(X);
map=zeros(m,n);
IntentionMap0=zeros(m,n);
IntentionMap=zeros(m,n);
for t=1:1  
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
            point = ceil(Bpos2);    %ceil������ȡ������ʹ��floor������ȡ�������п���ȡ��0����map��û��0
            map(point(2), point(1)) = map(point(2), point(1)) + 1;
        end
    end
    IntentionMap0=map;
    
    [row,col]=find(IntentionMap0~=0);%���ؾ���B�з���Ԫ�ض�Ӧ���к���
    TS.pos(1)=row(1);
    TS.pos(2)=col(1);
    WayPointTO = WayPoint( TS,OS,1500 );
    pointOfPass=[WayPointTO(1:2);WayPointTO(3:4)] ;
    IntentionMap=IntentionMap+IntentionMap0;   
end
v=find(IntentionMap~=0);%����B�з���Ԫ��
[row,col]=find(IntentionMap~=0);%���ؾ���B�з���Ԫ�ض�Ӧ���к���

figure
mesh(X,Y,IntentionMap);
% axis on;
axis equal;
xlabel('\it n miles', 'Fontname', 'Times New Roman');
ylabel('\it n miles', 'Fontname', 'Times New Roman');

