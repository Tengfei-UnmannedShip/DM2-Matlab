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

[X,Y]=meshgrid(-10:0.01:10,-10:0.01:10);    %ע���ͼ�ĳߴ磬ÿһ��������0.01��������ģ������Ҫ��������ӵļ��������µİ��ţ�����˼������õ�������
[m0,n0]=size(X);
map=zeros(m0,n0);
IntentionMap0=zeros(m0,n0);
IntentionMap=zeros(m0,n0);

WayPointOT = floor(WayPoint(OS,TS,1500)*100); %���ڵ�ͼ��-10:0.01:10,���ԣ�ÿһ������Ϊ��λ��Ҫ�Ŵ�100��ȡ����������ĳһ������
WayPointTO = floor(WayPoint(TS,OS,1500)*100);
% BAYESIANINTENTIONPRED ���ڴ�����ͼԤ��
% �ο����ģ�Bayesian Intention Inference for Trajectory Prediction with an Unknown Goal Destination
% inputs��
%    OtherTrack: n*2���飬�����켣(n>=2)
%                �˴���pos1��������pos2��Ŀ�괬��ǰ100s�켣��Ԥ��
OtherTrack0=pos2(1:50,:);
OtherTrack=floor(OtherTrack0*100);
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
IntentionMap0=map;

[row,col]=find(IntentionMap0~=0);%���ؾ���B�з���Ԫ�ض�Ӧ���к���
TS.pos(1)=row(1);
TS.pos(2)=col(1);
WayPointTO = WayPoint( TS,OS,1500 );
pointOfPass=[WayPointTO(1:2);WayPointTO(3:4)] ;
IntentionMap=IntentionMap+IntentionMap0;
v=find(IntentionMap~=0);%����B�з���Ԫ��
[row,col]=find(IntentionMap~=0);%���ؾ���B�з���Ԫ�ض�Ӧ���к���

IntentionMap1=IntentionMap/1;


colorpan=[1,1,1]; %����ɫ��һ��Ϊ0ʱ����ʾ��ɫ
% ����3���ɱ�׼ɫjet�޸Ķ����Ӱ׵�ǳ���������������Ƶ��쵽���
% �ף�λ��1����ɫ[1 1 1]
% ǳ����λ��15����ɫ[173/255 235/255 1](RGB=[173 235 255])
% ����λ��40����ɫ[0 0 1]
% ������λ��55����ɫ[0 1 1]
% �ƣ�λ��70����ɫ[1 1 0]
% �죺λ��90����ɫ[1 0 0]
% ��죺λ��100����ɫ[132/255 0 0](RGB=[132 0 0])
for k=1:1:99
    if k<15
        colorpan=[colorpan;1-((255-173)/255)*(k/14),1-((255-235)/255)*(k/14),1];  % ǳ����λ��15����ɫ[173/255 235/255 1](RGB=[173 235 255])
    elseif k>=15 && k<40
        colorpan=[colorpan;173/255*(1-(k-14)/(39-14)),235/255*(1-(k-14)/(39-14)),1]; % ����λ��40����ɫ[0 0 1]
    elseif k>=40 && k<55
        colorpan=[colorpan;0,(k-39)/(54-39),1]; % ������λ��55����ɫ[0 1 1]
    elseif k>=55 && k<70
        colorpan=[colorpan;(k-54)/(69-54),1,1-((k-54)/(69-54))]; % �ƣ�λ��70����ɫ[1 1 0]
    elseif k>=70 && k<90
        colorpan=[colorpan;1,1-((k-69)/(89-69)),0]; % �죺λ��90����ɫ[1 0 0]
    else
        colorpan=[colorpan;1-(132/255)*((k-89)/(100-89)),0,0]; % ��죺λ��100����ɫ[132/255 0 0](RGB=[132 0 0])
    end
end

ppos1 = OtherTrack0(n-1, :); %��ǰʱ�̺���һ��ʱ�̵�λ�ã����ڽ��б�Ҷ˹�ƶϣ�������ʱ�̵ļ������˲���
ppos2 = OtherTrack0(n, :);
%����ʾ�����ߵ�pcolor
figure
ss=pcolor(X,Y,IntentionMap1);  %����pcolor�Ĺٷ�ʾ��
colormap(colorpan);%����ɫ��
colorbar
set(ss, 'LineStyle','none');
hold on
plot(pos2(40,1),pos2(40,2),'or')
hold on
plot(pos2(50,1),pos2(50,2),'ob')
hold on
plot(0,0,'*')
hold on
xpos=pos2(1:400,1);
ypos=pos2(1:400,2);
plot(xpos,ypos,'LineWidth',1);


