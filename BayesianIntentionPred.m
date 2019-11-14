function Astarmap = BayesianIntentionPred(OtherTrack, pointOfPass, likelihood, map)
% BAYESIANINTENTIONPRED ���ڴ�����ͼԤ��
% �ο����ģ�Bayesian Intention Inference for Trajectory Prediction with an Unknown Goal Destination
% inputs��
%    OtherTrack:n*2���飬�����켣(n>=2)
%    likelihood: 2*2����,likelihood(1,1)�����²������ӱ�����ͷ��������Ȼ��
%                        likelihood(1,2)�����²������ӱ�����β��������Ȼ��
%                        likelihood(2,1)�����²��������²Ȿ����������ͷ��������Ȼ��
%                        likelihood(2,2)�����²��������²Ȿ����������β��������Ȼ��
%                likelihoodΪ�Գƾ�������[0.3, 0.7; 0.7, 0.3]
%    pointOfPass: 2*2���飬pointOfPass(1,:)�����²������ӱ�����ͷ�����ĵ�
%                          pointOfPass(2,:)�����²������ӱ�����β�����ĵ�


alpha = 1;
N = 100; % ���ؿ���ȡ������
k = 100; % Ԥ�ⲽ��

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

for i=1:N 
    pos1 = OtherTrack(n-1, :); 
    pos2 = OtherTrack(n, :);
    for j=1:k
        PrX = zeros(1, 3);
        tempX = zeros(3, 2);
        tempP = zeros(3, m);
        tempArrow = pos2 - pos1;
        tempSpeed = sqrt(sum(tempArrow.^2));
        tempAlpha = atan2d(tempArrow(2), tempArrow(1)); 
        tempX(1, :) = pos2 + tempSpeed * [cosd(tempAlpha), sind(tempAlpha)];   %��ǰλ�ò��䷽��
        tempX(2, :) = pos2 + tempSpeed * [cosd(tempAlpha+45), sind(tempAlpha+45)];   %+45��
        tempX(3, :) = pos2 + tempSpeed * [cosd(tempAlpha-45), sind(tempAlpha-45)];   %-45��
        for ii=1:m       %����һ����PrXTheta
            for jj=1:3   
                tempDistans1 = sqrt(sum((pos2-tempX(jj, :)).^2));
                tempDistans2 = sqrt(sum((goalPoints(ii, :)-tempX(jj, :)).^2));  
                tempDistans3 = sqrt(sum((goalPoints(ii, :)-pos2).^2));
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
        pos1 = pos2;
        pos2 = tempX(jj, :);
        point = floor(pos2);
        map(point(2), point(1)) = map(point(2), point(1)) + 1;
    end
end
Astarmap=map;
end

