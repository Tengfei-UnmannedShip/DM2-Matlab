function Astarmap = BayesianIntentionPred(OtherTrack, pointOfPass, likelihood, map)
% BAYESIANINTENTIONPRED 用于船舶意图预测
% 参考论文：Bayesian Intention Inference for Trajectory Prediction with an Unknown Goal Destination
% inputs：
%    OtherTrack:n*2数组，他船轨迹(n>=2)
%    likelihood: 2*2数组,likelihood(1,1)本船猜测他船从本船船头经过的似然度
%                        likelihood(1,2)本船猜测他船从本船船尾经过的似然度
%                        likelihood(2,1)本船猜测他船，猜测本船从他船船头经过的似然度
%                        likelihood(2,2)本船猜测他船，猜测本船从他船船尾经过的似然度
%                likelihood为对称矩阵，例如[0.3, 0.7; 0.7, 0.3]
%    pointOfPass: 2*2数组，pointOfPass(1,:)本船猜测他船从本船船头经过的点
%                          pointOfPass(2,:)本船猜测他船从本船船尾经过的点


alpha = 1;
N = 100; % 蒙特卡洛取样次数
k = 100; % 预测步数

n = size(OtherTrack, 1);
m = size(likelihood, 2);
goalPoints = pointOfPass; %船头船尾的目标点
PrXTheta = zeros(n-1, m); % 文献中公式(1)
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
        PrXTheta(i, j) = tempP(1)/sum(tempP); %PrXTheta是公式（1）的结果
    end
end

PrTheta = zeros(n, m);
PrTheta(1, :) = likelihood(1, :);
for i=1:n-1     % 文献中公式(2)
	PrTheta(i+1, :) = PrTheta(i, :).*PrXTheta(i, :);
    PrTheta(i+1, :) = PrTheta(i+1, :)./sum(PrTheta(i+1, :)); %归一化
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
        tempX(1, :) = pos2 + tempSpeed * [cosd(tempAlpha), sind(tempAlpha)];   %当前位置不变方向
        tempX(2, :) = pos2 + tempSpeed * [cosd(tempAlpha+45), sind(tempAlpha+45)];   %+45度
        tempX(3, :) = pos2 + tempSpeed * [cosd(tempAlpha-45), sind(tempAlpha-45)];   %-45度
        for ii=1:m       %更新一步的PrXTheta
            for jj=1:3   
                tempDistans1 = sqrt(sum((pos2-tempX(jj, :)).^2));
                tempDistans2 = sqrt(sum((goalPoints(ii, :)-tempX(jj, :)).^2));  
                tempDistans3 = sqrt(sum((goalPoints(ii, :)-pos2).^2));
                tempP(jj, ii) = exp(-alpha*(tempDistans1+tempDistans2-tempDistans3)); %公式（3）第一部分
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

