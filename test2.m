%% 贝叶斯推断测试
%本程序用于测试贝叶斯推断函数
load pos.mat
%此处用pos1做本船，pos2做目标船，当前为第21s
OS.currentPos=pos1(21,:);
TS.currentPos=pos2(21,:);

% BAYESIANINTENTIONPRED 用于船舶意图预测
% 参考论文：Bayesian Intention Inference for Trajectory Prediction with an Unknown Goal Destination
% inputs：
%    OtherTrack: n*2数组，他船轨迹(n>=2)
%                此处用pos1做本船，pos2做目标船，前100s轨迹做预测

OtherTrack=pos2(1:20,:);
%    likelihood: 2*2数组,likelihood(1,1)本船猜测他船从本船船头经过的似然度
%                        likelihood(1,2)本船猜测他船从本船船尾经过的似然度
%                        likelihood(2,1)本船猜测他船，猜测本船从他船船头经过的似然度
%                        likelihood(2,2)本船猜测他船，猜测本船从他船船尾经过的似然度
%                likelihood为对称矩阵，例如[0.3, 0.7; 0.7, 0.3]
likelihood=[0.3, 0.7; 0.7, 0.3];

%    pointOfPass: 2*2数组，pointOfPass(1,:)本船猜测他船从本船船头经过的点
%                          pointOfPass(2,:)本船猜测他船从本船船尾经过的点


pointOfPass=

map=


function Astarmap = BayesianIntentionPred(OtherTrack, pointOfPass, likelihood, map);
