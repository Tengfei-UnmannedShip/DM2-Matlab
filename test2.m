%% ��Ҷ˹�ƶϲ���
%���������ڲ��Ա�Ҷ˹�ƶϺ���
load pos.mat
%�˴���pos1��������pos2��Ŀ�괬����ǰΪ��21s
OS.currentPos=pos1(21,:);
TS.currentPos=pos2(21,:);

% BAYESIANINTENTIONPRED ���ڴ�����ͼԤ��
% �ο����ģ�Bayesian Intention Inference for Trajectory Prediction with an Unknown Goal Destination
% inputs��
%    OtherTrack: n*2���飬�����켣(n>=2)
%                �˴���pos1��������pos2��Ŀ�괬��ǰ100s�켣��Ԥ��

OtherTrack=pos2(1:20,:);
%    likelihood: 2*2����,likelihood(1,1)�����²������ӱ�����ͷ��������Ȼ��
%                        likelihood(1,2)�����²������ӱ�����β��������Ȼ��
%                        likelihood(2,1)�����²��������²Ȿ����������ͷ��������Ȼ��
%                        likelihood(2,2)�����²��������²Ȿ����������β��������Ȼ��
%                likelihoodΪ�Գƾ�������[0.3, 0.7; 0.7, 0.3]
likelihood=[0.3, 0.7; 0.7, 0.3];

%    pointOfPass: 2*2���飬pointOfPass(1,:)�����²������ӱ�����ͷ�����ĵ�
%                          pointOfPass(2,:)�����²������ӱ�����β�����ĵ�


pointOfPass=

map=


function Astarmap = BayesianIntentionPred(OtherTrack, pointOfPass, likelihood, map);
