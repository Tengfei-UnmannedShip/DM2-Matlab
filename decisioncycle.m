function [ decisioncycle_lable ] = decisioncycle( ShipNum,time,ship )
%DECISIONCYCLE Summary of this function goes here
%   Detailed explanation goes here
%% �жϵ�ǰiʱ�̣�time������j����ShipNum���ľ���������
t=ship(ShipNum).decisioncycle;

% ���ship j�ľ���������5��ʱ϶�����е�һ��ʱ϶���ھ��ߣ�4������ִ�У��ȴ�����ô���߹����������ģ�
% ���߿�ʼ   �ȴ����������߿�ʼ �ȴ����������߿�ʼ
%    |- ||- - - - |- ||- - - - |- || - - - - 
%   ���߽������ȴ� ���߽������ȴ�    
% ���ԣ���time=ship j��decisioncycle��������ʱ����ʾ�������ڿ�ʼ����ʼ����
  if  mod(time,t) == 0
      decisioncycle_lable=1;    %��ǰʱ�̴���ship j�ľ��������У���ʼ����
  else
      decisioncycle_lable=0;    %��ǰʱ�̲�����ship j�ľ��������У���������
  end
  
end