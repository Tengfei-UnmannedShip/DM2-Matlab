function [ decisioncycle_lable ] = decisioncycle( ShipNum,time,ship )
%DECISIONCYCLE Summary of this function goes here
%   Detailed explanation goes here
%% 判断当前i时刻（time）是在j船（ShipNum）的决策周期中
t=ship(ShipNum).decisioncycle;

% 如果ship j的决策周期是5个时隙，其中第一个时隙用于决策，4个周期执行／等待，那么决策过程是这样的：
% 决策开始   等待结束／决策开始 等待结束／决策开始
%    |- ||- - - - |- ||- - - - |- || - - - - 
%   决策结束／等待 决策结束／等待    
% 所以，当time=ship j的decisioncycle的整数倍时，表示处于周期开始，开始决策
  if  mod(time,t) == 0
      decisioncycle_lable=1;    %当前时刻处于ship j的决策周期中，开始决策
  else
      decisioncycle_lable=0;    %当前时刻不处于ship j的决策周期中，跳过决策
  end
  
end