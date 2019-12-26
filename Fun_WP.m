function F_WP = Fun_WP( x0,pos,waypoint ) 
% 输入：pos:当前所有船舶位置，OS是第一个; 
%      waypoint:当前OS与每一个TS的WP，已根据CAL选定
% 用于设置考虑目标船风险比例的路径点方程函数确定方法
% 输入:
% 位置信息pos: 
% 本船位置
OS_pos=pos(1,:);
% 其他目标船位置
TS_pos=pos(2:end,:);
%计算比例
SumDis=0;
for i=1:1:length(TS_pos)
    Dis(i) = norm(OS_pos-TS_pos(i,:),2); %求OS与每一个TS的距离
    SumDis = SumDis+Dis(i);              %求距离的和
    x(i) = waypoint(i,1);
    y(i) = waypoint(i,2);   
end
r = Dis/SumDis;
R = r.^2;
XX=x0(1);
YY=x0(2);
f1 = R(2)*(XX-x(1))^2+R(2)*(YY-y(1))^2-R(1)*(XX-x(2))^2-R(1)*(YY-y(2))^2;
f2 = R(2)*(XX-x(3))^2+R(2)*(YY-y(3))^2-R(3)*(XX-x(2))^2-R(3)*(YY-y(2))^2;
F_WP = [f1;f2];

% 参考基本设置：
% 拟牛顿法的方程组函数 
% 注意方程组输出为列向量函数
% function F = f2(x0) 
%     x=x0(1); 
%     y=x0(2); 
%     z=x0(3); 
%     f1=3*x-cos(y*z)-1/2; 
%     f2=x^2-81*(y+0.1)^2+sin(z)+1.06; 
%     f3=exp(-x*y)+20*z+(10*pi-3)/3; 
% F=[f1;f2;f3];

end

