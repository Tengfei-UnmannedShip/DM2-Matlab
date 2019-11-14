function goal_point = Goal_point1(x0,y0,ang,range)
%% 单位米m，根据当前的位置和航向，确定目标点，该目标点在矩形地图的边界上
% 边界范围range=[X,Y],如range=[5,5]，则就是一个 长-5nm～5nm，宽-5nm～5nm的范围
%本来思路是找出在距离中心相同的圆上的点作为终点，这样思路比较简单，但是需要解方程组(1个二元一次，1个二元二次)，非常影响运行时间
%直接调用Matlab函数解方程实在是太慢了
%现在是找出原来的那条线在一个方形范围内的交点，实质上是代数运算，简化了计算。
%麻烦的不行，肯定会有简化的好办法，懒得弄了，能用就行

X = range(1);
Y = range(2);
a1 = atand(abs(X-x0)/abs(Y-y0));
b1 = -(360-a1);
a2 = 90+acotd(abs(X-x0)/abs(-Y-y0));
b2 = -(360-a2);
a3 = 180+atand(abs(-X-x0)/abs(-Y-y0));
b3 = -(360-a3);
b4 = -atand(abs(-X-x0)/abs(Y-y0));
a4 = 360+b4;
k = tand(90-ang);
if ang == 0
    x = x0;
    y = Y;
elseif ang ==90 || ang == -270
    x = X; 
    y = y0; 
elseif ang ==180 || ang == -180
    x = x0;
    y = -Y;
elseif ang ==270 || ang == -90
    x = -X; 
    y = y0;
elseif ang >= a1 && ang < a2 || ang >= b1  ...
        && ang < b2 && ang ~= 90 && ang ~= -270
    x = X;
    y = y0+k*(x-x0);
elseif ang >= a2 && ang < a3  || ang >= b2  ...
        && ang < b3 && ang ~= 180 && ang ~= -180
    y = -Y;
    x = (y-y0)/k+x0;
elseif ang >= a3 && ang < a4 || ang >= b3  ...
        && ang < b4 && ang ~= -90 && ang ~= 270
    x = -X;
    y = y0+k*(x-x0); 
else
    y = Y;
    x = (y-y0)/k+x0;
end
%最后求得得当前的目标点
goal_point=[x,y];

end

