function goal_point = Goal_point(x0,y0,ang,range)
%% ��λ����nm�����ݵ�ǰ��λ�úͺ���ȷ��Ŀ��㣬��Ŀ����ھ��ε�ͼ�ı߽���
% �߽緶Χrange=[X,Y],��range=[5,5]�������һ�� ��-5nm��5nm����-5nm��5nm�ķ�Χ
%����˼·���ҳ��ھ���������ͬ��Բ�ϵĵ���Ϊ�յ㣬����˼·�Ƚϼ򵥣�������Ҫ�ⷽ����(1����Ԫһ�Σ�1����Ԫ����)���ǳ�Ӱ������ʱ��
%ֱ�ӵ���Matlab�����ⷽ��ʵ����̫����
%�������ҳ�ԭ������������һ�����η�Χ�ڵĽ��㣬ʵ�����Ǵ������㣬���˼��㡣
%�鷳�Ĳ��У��϶����м򻯵ĺð취������Ū�ˣ����þ���

X = range(1)*1852;
Y = range(2)*1852;
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
%�����õõ�ǰ��Ŀ���
goal_point=[x,y];

end

