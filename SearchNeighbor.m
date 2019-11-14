function ship = SearchNeighbor(ship1,ship2,d)%d为ship1的传感器检测范围

ship=[];
for i=1:length(ship2)%length(ship2)为ship2集合中有几艘船，有几艘船就是几
    if ~(ship1.pos(1)==ship2(i).pos(1)&& ship1.pos(2)==ship2(i).pos(2)) ...%x1=x2;y1=y2即两个船处于一个位置，那就是一艘船
            && norm(ship1.pos-ship2(i).pos,2)<=d   %两船距离小于d，调用时d为ship1的传感器检测范围
        ship = [ship;ship2(i)];%每一次判断既不是本船也在较侧范围内则往结果数组（ship）里存一个数据。
    end
end
%length(a)为向量a的长度，即元素数量，也表示矩阵a的长度，即行数和列数中的大者。
%norm(a,p)为向量a的欧式范数，即（x^p+y^p）^（1／p）,当a为矩阵时，有其他定义。