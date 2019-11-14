function [X,Y] = MKT(L,B)
%% KT方程
% 在ObstaclePotentialTrans函数中使用
Pi = 3.1415926;
a = 6378137;
b = 6356752.3142;
B = B * Pi / 180 ;
L = L * Pi / 180 ;
B0 = 0;
L0 = 0;
e1 = sqrt(1- (b / a) * (b / a));
e2 = sqrt((a / b) * (a / b) -1);
Cos =cos(B0);
N = (a * a / b) / sqrt(1+ e2 * e2 * Cos * Cos);
K = N * Cos;
Tan = tan(Pi /4+ B /2);
Temp1 = 1- e1 * sin(B);
Temp2 = 1+ e1 * sin(B);
Temp3 = power(Temp1/Temp2, e1 /2);
Temp4 = Tan * Temp3;
X = K * log(Temp4);
Y = K * (L-L0);