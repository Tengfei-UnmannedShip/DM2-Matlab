function [x,n,data]=broyden(x0,tol)
% 一种典型的拟牛顿法broyden计算非线性方程组的解
% 程序参考《MATLAB数值分析与应用_宋叶志》
% 实验 6.5 拟牛顿法（Broyden 方法）（全书第248页）
% 输入
% x0 为迭代初值
% tol 为误差容限，如果缺省默认为10的-10次方
% data 用来存放计算的中间数据便于计算收敛情况分析
if nargin==1
    tol=1e-5;
end

H0=df2(x0);
H0=inv(H0);
x1=x0-H0*f2(x0);

n=1;

%设置初始误差使之可以进入循环
wucha=0.1;
%循环迭代
while (wucha>tol) && (n<20) && (n<500)
    
    wucha=norm(x1-x0);
    dx=x1-x0;
    y=f2(x1)-f2(x0);
    fenzi=dx'*H0*y; %是标量
    H1=H0+(dx-H0*y)*(dx)'*H0/fenzi;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %这一段相当重要，技巧性也比较强，请体会
    temp_x0=x0;
    x0=x1;
    x1=temp_x0-H1*f2(temp_x0); %x1 的更新
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %更新 H 矩阵
    H=H1;
    n=n+1;
    %data 用来存放中间数据
    data(:,n)=x1;
    
end
x=x1;
end

