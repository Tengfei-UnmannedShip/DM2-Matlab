function Jacobi = Jacobi4WP( x0,pos,waypoint )
% JACOBI4WP ���㱾�����п������д�ʱ��WayPoint����Ҫ���Ÿ��Ⱦ���
% ʵ��Ϊ��ϵ����ƫ��������WayPoint�ļ�������һ����Ԫ���η����飬
% ���Jacobi����Ϊ2x2�ľ���
% f1=a_11*x^2+a_12*x+b_11*y^2+b_12*y+c11
% f2=a_21*x^2+a_22*x+b_21*y^2+b_22*y+c21
% Jacobi= [   df1/x   df1/y
%             df2/x   df2/y   ]
% ���������
% Jacobi= [   a_11*x+a_12   b_11*y+b_12
%             a_21*x+a_22   b_21*y+b_22]
% x0=[0,0]; %��ʼ������Ϊ[0,0]��
% ����:
% λ����Ϣpos: 
% ����λ��
OS_pos=pos(1,:);
% ����Ŀ�괬λ��
TS_pos=pos(2:end,:);
%�������
SumDis=0;
for i=1:1:length(TS_pos)
    Dis(i) = norm(OS_pos-TS_pos(i,:),2); %��OS��ÿһ��TS�ľ���
    SumDis = SumDis+Dis(i);              %�����ĺ�
    x(i) = waypoint(i,1);
    y(i) = waypoint(i,2);   
end
r = Dis./SumDis;
R = r.^2;
XX=x0(1);
YY=x0(2);


Jacobi= [ 2*(R(2)-R(1))*XX+2*(R(1)*x(2)-R(2)*x(1))      2*(R(2)-R(1))*YY+2*(R(1)*y(2)-R(2)*y(1))
          2*(R(2)-R(3))*XX+2*(R(3)*x(2)-R(2)*x(3))      2*(R(2)-R(3))*YY+2*(R(3)*y(2)-R(2)*y(3))];

% �ο��������ã�
% x=x0(1);
% y=x0(2);
% Jacobi=[3 z*sin(x*y) y*sin(y*z)
%     2*x -162*(y+0.1) cos(z)
%     -y*exp(-x*y) -x*exp(-x*y) 20];

end

