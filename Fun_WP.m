function F_WP = Fun_WP( x0,pos,waypoint ) 
% ���룺pos:��ǰ���д���λ�ã�OS�ǵ�һ��; 
%      waypoint:��ǰOS��ÿһ��TS��WP���Ѹ���CALѡ��
% �������ÿ���Ŀ�괬���ձ�����·���㷽�̺���ȷ������
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
r = Dis/SumDis;
R = r.^2;
XX=x0(1);
YY=x0(2);
f1 = R(2)*(XX-x(1))^2+R(2)*(YY-y(1))^2-R(1)*(XX-x(2))^2-R(1)*(YY-y(2))^2;
f2 = R(2)*(XX-x(3))^2+R(2)*(YY-y(3))^2-R(3)*(XX-x(2))^2-R(3)*(YY-y(2))^2;
F_WP = [f1;f2];

% �ο��������ã�
% ��ţ�ٷ��ķ����麯�� 
% ע�ⷽ�������Ϊ����������
% function F = f2(x0) 
%     x=x0(1); 
%     y=x0(2); 
%     z=x0(3); 
%     f1=3*x-cos(y*z)-1/2; 
%     f2=x^2-81*(y+0.1)^2+sin(z)+1.06; 
%     f3=exp(-x*y)+20*z+(10*pi-3)/3; 
% F=[f1;f2;f3];

end

