function ship_icon( x0,y0,a,b,course_angle,k )%���ĵ�λ��(x0,y0),����a������b�������course_angle��k�Ǵ��ţ����Ų�ͬ����ɫ��ͬ
%% �ڶ��棬����ĺ�����������ı����㷨��ͬ��y������Ϊ0��˳ʱ�����
%SHIP_ICON Summary of this function goes here
%   Detailed explanation goes here
% ���������ڴ���ͼ��ĵ��ԣ�����Ϊ�����Ĵ�С��������λ�ã����Ϊͼ��
% λ�ü�����ת������ͬ����������Ŀ�ĵ�ͼ�꣬ѧϰ���ʦ�ֵĳ���
% ���ڴ���ͼ�꣬�漰���������⣬ԭ�㵽ԭͼ�е��ת�����ʹ������ת��
% ��Բ��������Ϊ[x0,y0],b�볤��,a�����,alpha�����
p_end=0.75;      %�ϰ벿�ֽ���λ��
p_start=2-p_end; %�°벿�ֿ�ʼλ�ã����ϰ벿�ֶԳ�
theta_up=0:pi/20:p_end*pi;         %�ϰ벿�ֵĲ������̵��Ա���������Χ
theta_down=p_start*pi:pi/20:2*pi;  %�ϰ벿�ֵĲ������̵��Ա���������Χ
alpha0=90; %�����ƫ���
% beta=150;    %ʵ�ʺ����
alpha=alpha0+(360-course_angle);
x_up=x0+a*cosd(alpha)*cos(theta_up)-b*sind(alpha)*sin(theta_up);%�ϰ벿�ֵ�x
y_up=y0+a*sind(alpha)*cos(theta_up)+b*cosd(alpha)*sin(theta_up);
x_down=x0+a*cosd(alpha)*cos(theta_down)-b*sind(alpha)*sin(theta_down);%�°벿�ֵ�x
y_down=y0+a*sind(alpha)*cos(theta_down)+b*cosd(alpha)*sin(theta_down);

x_up_end=x0+a*cosd(alpha)*cos(p_end*pi)-b*sind(alpha)*sin(p_end*pi);
y_up_end=y0+a*sind(alpha)*cos(p_end*pi)+b*cosd(alpha)*sin(p_end*pi);
x_down_end=x0+a*cosd(alpha)*cos(p_start*pi)-b*sind(alpha)*sin(p_start*pi);
y_down_end=y0+a*sind(alpha)*cos(p_start*pi)+b*cosd(alpha)*sin(p_start*pi);

% axis([-50 50 -50 50])	
switch(k)
    case 0
         plot(x_up,y_up,'w-','LineWidth',1)      %�ϰ벿��
         hold on
         plot(x_down,y_down,'w-','LineWidth',1)  %�°벿��
         plot([x_up_end,x_down_end],[y_up_end,y_down_end],'w-','LineWidth',1);%��β����
    case 1
         plot(x_up,y_up,'r-','LineWidth',1)      %�ϰ벿��
         hold on
         plot(x_down,y_down,'r-','LineWidth',1)  %�°벿��
         plot([x_up_end,x_down_end],[y_up_end,y_down_end],'r-','LineWidth',1);%��β����
    case 2
         plot(x_up,y_up,'b-','LineWidth',1)      %�ϰ벿��
         hold on
         plot(x_down,y_down,'b-','LineWidth',1)  %�°벿��
         plot([x_up_end,x_down_end],[y_up_end,y_down_end],'b-','LineWidth',1);%��β����
    case 3
         plot(x_up,y_up,'g-','LineWidth',1)      %�ϰ벿��
         hold on
         plot(x_down,y_down,'g-','LineWidth',1)  %�°벿��
         plot([x_up_end,x_down_end],[y_up_end,y_down_end],'g-','LineWidth',1);%��β����
    case 4
         plot(x_up,y_up,'k-','LineWidth',1)      %�ϰ벿��
         hold on
         plot(x_down,y_down,'k-','LineWidth',1)  %�°벿��
         plot([x_up_end,x_down_end],[y_up_end,y_down_end],'k-','LineWidth',1);%��β����
end
% axis ([-50 50],[-50 50])
% axis ([-100 100 -100 100]) %������Ӧ�ô�����������
%Eccentricity of ellipse from axes length��Բ�����ϵ�ƫ�Ķȣ�����Ϊ�볤��Ͱ���ἴb��a


% ��ע����Բ�����еĸ����������壺a�볤�᳤��b����᳤��c�뽹�࣬������e=a/c��ʾ��Բ�ı�ƽ�̶ȣ�pΪ
% e=0.99; %��ʾ��Բ�ı�ƽ�̶�
% p=0.01; %Ӧ����Ϊ���룬��ʾ��Բ�Ĵ�С

end

