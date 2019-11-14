function ship_icon( x0,y0,a,b,course_angle,k )%中心点位置(x0,y0),长轴a，短轴b，航向角course_angle，k是船号，船号不同，颜色不同
%% 第二版，输入的航向角与正常的避碰算法相同，y正方向为0，顺时针计数
%SHIP_ICON Summary of this function goes here
%   Detailed explanation goes here
% 本程序用于船舶图标的调试，输入为船舶的大小，船艏向，位置；输出为图像。
% 位置及坐标转换程序，同样可以用于目的地图标，学习金奋师兄的程序。
% 对于船舶图标，涉及到两个问题，原点到原图中点的转换，和船艏向的转换
% 椭圆中心坐标为[x0,y0],b半长轴,a半短轴,alpha航向角
p_end=0.75;      %上半部分结束位置
p_start=2-p_end; %下半部分开始位置，与上半部分对称
theta_up=0:pi/20:p_end*pi;         %上半部分的参数方程的自变量参数范围
theta_down=p_start*pi:pi/20:2*pi;  %上半部分的参数方程的自变量参数范围
alpha0=90; %航向角偏差补齐
% beta=150;    %实际航向角
alpha=alpha0+(360-course_angle);
x_up=x0+a*cosd(alpha)*cos(theta_up)-b*sind(alpha)*sin(theta_up);%上半部分的x
y_up=y0+a*sind(alpha)*cos(theta_up)+b*cosd(alpha)*sin(theta_up);
x_down=x0+a*cosd(alpha)*cos(theta_down)-b*sind(alpha)*sin(theta_down);%下半部分的x
y_down=y0+a*sind(alpha)*cos(theta_down)+b*cosd(alpha)*sin(theta_down);

x_up_end=x0+a*cosd(alpha)*cos(p_end*pi)-b*sind(alpha)*sin(p_end*pi);
y_up_end=y0+a*sind(alpha)*cos(p_end*pi)+b*cosd(alpha)*sin(p_end*pi);
x_down_end=x0+a*cosd(alpha)*cos(p_start*pi)-b*sind(alpha)*sin(p_start*pi);
y_down_end=y0+a*sind(alpha)*cos(p_start*pi)+b*cosd(alpha)*sin(p_start*pi);

% axis([-50 50 -50 50])	
switch(k)
    case 0
         plot(x_up,y_up,'w-','LineWidth',1)      %上半部分
         hold on
         plot(x_down,y_down,'w-','LineWidth',1)  %下半部分
         plot([x_up_end,x_down_end],[y_up_end,y_down_end],'w-','LineWidth',1);%船尾连线
    case 1
         plot(x_up,y_up,'r-','LineWidth',1)      %上半部分
         hold on
         plot(x_down,y_down,'r-','LineWidth',1)  %下半部分
         plot([x_up_end,x_down_end],[y_up_end,y_down_end],'r-','LineWidth',1);%船尾连线
    case 2
         plot(x_up,y_up,'b-','LineWidth',1)      %上半部分
         hold on
         plot(x_down,y_down,'b-','LineWidth',1)  %下半部分
         plot([x_up_end,x_down_end],[y_up_end,y_down_end],'b-','LineWidth',1);%船尾连线
    case 3
         plot(x_up,y_up,'g-','LineWidth',1)      %上半部分
         hold on
         plot(x_down,y_down,'g-','LineWidth',1)  %下半部分
         plot([x_up_end,x_down_end],[y_up_end,y_down_end],'g-','LineWidth',1);%船尾连线
    case 4
         plot(x_up,y_up,'k-','LineWidth',1)      %上半部分
         hold on
         plot(x_down,y_down,'k-','LineWidth',1)  %下半部分
         plot([x_up_end,x_down_end],[y_up_end,y_down_end],'k-','LineWidth',1);%船尾连线
end
% axis ([-50 50],[-50 50])
% axis ([-100 100 -100 100]) %坐标轴应该从主函数定义
%Eccentricity of ellipse from axes length椭圆轴向上的偏心度，参数为半长轴和半短轴即b和a


% 备注：椭圆方程中的各参数的意义：a半长轴长，b半短轴长，c半焦距，离心率e=a/c表示椭圆的扁平程度，p为
% e=0.99; %表示椭圆的扁平程度
% p=0.01; %应该作为输入，表示椭圆的大小

end

