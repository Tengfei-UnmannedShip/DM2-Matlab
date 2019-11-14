%% 来自Ning Wang的基本SCR的船舶领域模型，公式较简单
clear
clc
close all
r0 = 0.5;
L = 160/1852;
k=2;
Vos = 15;
k_AD = 10^(0.3591*log10(Vos)+0.0952);
k_DT = 10^(0.5441*log10(Vos)-0.0795);

Rfore = (1+1.34*sqrt(k_AD^2+0.25*k_DT^2))*L;
Raft = (1+0.67*sqrt(k_AD^2+0.25*k_DT^2))*L;
Rstarb = (0.2+k_DT)*L;
Rport = (0.2+0.75*k_DT)*L;
R = [Rfore Raft Rstarb Rport];
Sigma = R/((log(1/r0))^(1/k));
a=Sigma(1);
% x= -1.5:0.1:1.5
% y= -1.5:0.1:1.5
% CRx=exp(-(2*x./((1+sign(x))*Sigma(1)+(1-sign(x))*Sigma(2))).^k);
% plot(x,CRx)
[x,y]=meshgrid(-1.5:0.01:1.5, -1.5:0.01:1.5);  
[m,n]=size(x);
SCR1= exp(-(2*x./((1+sign(x))*Sigma(1)+(1-sign(x))*Sigma(2))).^k-(2*y./((1+sign(y))*Sigma(4)+(1-sign(y))*Sigma(3))).^k);
SCR2= exp(-(2*x./((1+sign(x))*Sigma(2)+(1-sign(x))*Sigma(1))).^k-(2*y./((1+sign(y))*Sigma(3)+(1-sign(y))*Sigma(4))).^k);
SCR3= exp(-(2*y./((1+sign(y))*Sigma(1)+(1-sign(y))*Sigma(2))).^k-(2*x./((1+sign(x))*Sigma(3)+(1-sign(x))*Sigma(4))).^k);
SCR4= exp(-(2*y./((1-sign(y))*Sigma(2)+(1+sign(y))*Sigma(1))).^k-(2*x./((1+sign(x))*Sigma(3)+(1-sign(x))*Sigma(4))).^k);
% z=SCR;
figure
surf(x,y,SCR1,'edgecolor','none','facecolor','interp') %不带填充颜色的等高线图
hold on
surfc(x,y,-1+0*SCR1,SCR1,'edgecolor','none','facecolor','interp') %不带填充颜色的等高线图
axis equal
xlabel('\it n miles', 'Fontname', 'Times New Roman');
ylabel('\it n miles', 'Fontname', 'Times New Roman');

figure
surf(x,y,SCR3,'edgecolor','none','facecolor','interp') %不带填充颜色的等高线图
hold on
surfc(x,y,-1+0*SCR3,SCR3,'edgecolor','none','facecolor','interp') %不带填充颜色的等高线图
axis equal
xlabel('\it n miles', 'Fontname', 'Times New Roman');
ylabel('\it n miles', 'Fontname', 'Times New Roman');

% figure
% % subplot(2,2,1)
% % contourf(x,y,SCR1) %不带填充颜色的等高线图
% % subplot(2,2,2)
% % contourf(x,y,SCR2) %不带填充颜色的等高线图
% subplot(2,1,1)
% contourf(x,y,SCR3) %不带填充颜色的等高线图
% subplot(2,1,2)
% contourf(x,y,SCR4) %不带填充颜色的等高线图


% surfc(x,y,z,'edgecolor','none','facecolor','interp')
% axis([-1.5 1.5 -1.5 1.5 -1 1]);
% figure
% meshc(x,y,z)
% axis([-1.5 1.5 -1.5 1.5 0 4]);
% light('position',[50,-10,5]),lighting flat
% material([0.9,0.9,0.6,15,0.4])
% hold on
% surf(x,y,-1+0*z,z,'edgecolor','none','facecolor','interp') %不带填充颜色的等高线图

% 
% figure
% 
% mesh(x,y,z)