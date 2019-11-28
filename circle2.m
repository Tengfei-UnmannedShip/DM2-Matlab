function circle2(pos,R,k,Width)

alpha=0:pi/50:2*pi;%½Ç¶È[0,2*pi]
%R=2;%°ë¾¶
x=pos(1)+R*cos(alpha);
y=pos(2)+R*sin(alpha);
if k==1
    plot(x,y,'r-','LineWidth',Width);
elseif k==2
    plot(x,y,'g-','LineWidth',Width);
elseif k==3
    plot(x,y,'b-','LineWidth',Width);
else
    plot(x,y,'k-','LineWidth',Width);
end