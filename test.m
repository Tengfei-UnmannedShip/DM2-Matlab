% APF绘图%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
APFvalue=APF(1).map;
for i=2:1:length(APF)
    APFvalue=APFvalue+APF(i).map;
end

x1=-APFmapSize(1)*1852;
x2=APFmapSize(1)*1852;
y1=-APFmapSize(2)*1852;
y2=APFmapSize(2)*1852;

[APF_X,APF_Y]=meshgrid(x1:100:x2,y1:100:y2);

figure
contourf(APF_X,APF_Y,APFvalue,'LevelStep',30);  %带填充颜色的等高线图
colorpan=ColorPanSet(6);
colormap(colorpan);%定义色盘
hold on
plot(10,10,'ro','MarkerFaceColor','r');
plot(0,0,'bo','MarkerFaceColor','b');
plot(-9*1852,9*1852,'bo','MarkerFaceColor','b');
axis equal;
axis off;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%