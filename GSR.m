function TS_GSR = GSR( OS, TS )
% 本函数用于判断GSR（Give-way/Stand-on Relation）
% 输入是OS和TS的结构体
% 输出是一个数组：GSR=[TS1,TS2,TS3]，即当前在OS眼中，TS1，TS2，TS3的关系，
% 1=give-way为OS需要对TSi让路，TSi出现在OS的右侧
% 0=stand-on为OS需要对TSi直航，TSi出现在OS的左侧

%coordinate transformation
TS_GSR = [];
for j=1:length(TS)
    %WTF:将目标船的坐标转换成以本船为坐标原点的坐标系中
    TS(j).pos = TS(j).pos-OS.pos; %平移
    %WTF:将转化后的目标船的坐标通过旋转，进一步转化为本船航向指向y轴正向的坐标系中
    TS(j).pos = coord_conv(TS(j).pos(1),TS(j).pos(2),OS.Course);%旋转
    %WTF:将目标船的航向转化为以本船航向为y轴正向的坐标系中
    TS(j).Course = TS(j).Course-OS.Course;
    if TS(j).Course<0
        TS(j).Course=TS(j).Course+360;
    end
end

%WTF:目标船的位置和航向都转化完成后，将本船的位置置于原点，将本船的航向归为y轴正向
OS.pos = [0 0];
OS.Course = 0;

%% find all the ships that the own ship should give way
% 金奋老师的论文没有把对遇和追越考虑在内，并且对于让路和直航之下的细致划分在我的论文中并不适用，借鉴并修改之后如下：
% 本船为让路船(1)：即目标船位于本船前方5°～67.5°(A)&67.5°～112.5°(B)的位置，这时，本船让路，目标船直航即从本船的船头经过，TSi=1
% 本船为对遇船(1)：即目标船位于本船前方355°～5°（F）的位置，这时，两船对遇或本船追越，目标船依然是要从本船的船头经过了，TSi=1
% 本船为直航船(0)：即目标船位于本船前方247.5°～355°（E）的位置，这时，本船直航，目标船让路即从本船的船尾经过，TSi=0
% 本船为被追越(0)：即目标船位于本船前方112.5°～247.5°（C&D）的位置，这时，目标船试图追越本船，即从本船的船尾经过，TSi=0

for i=1:length(TS)
    if CollisionRisk0(OS,TS(i))
        % 判断TSi的位置theta
        a=[0 1 0];
        b=[TS(i).pos 0];
        c=cross(a,b);
        theta=acosd(dot(a(1:2),b(1:2))/(norm(a(1:2))*norm(b(1:2))));
        if c>0
            theta= theta;
        elseif c<0
            theta= 360-theta;
        end
        
        if theta<112.5 || theta>355
            TS_GSR(i)=1; 
        elseif theta>112.5 && theta<355
            TS_GSR(i)=0;
        end
    end
end

end

