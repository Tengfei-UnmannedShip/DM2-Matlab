function WP0_real = Find_WP( ship,i,CAL )
%计算TSi眼中考虑CAL的备选waypoint
% 输入：ship，本船船号i，当前CAL
OSwp.pos=ship(i).pos(end,:);
OSwp.Course=ship(i).Course(end,:);
OSwp.speed=ship(i).speed;
OSwp.length=ship(i).length;
wp_pos=OSwp.pos;
TSwp(1).pos=OS.pos(end,:);
TSwp(1).Course=OS.Course(end,:);
TSwp(1).speed=OS.speed;
TSwp(1).length=OS.length;
wp_pos=[wp_pos;TSwp(1).pos];
j=1;
for k=1:1:4
    if k~=i
        TSwp(j).pos=ship(k).pos(end,:);
        TSwp(j).Course=ship(k).Course(end,:);
        TSwp(j).speed=ship(k).speed;
        TSwp(j).length=ship(k).length;
        wp_pos=[wp_pos;TSwp(j).pos];
        j=j+1;
    else
        continue
    end
end
%             计算没有经过CAL过滤的wp
wp0=WayPoint0( OSwp,TSwp,2500 );
WP0_real=[];
k=1;
for j=1:1:4
    if j~=2
        if CAL(i,j)==1
            WP0_real = [WP0_real; wp0(k,1),wp0(k,2)];
        else
            WP0_real = [WP0_real; wp0(k,3),wp0(k,4)];
        end
        k=k+1;
    else
        continue
    end
end

end

