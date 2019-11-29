function p=CollisionRisk0(ship1,ship2) %WTF:确定碰撞风险的函数，ship1为本船，ship2为目标船集合

d_thre = 3*1852;%WTF:风险阈值

p=0; %无风险为0

time=2500;
%%
for i=1:length(ship2)
    if length(ship1.pos)>1
        ship1.pos=ship1.pos(end,:);
    end
    if length(ship2(i).pos)>1
        ship2(i).pos=ship2(i).pos(end,:);
    end
    if length(ship1.Course)>1
        ship1.Course=ship1.Course(end,:);
    end
    if length(ship2(i).Course)>1
        ship2(i).Course=ship2(i).Course(end,:);
    end
    
    d1= computeCPA0(ship1,ship2(i),time);
    
    if d1<=d_thre
        p=1; %有碰撞风险为1
        break;
    end
end
end

