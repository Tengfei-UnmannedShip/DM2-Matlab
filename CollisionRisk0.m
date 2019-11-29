function p=CollisionRisk0(ship1,ship2,d_thre) %WTF:确定碰撞风险的函数，ship1为本船，ship2为目标船集合,d_thre风险阈值

% d_thre = 3*1852;%WTF:风险阈值

p=0; %无风险为0

time=2500;
%%
for i=1:length(ship2)
    
    CAP_temp= computeCPA0(ship1,ship2(i),time);
    DCPA_temp=CAP_temp(1,5);
    TCPA_temp=CAP_temp(1,6);
    if DCPA_temp<=d_thre && TCPA_temp>0
        p=1; %有碰撞风险为1
        break;
    end
end
end

