function p=CollisionRisk0(ship1,ship2) %WTF:ȷ����ײ���յĺ�����ship1Ϊ������ship2ΪĿ�괬����

d_thre = 3*1852;%WTF:������ֵ

p=0; %�޷���Ϊ0

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
        p=1; %����ײ����Ϊ1
        break;
    end
end
end

