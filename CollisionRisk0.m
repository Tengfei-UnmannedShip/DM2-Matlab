function p=CollisionRisk0(ship1,ship2) %WTF:ȷ����ײ���յĺ�����ship1Ϊ������ship2ΪĿ�괬����

d_thre = 3*1852;%WTF:������ֵ

p=0;

time=2500;
%%
for i=1:length(ship2)
    pos11=ship1.pos;
    pos21=ship2(i).pos;
    d1= computeCPA(ship1,ship2(i),time);

    if d1<=d_thre
        p=1;
        break;
    end
end
end
    
    