function p=CollisionRisk0(ship1,ship2,d_thre) %WTF:ȷ����ײ���յĺ�����ship1Ϊ������ship2ΪĿ�괬����,d_thre������ֵ

% d_thre = 3*1852;%WTF:������ֵ

p=0; %�޷���Ϊ0

time=2500;
%%
for i=1:length(ship2)
    
    CAP_temp= computeCPA0(ship1,ship2(i),time);
    DCPA_temp=CAP_temp(1,5);
    TCPA_temp=CAP_temp(1,6);
    if DCPA_temp<=d_thre && TCPA_temp>0
        p=1; %����ײ����Ϊ1
        break;
    end
end
end

