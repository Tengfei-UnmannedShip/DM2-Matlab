function ship = SearchNeighbor(ship1,ship2,d)%dΪship1�Ĵ�������ⷶΧ

ship=[];
for i=1:length(ship2)%length(ship2)Ϊship2�������м��Ҵ����м��Ҵ����Ǽ�
    if ~(ship1.pos(1)==ship2(i).pos(1)&& ship1.pos(2)==ship2(i).pos(2)) ...%x1=x2;y1=y2������������һ��λ�ã��Ǿ���һ�Ҵ�
            && norm(ship1.pos-ship2(i).pos,2)<=d   %��������С��d������ʱdΪship1�Ĵ�������ⷶΧ
        ship = [ship;ship2(i)];%ÿһ���жϼȲ��Ǳ���Ҳ�ڽϲ෶Χ������������飨ship�����һ�����ݡ�
    end
end
%length(a)Ϊ����a�ĳ��ȣ���Ԫ��������Ҳ��ʾ����a�ĳ��ȣ��������������еĴ��ߡ�
%norm(a,p)Ϊ����a��ŷʽ����������x^p+y^p��^��1��p��,��aΪ����ʱ�����������塣