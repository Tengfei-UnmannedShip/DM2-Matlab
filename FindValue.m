function p = FindValue( A,n )
%% �����������ҵ�����A�е�n�����p
A0=A;
i=0;
while i~=n
    pp=max(max(A0)); %�ҵ���ǰ������
    p=pp;
    A0(A0==pp)=0;
    i=i+1;
end

end

