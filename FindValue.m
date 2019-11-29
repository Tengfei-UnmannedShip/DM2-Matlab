function p = FindValue( A,n )
%% 函数，用于找到矩阵A中第n大的数p
A0=A;
i=0;
while i~=n
    pp=max(max(A0)); %找到当前最大的数
    p=pp;
    A0(A0==pp)=0;
    i=i+1;
end

end

