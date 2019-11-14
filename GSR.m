function TS_GSR = GSR( OS, TS )
% �����������ж�GSR��Give-way/Stand-on Relation��
% ������OS��TS�Ľṹ��
% �����һ�����飺GSR=[TS1,TS2,TS3]������ǰ��OS���У�TS1��TS2��TS3�Ĺ�ϵ��
% 1=give-wayΪOS��Ҫ��TSi��·��TSi������OS���Ҳ�
% 0=stand-onΪOS��Ҫ��TSiֱ����TSi������OS�����

%coordinate transformation
TS_GSR = [];
for j=1:length(TS)
    %WTF:��Ŀ�괬������ת�����Ա���Ϊ����ԭ�������ϵ��
    TS(j).pos = TS(j).pos-OS.pos; %ƽ��
    %WTF:��ת�����Ŀ�괬������ͨ����ת����һ��ת��Ϊ��������ָ��y�����������ϵ��
    TS(j).pos = coord_conv(TS(j).pos(1),TS(j).pos(2),OS.Course);%��ת
    %WTF:��Ŀ�괬�ĺ���ת��Ϊ�Ա�������Ϊy�����������ϵ��
    TS(j).Course = TS(j).Course-OS.Course;
    if TS(j).Course<0
        TS(j).Course=TS(j).Course+360;
    end
end

%WTF:Ŀ�괬��λ�úͺ���ת����ɺ󣬽�������λ������ԭ�㣬�������ĺ����Ϊy������
OS.pos = [0 0];
OS.Course = 0;

%% find all the ships that the own ship should give way
% �����ʦ������û�аѶ�����׷Խ�������ڣ����Ҷ�����·��ֱ��֮�µ�ϸ�»������ҵ������в������ã�������޸�֮�����£�
% ����Ϊ��·��(1)����Ŀ�괬λ�ڱ���ǰ��5�㡫67.5��(A)&67.5�㡫112.5��(B)��λ�ã���ʱ��������·��Ŀ�괬ֱ�����ӱ����Ĵ�ͷ������TSi=1
% ����Ϊ������(1)����Ŀ�괬λ�ڱ���ǰ��355�㡫5�㣨F����λ�ã���ʱ�����������򱾴�׷Խ��Ŀ�괬��Ȼ��Ҫ�ӱ����Ĵ�ͷ�����ˣ�TSi=1
% ����Ϊֱ����(0)����Ŀ�괬λ�ڱ���ǰ��247.5�㡫355�㣨E����λ�ã���ʱ������ֱ����Ŀ�괬��·���ӱ����Ĵ�β������TSi=0
% ����Ϊ��׷Խ(0)����Ŀ�괬λ�ڱ���ǰ��112.5�㡫247.5�㣨C&D����λ�ã���ʱ��Ŀ�괬��ͼ׷Խ���������ӱ����Ĵ�β������TSi=0

for i=1:length(TS)
    if CollisionRisk0(OS,TS(i))
        % �ж�TSi��λ��theta
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

