function  APFmap= DrawAPF( OS, TS, map,Re,draw )
%�˹��Ƴ���ͼ���ƺ��������Ծɵ��˹��Ƴ�����

%% ��ͼ����
% map=[x,y],��ʾ�ԳƵ�4���߽磬��λ����
x1=-map(1)*1852;
x2=map(1)*1852;
y1=-map(2)*1852;
y2=map(2)*1852;
%Re,Resolution����ͼ�ķֱ���
[APF_X,APF_Y]=meshgrid(x1:Re:x2,y1:Re:y2);
[m,n]=size(APF_X);
%% ==================================================
% Ŀ�괬����������
% ����ģ�Ͳο��Ŷռ��㷽��
% ==================================================
%������ʵ�ʴ�С֮�󣬿��ܻ���Ҫ�Ƚϴ�һЩ�Ĳ�������Ȼ����û����ʾ
Boat_Speed_Factor=1;           %�ٶȷ����Ƴ�˥�����ӣ�ȡֵԽ�����ٶȷ�����Ӱ��Խ��
BoatRiskFieldPeakValue=100;    %�������ֵ���Ը�����Ҫ��������
Boat_eta=1;
Boat_alfa=0.1;
BoatCut=0;
RiskFieldValue=zeros(m,n);%wtf--Z=RiskFieldValue����ÿһ����Ƴ�ֵ���˴��Ƚ������Ϊһ����X��ͬ��С��0����
APFmap=zeros(m,n);

for i=1:1:length(TS)
    %���������ʼ��
    Boat_x(i)=TS(i).pos(end,1);                   %��i����x����
    Boat_y(i)=TS(i).pos(end,2);                   %��i����y����
    Boat_theta(i)=-TS(i).Course(end,:)/180*pi;       %��i��������Ƕ�
    Boat_Speed(i)=TS(i).speed(end,:);                %��i�����ٶȴ�С
    Boat_length(i)=TS(i).length;              %��i������
    Boat_width(i)=TS(i).width;                %��i������
    %�ֲ�����ϵת�����Ѽ����Ƴ������꣨X,Y���任����������ϵ�µ㣨BoatX{i},BoatY{i}��
    BoatX{i} = (APF_X-Boat_x(i))*cos(Boat_theta(i))+(APF_Y-Boat_y(i))*sin(Boat_theta(i));
    BoatY{i} = (APF_Y-Boat_y(i))*cos(Boat_theta(i))-(APF_X-Boat_x(i))*sin(Boat_theta(i));
    
    BoatSpeedFactor{i}=Boat_Speed_Factor*Boat_Speed(i);
    
    %����ռ��е㵽��i�������ؾ���Dis{i}
    Dis{i}=zeros(m,n);
    Dis{i}=sqrt(BoatX{i}.^2+BoatY{i}.^2).*(BoatY{i}<=0)+sqrt((BoatY{i}/(Boat_Speed(i)+1)).^2+BoatX{i}.^2).*(BoatY{i}>0);
    
    %�����i�������ճ�
    BoatRiskField{i}=BoatRiskFieldPeakValue.*(exp(-Boat_eta*Boat_alfa*Dis{i})./(Boat_alfa*Dis{i}));
    
    if  BoatRiskField{i}>BoatRiskFieldPeakValue
        BoatRiskField{i}=BoatRiskFieldPeakValue;
    end
    BoatRiskField{i}(BoatRiskField{i}>BoatRiskFieldPeakValue)=BoatRiskFieldPeakValue;
    
    if BoatCut==1
        BoatRiskField{i}=BoatRiskField{i}.*(BoatX{i}>=0)+0.*(BoatX{i}<0);
    end
    
    %����ÿ�����ڲ�ͬ���µĳ�֮���ݲ��ü򵥼Ӻ�
    RiskFieldValue=RiskFieldValue+BoatRiskField{i};
end
APFmap=RiskFieldValue;
% APFValue=RiskFieldValue;
% newfield=RiskFieldValue/BoatRiskFieldPeakValue;
% APF.X=APF_X;
% APF.Y=APF_Y;
% APF.map0=APFValue;
% APF.map1=newfield;

%% ��ͼ�������ÿһ�ζ���ͼ�Ļ�����̫�ֻ࣬��һЩʱ�̻�ͼ�����ƺ���Ϊdraw
if draw==1  %ȷ�ϻ�ͼ
    figure;
    mesh(APF_X,APF_Y,APFmap);
    hold on;
    plot(goal(1,1),goal(1,2),'ro','MarkerFaceColor','r');
    axis equal;
    axis off;
    figure
    contourf(APF_X,APF_Y,APFmap,'LevelStep',30);  %�������ɫ�ĵȸ���ͼ
    hold on;
    plot(goal(1,1),goal(1,2),'ro','MarkerFaceColor','r');
    hold on;
    ship_icon(OS.pos(1),OS.pos(2),OS.length,OS.width, OS.Course,0 ); %����ʹ��ship_icon��������ʾ�����Ŀ��ܻ��е�С
    axis equal;
    axis off;
end
end

