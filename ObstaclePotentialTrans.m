function P = ObstaclePotentialTrans(InMap, TypeofObs)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TypeofObs = 3;
L=1;%1像素点，代表的实际距离需换算，这里代表27m，一倍船长
[m,n]=size(InMap);
P=zeros(m,n);
%%%%%%%%%%%%%%%%%%%%%%%%%%
%%转换前期
%%%%%%%%%%%%%%%%%%%%%%%%%%
SelectedAreaLT.La=30.90;        SelectedAreaRB.La=30.70;
SelectedAreaLT.Lo=121.83;       SelectedAreaRB.Lo=122.03;
[SaLT_la,SaLT_lo]=MKT(SelectedAreaLT.Lo,SelectedAreaLT.La);
[SaRB_la,SaRB_lo]=MKT(SelectedAreaRB.Lo,SelectedAreaRB.La);
scale_y=m/(SaLT_la-SaRB_la);
scale_x=n /(SaRB_lo-SaLT_lo);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
switch TypeofObs
    case 1 %风机
%         load('C:\Users\Administrator\Desktop\论文\Files\最终数据\障碍物分类\Fans.mat')
%         InMap=Fans;
        NumOfObs=62;
        load('FansLocation.mat');
        for ii=1:m
            for jj=1:n
                P(ii,jj)=0;
                for k=1:NumOfObs 
                    [Fy,Fx]=MKT(FansLocation(k).Lo,FansLocation(k).La);
                    a=floor((SaLT_la-Fy)*scale_y);
                    b=floor((Fx-SaLT_lo)*scale_x);
                    distance=sqrt((ii-a)^2+(jj-b)^2);
                    P(ii,jj)=P(ii,jj)+exp(-(distance-L)/(10*L));
                end
            end
        end
    case 2 %航标
        %%%%%%%%%%%%%桥区航标覆盖掩膜%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        SelectedAreaLT.La=30.90;        SelectedAreaRB.La=30.70;
        SelectedAreaLT.Lo=121.83;       SelectedAreaRB.Lo=122.03;
        [SaLT_la,SaLT_lo]=MKT(SelectedAreaLT.Lo,SelectedAreaLT.La);
        [SaRB_la,SaRB_lo]=MKT(SelectedAreaRB.Lo,SelectedAreaRB.La);
        [height,width]=size(InMap);
        scale_y=height/(SaLT_la-SaRB_la);
        scale_x=width /(SaRB_lo-SaLT_lo);
        Mask=ones(m,n);
        [Fy,Fx]=MKT(121.9574333,30.779933);
            a=floor((SaLT_la-Fy)*scale_y);
            b=floor((Fx-SaLT_lo)*scale_x);
            for ii=(b-50):(b+50)
                Mask(a-(ii-b)*5/10-15:a-(ii-b)*5/10+15,ii)=0;
            end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        load('C:\Users\Administrator\Desktop\论文\Files\最终数据\障碍物地理位置坐标\Buoy.mat');
        NumOfObs=length(Buoy(:,1));
        for ii=1:m
            for jj=1:n
                P(ii,jj)=0;
                for k=1:NumOfObs
                    [Fy,Fx]=MKT(Buoy(k,1),Buoy(k,2));
                    a=floor((SaLT_la-Fy)*scale_y);
                    b=floor((Fx-SaLT_lo)*scale_x);
                    if  a>0 && a<m && b>0 && b< n && Mask(a,b)== 1
                        distance=sqrt((ii-a)^2+(jj-b)^2);
                        P(ii,jj)=P(ii,jj)+exp(-(distance-L)/(10*L));
                    end
                end
            end
        end
    case 3 %陆地
        load('C:\Users\Administrator\Desktop\论文\Files\最终数据\障碍物分类\LandWithoutBridge.mat')
        load('C:\Users\Administrator\Desktop\论文\Files\最终数据\障碍物分类\Shallow.mat')
        load('C:\Users\Administrator\Desktop\论文\Files\最终数据\障碍物分类\Shore.mat')
        Shore=1-im2bw(Shore,0.3);
        OBJES=im2bw(Land+Shore+Shallow);
        InMap=OBJES;
        Erge=edge(InMap,'canny');
        for ii=1:m
            for jj=1:n
                if Erge(ii,jj)==1
                    for a=1:m
                        for b =1:n
                            distance=sqrt((ii-a)^2+(jj-b)^2);
                            P(a,b)=P(a,b)+exp(-(distance-L)/(10*L));
                        end
                    end
                end
            end
        end 
end
% end