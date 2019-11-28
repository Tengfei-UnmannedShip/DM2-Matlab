function ha  = MarginEdit(Nh, Nw, gap, marg_h, marg_w,TickLabel)
%% 自己手动设置绘图的边距和边界尺寸
% 使用subplot(row, col, i)创建的子图，matlab会隐式地调整它们的间距以及它们和整个figure的边距，
% 以达到它所认为的美观或者合理的设置，然而如果我们想根据需求设置合理的间距以及边距，该怎么定制呢？
% 这里提供一个函数（实在没必要把时间花费在这种繁琐的格式上）：
%% 参数的含义%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Nh, Nw用法同subplot(row, col)表示行数和列数
% gap（如[0.01, 0.1]）表示子图之间垂直方向和水平方向的间隔，
% marg_h表示的是全部子图到figure上下边界的距离，
% marg_w则表示的是全部子图到figure左右边界的距离。
%TickLabel=1,有x,y轴刻度
%  ==========================上边框===================================
% |               |上边距：marg_h             |上边距：marg_h          |
% |        -----------------         ----------------------         |
% |       |                 |       |                      |        |
% |       |    子图          |       |     子图             |        |
% |-------|                 |-------|                      |--------|
% | 左边距 |                 |水平间距 |                      |  右边距 |
% |marg_w |                 |gap_w  |                      | marg_w |
% |       |                 |       |                      |        |
% |        -----------------         ----------------------         |
% |               |垂直间距：gap_h                                    |
% |               |                                                 |
% |        -----------------         ----------------------         |
% |       |                 |       |                      |        |
% |       |                 |       |                      |        |
% |       |                 |       |                      |        |
% |       |                 |       |                      |        |
% |       |                 |       |                      |        |
% |       |                 |       |                      |        |
% |        -----------------         ----------------------         |
% |               |下边距：marg_h             |下边距：marg_h          |
%  ==========================下边框===================================

%% 程序主体%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% tight_subplot creates "subplot" axes with adjustable gaps and margins
%
% ha = tight_subplot(Nh, Nw, gap, marg_h, marg_w)
%
%   in:  Nh      number of axes in hight (vertical direction)
%        Nw      number of axes in width (horizontaldirection)
%        gap     gaps between the axes in normalized units (0...1)
%                   or [gap_h gap_w] for different gaps in height and width
%        marg_h  margins in height in normalized units (0...1)
%                   or [lower upper] for different lower and upper margins
%        marg_w  margins in width in normalized units (0...1)
%                   or [left right] for different left and right margins
%
%  out:  ha     array of handles of the axes objects
%                   starting from upper left corner, going row-wise as in
%                   going row-wise as in
%
%  Example: ha = tight_subplot(3,2,[.01 .03],[.1 .01],[.01 .01])
%           for ii = 1:6; axes(ha(ii)); plot(randn(10,ii)); end
%           set(ha(1:4),'XTickLabel',''); set(ha,'YTickLabel','')

% Pekka Kumpulainen 20.6.2010   @tut.fi
% Tampere University of Technology / Automation Science and Engineering


if nargin<3; gap = .02; end
if nargin<4 || isempty(marg_h); marg_h = .05; end
if nargin<5; marg_w = .05; end

if numel(gap)==1;
    gap = [gap gap];
end
if numel(marg_w)==1;
    marg_w = [marg_w marg_w];
end
if numel(marg_h)==1;
    marg_h = [marg_h marg_h];
end

axh = (1-sum(marg_h)-(Nh-1)*gap(1))/Nh;
axw = (1-sum(marg_w)-(Nw-1)*gap(2))/Nw;

py = 1-marg_h(2)-axh;

ha = zeros(Nh*Nw,1);
ii = 0;
for ih = 1:Nh
    px = marg_w(1);
    
    for ix = 1:Nw
        ii = ii+1;
        if TickLabel==1
            ha(ii) = axes('Units','normalized', ...
                'Position',[px py axw axh]);
        elseif TickLabel==0
            ha(ii) = axes('Units','normalized', ...
                'Position',[px py axw axh], ...
                'XTickLabel','', ...
                'YTickLabel','');
        end
        px = px+axw+gap(2);
    end
    py = py-axh-gap(1);
end

% 举例：
% ha = MarginEdit(3,2,[.01 .03],[.1 .01],[.01 .01])
% for ii = 1:6;
%     axes(ha(ii));
%     plot(randn(10,ii));
% end
% set(ha(1:4),'XTickLabel','');
% set(ha,'YTickLabel','')

end

