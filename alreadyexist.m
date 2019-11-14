function bool=alreadyexist(point,list)
    bool=0;
    for ii=1:length(list)
        if point.row == list(ii).row && point.col==list(ii).col
            bool = 1;
        end
    end
end