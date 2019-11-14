function Dir= ShipDirection(frontPositionrow,frontPositioncol,currentPositionrow,currentPositioncol)
    if (currentPositioncol>=frontPositioncol)
        Dir=atan((currentPositionrow-frontPositionrow)/(currentPositioncol-frontPositioncol));
    else
        Dir=pi+atan((currentPositionrow-frontPositionrow)/(currentPositioncol-frontPositioncol));
    end
end