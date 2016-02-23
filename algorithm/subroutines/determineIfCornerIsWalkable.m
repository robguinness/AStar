function corner = determineIfCornerIsWalkable(a, b, parentXval, parentYval, whichList, corner)
    unwalkable = 4;
    if (a == parentXval-1)
        if (b == parentYval-1)
            if ( (whichList(parentXval-1,parentYval) == unwalkable) || ...
                    (whichList(parentXval,parentYval-1) == unwalkable) )
                corner = unwalkable;
            end
        elseif (b == parentYval+1)
            if ( (whichList(parentXval,parentYval+1) == unwalkable) || ...
                    (whichList(parentXval-1,parentYval) == unwalkable) )
                corner = unwalkable;
            end
        end
    elseif (a == parentXval+1)
        if (b == parentYval-1)
            if ( (whichList(parentXval,parentYval-1) == unwalkable) || ...
                    (whichList(parentXval+1, parentYval) == unwalkable) )
                corner = unwalkable;
            end
        elseif (b == parentYval+1)
            if ( (whichList(parentXval+1,parentYval) == unwalkable) || ...
                    (whichList(parentXval,parentYval+1) == unwalkable) )
                corner = unwalkable;
            end
        end
    end
end    