function [x, y] = calcXY(latitude,longitude,pointLat,pointLong)
    [m, n] = size(latitude);
    y = 0;
    x = 0;
    yCheck1 = NaN;
    xCheck1 = NaN;
    for i = 1:m
        if pointLong == longitude(i,1)
            x = i;
            break;
        elseif pointLong < longitude(1,1)
            disp('longitude is out of range');
            return
        elseif pointLong < longitude(i,1)
            x = i;
            xCheck1 = longitude(i,1);
            xCheck2 = longitude(i-1,1);
            break;
        end
    end
    if x == 0
        disp('longitude is out of range');
        return
    end
    
    if ~isnan(xCheck1)
        avg = (xCheck1+xCheck2)/2;
        if (pointLong < avg)
            x = i-1;
        end    
    end
    
    for i = 1:n
        if pointLat == latitude(1,i)
            y = i;
            break;
        elseif pointLat < latitude(1,1)
            disp('latitude is out of range');
            return
        elseif pointLat < latitude(1,i)
            y = i;
            yCheck1 = latitude(1,i);
            yCheck2 = latitude(1,i-1);
            break;
        end
    end
    if y == 0
        disp('latitude is out of range');
        return
    end
    
    if ~isnan(yCheck1)
        avg = (yCheck1+yCheck2)/2;
        if (pointLat < avg)
            y = i-1;
        end    
    end
    
    
end