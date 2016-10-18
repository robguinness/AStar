% This function uses geographical distance between two ponts on a sphere, utilizing DISTANCE function.
% The difference between this one and calculateGCost3 is that this more
% accurate geodetic distance (but is slower to compute however still faster than using calculateGCost4), whereas
% calculateGCost3 uses precomputed geometric distances based on Euclidean distance.
% Distance is given in meters, however the original output of DISTANC function is in degrees.
% GCost is time elapsed from a starting point to a given point and is
% expressed in seconds
function [gCost, dist] = calculateGCost5(x1,y1,x2,y2,latitude, longitude,inverseSpeed, waypoints, maxSpeed,distThreshold)
    lat1 = latitude(1,y1);
    long1 = longitude(x1,1);
    lat2 = latitude(1,y2);
    long2 = longitude(x2,1);
    dist = distance(lat1, long1, lat2, long2)*60*1852;
    numberOfIntervals=10;
    
    % parameterize line from (x1,y1) to (x2,y2) by c(t)
    t = linspace(0,1);
    cx_t = (x2-x1)*t+x1-0.5;   %from center of cell, NOT upper-right corner!
    cy_t = (y2-y1)*t+y1-0.5;   %from center of cell, NOT upper-right corner!
    
    dt = dist/numberOfIntervals;
    
    [distanceFromLine] = distance2curve2(waypoints,x2,y2);
    %[~,distanceFromLine,~] = distance2curve(waypoints,[x2, y2]); %this function is overkill, but at least it works
    if (distanceFromLine < distThreshold)
        gCost = dist / maxSpeed; %distance in [m], speed in [m/s]
    else
        inverseSpeedInterp = interp2(inverseSpeed,cy_t,cx_t);

        % i don't know why, but sometimes inverseSpeedInterp(100) is NaN
        if isnan(inverseSpeedInterp(numberOfIntervals))
           inverseSpeedInterp(numberOfIntervals) = inverseSpeed(x2,y2); 
        end

        gCost = dt * sum(inverseSpeedInterp);


        if isnan(gCost)
           gCost = Inf;
        end
    end
end