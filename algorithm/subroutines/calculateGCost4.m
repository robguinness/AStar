% This is a good implementation that takes into account ice data. The
% difference between this one and calculateGCost3 is that this more
% accurate geodetic distance (but is slower to compute), whereas
% calculateGCost3 uses precomputed geometric distances based on Euclidean
% distance

function [gCost, dist] = calculateGCost4(x1,y1,x2,y2,latitude, longitude,inverseSpeed, waypoints, maxSpeed,distThreshold)
    lat1 = latitude(1,y1);
    long1 = longitude(x1,1);
    lat2 = latitude(1,y2);
    long2 = longitude(x2,1);
    %dist = geoddistance(lat1, long1, lat2, long2);
    dist = distance(lat1, long1, lat2, long2)*60*1852;
    
    % parameterize line from (x1,y1) to (x2,y2) by c(t)
    t = linspace(0,1);
    cx_t = (x2-x1)*t+x1-0.5;   %from center of cell, NOT upper-right corner!
    cy_t = (y2-y1)*t+y1-0.5;   %from center of cell, NOT upper-right corner!
    
    dt = dist/10;
    
    %if (p_poly_dist(x2,y2,waypoints(:,1),waypoints(:,2)) < distThreshold)
    [~,distanceFromLine,~] = distance2curve(waypoints,[x2, y2]); %this function is overkill, but at least it works
    if (distanceFromLine < distThreshold)
%         distToIBW = zeros(1,100);
%         for i=1:100
%             % dist to IB waypoints
%             distToIBW(i) = p_poly_dist(cx_t(i),cy_t(i),waypoints(:,1),waypoints(:,2));
%         end
%         pctDistToIBW = sum(distToIBW)/(100 * distThreshold);
%         inverseSpeed = inverseSpeed / (1 - cIB * pctDistToIBW);
        gCost = dist / maxSpeed;
    else
        inverseSpeedInterp = interp2(inverseSpeed,cy_t,cx_t);

        % i don't know why, but sometimes inverseSpeedInterp(100) is NaN
        if isnan(inverseSpeedInterp(10))
           inverseSpeedInterp(10) = inverseSpeed(x2,y2); 
        end

        gCost = dt * sum(inverseSpeedInterp);


        if isnan(gCost)
           gCost = Inf;
        end
    end
end