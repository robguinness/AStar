% This is a good implementation that takes into account ice data. The
% difference between this one and calculateGCost4 is that this uses
% precomputed geometric distances, whereas calculateGCost4 uses more
% accurate geodetic distance (but is slower to compute)

function gCost = calculateGCost3(x1,y1,x2,y2,j,inverseSpeed, ibWaypoints, maxSpeed,distThreshold)
    switch j
        case {1, 2, 3, 4}
                dist = 10.00;
        case {5, 6, 7, 8}
                dist = 14.142;
        case {9, 10, 11, 12, 13, 14, 15, 16}
                dist = 22.361;                                                     
        case {17, 18, 19, 20, 21, 22, 23, 24}
                dist = 31.623;  
        case {25, 26, 27, 28, 29, 30, 31, 32}
                dist = 36.056;  
        case {33, 34, 35, 36, 37, 38, 39, 40}
                dist = 64.031;
        case {41, 42, 43, 44, 45, 46, 47, 48}
                dist = 50.990;
        case {49, 50, 51, 52, 53, 54, 55, 56}
                dist = 100.499;
        otherwise
            disp('Error!')
    end
    
    

    % parameterize line from (x1,y1) to (x2,y2) by c(t)
    t = linspace(0,1);
    %cx_t = (x2-x1)*t+x1-0.5;   %from center of cell, NOT upper-right corner!
    cx_t = (x2-x1)*t+x1;
    %cy_t = (y2-y1)*t+y1-0.5;   %from center of cell, NOT upper-right corner!
    cy_t = (y2-y1)*t+y1;
    
    dt = dist/100;
    
    %if (p_poly_dist(x2,y2,ibWaypoints(:,1),ibWaypoints(:,2)) < distThreshold)
    [~,distanceFromLine,~] = distance2curve(ibWaypoints,[x2, y2]); %this function is overkill, but at least it works
    if (distanceFromLine < distThreshold)
%         distToIBW = zeros(1,100);
%         for i=1:100
%             % dist to IB ibWaypoints
%             distToIBW(i) = p_poly_dist(cx_t(i),cy_t(i),ibWaypoints(:,1),ibWaypoints(:,2));
%         end
%         pctDistToIBW = sum(distToIBW)/(100 * distThreshold);
%         inverseSpeed = inverseSpeed / (1 - cIB * pctDistToIBW);
        gCost = dist / maxSpeed;
    else
        inverseSpeedInterp = interp2(inverseSpeed,cy_t,cx_t);
        
        if numel(find(isnan(inverseSpeedInterp))~= 0 )
           inverseSpeedInterp = interp2(inverseSpeed,cy_t,cx_t);
        end

        % i don't know why, but sometimes inverseSpeedInterp(100) is NaN
        if isnan(inverseSpeedInterp(100))
           inverseSpeedInterp(100) = inverseSpeed(x2,y2); 
        end

        gCost = dt * sum(inverseSpeedInterp);


        if isnan(gCost)
           gCost = Inf;
        end
    end
end