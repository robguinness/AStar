function navigable = isNavigable(xDest, yDest, xOrigin, yOrigin, whichList)
    
    unwalkable = 4; %constant used in whichList to define obstacles
    continent = 5;
    
    % Next define a threshold (pixel) distance that an obstacle must be
    % from the path of navigation
    thresholdDistance = 0.7; % chosen to correspond roughly to sqrt(2)
    
    %% first check that destination is not an obstacle
    if (whichList(xDest,yDest) == unwalkable || whichList(xDest,yDest) == continent)
        navigable = false;
        return
    end
    
    %% find equation for line drawn from origin to destination
    a1 = (yDest-yOrigin) / (xDest-xOrigin); %slope
    b1 = yDest - a1*xDest; % y-intercept
    
    % find slope of orthogonal line (i.e. orthogonal to the above line)
    %phi = atand(a1); %Angle defined by slope of above line (degrees)
    %a2 = tand(phi+90); % slope of orthogonal line
    a2 = -1/a1;
    
    
    %% Get obstacles within bounding box
    
    % define bounding box
    minX = min(xDest, xOrigin);
    maxX = max(xDest, xOrigin);
    minY = min(yDest, yOrigin);
    maxY = max(yDest, yOrigin);
    
    % index of upper-left (in geometry of whichList)
    %indexUL = sub2ind(size(whichList),minX,minY);
    % index of lower-right (in geometry of whichList)
    %indexLR = sub2ind(size(whichList),maxX,maxY);

    % get all obstacles
    [obstacleX, obstacleY] = find(whichList==unwalkable | whichList==continent);
    obstacles = [obstacleX obstacleY];

    % find all obstacles that are in the bounding box
    obstaclesInBoundingBox = obstacles(logical(isInBox(obstacles,[minX minY maxX maxY])),:);
   
    %% check the distance to the line for all obstacles in the bounding box
    
    for i=1:size(obstaclesInBoundingBox,1);
       obstacle = obstaclesInBoundingBox(i,:);
       m = obstacle(1); % x-coordinate of obstacle
       n = obstacle(2); % y-coordinate of obstacle
        
       % for a few special cases, this calculation is easier
       if (a1 == Inf || a1 == -Inf)
            dist = m-xDest;
       elseif (a1 == 0)
            dist = n-yDest;
       else
       
           % otherwise, we need to define a line passing through the obstacle
           % with slope a2 (i.e. y = a2*x + b2). Since we already know its
           % slope and a point on the line, we can solve for the y-intercept.
           b2 = n - a2*m; % y-intercept of "orthogonal line"

           % solve for point where the two lines intersect
           xIntersect = (b2-b1)/(a1-a2);
           yIntersect = (a1*b2 - a2*b1)/(a1-a2);

           % dist is Euclidean dist between (m,n) and (xIntersect,yIntersect)
           dist = sqrt((xIntersect-m)^2 + (yIntersect-n)^2);
       end
       if (dist < thresholdDistance)
           navigable = false;
           return;
       end

    end
    
    navigable = true;
end