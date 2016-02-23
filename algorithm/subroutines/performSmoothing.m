function [pathArray pathLength] = performSmoothing(whichList, pathArray, pathLength)
    
    disp('Performing smoothing operation...')
    close all
    figure
    

    
    i = 2;
    
    while (i<pathLength)
        disp(['Current index is ' num2str(i)]);
        disp(['Pathlength is currently ' num2str(pathLength)]);
        checkPoints = pathArray(i-1:i+1,:);
        
        if(mod(i,10)==0)
            drawPlot=true;
        else
            drawPlot = false;
        end
        
        if isNavigable(checkPoints, whichList, drawPlot)
            % Delete intermediate point
            disp(['Deleting point ' num2str(i-1) '...'])
            pathArray = deleteIntermediatePoint(pathArray, i);
            
            % Decrement pathLength by one
            pathLength = pathLength-1;
        else
             % Just increment index
            i = i + 1;
        end
        disp(' ')
 
    end
end

function navigable = isNavigable(checkPoints, whichList, drawPlot)
    % IN PROGRESS
    %navigable = randi(2)-1;
    %drawPlot = false;
    
    boundaries = findBoundaries(checkPoints);
    
    pointA = checkPoints(1,:);
    pointB = checkPoints(3,:);
    slopeOfLine = (pointB(2)-pointA(2)) / (pointB(1)-pointA(1));
    
    % The route should be considered navigable if every intermediate point 
    % between the two end check-points (i.e. 1 and 3) is sufficiently far 
    % from the all nearby obstacles. This will be done in a two-step process.
    neighborObstacles = getNeighboringObstacles(boundaries, whichList);
    numberOfNeighborObstacles = size(neighborObstacles,1);
    
    if (drawPlot == true)
        %Expand the boundaries by one macro-pixel.
        %boundaries(1) = boundaries(1)-1;
        %boundaries(2) = boundaries(2)+1;
        %boundaries(3) = boundaries(3)-1;
        %boundaries(4) = boundaries(4)+1;
        plotGrid(checkPoints, whichList, boundaries, ...
            neighborObstacles, numberOfNeighborObstacles,...
            slopeOfLine)
    end
    
    %Plot the intermediate step points
    stepLength = 1/5;
    i=0;
    %intermediatePoint = zeros(1,2);
    scale = sqrt(slopeOfLine^2 + 1);

    minX = min(pointA(1),pointB(1)); %duplication!
    % Attempted [minX whichX] = min(pointA(1),pointB(1)), but got this
    % error:  (????)
    % Error using min
    % MIN with two matrices to compare and two output arguments is not supported.
    

    intermediatePoints = [];
    
    stopFlag = false;
    while(~stopFlag)
            % Calculate intermediate point
            intermediatePointX = pointA(1)+i*stepLength/scale;
            intermediatePointY = pointA(2)+i*stepLength*slopeOfLine/scale;
            intermediatePoints = [intermediatePoints; ...
                intermediatePointX, intermediatePointY];
            
            if ((abs(intermediatePointX-pointB(1))+4*eps) >= stepLength || ...
                    (abs(intermediatePointY-pointB(2))+4*eps) >= stepLength)
                  
                if (drawPlot == true)
                    circle(intermediatePointX, intermediatePointY,0.1,'g',101);
                    plot(intermediatePointX,intermediatePointY, 'y*');
                end
            else
                stopFlag = true;
            end

            i=i+1;
    end 

    
    % Calculate the distances
    distanceMatrix = calculateDistanceMatrix(intermediatePoints,neighborObstacles);
    minDistance = min(min(distanceMatrix))
    %Find all below a certain threshold distance.
    thresholdDistance = 0.7;
    [row, col] = find(distanceMatrix <= thresholdDistance);
    tmpDistances = distanceMatrix(row,col);
    if  isempty(row)
        navigable = true;
        
    else
        navigable = false;
        troubleSpots = [intermediatePoints(row,:) neighborObstacles(col,:) tmpDistances(:,1) ];
    end
end

function distanceMatrix = calculateDistanceMatrix(intermediatePoints,neighborObstacles)
    
    numberOfIntermediatePoints = size(intermediatePoints,1);
    numberOfNeighborObstacles = size(neighborObstacles,1);
    
    % Initialize the matrix
    distanceMatrix = zeros(numberOfIntermediatePoints,numberOfNeighborObstacles);
    
    for i=1:numberOfNeighborObstacles
            % First make a copy of neighbor obstacle (for vectorization)
            tmpCopyOfNeighborObstacle = repmat(neighborObstacles(i,:), ...
                numberOfIntermediatePoints,1);
            % Compute distance of intermediate points to neighbor obstacle
            distanceMatrix(:,i) = sqrt((intermediatePoints(:,1)-tmpCopyOfNeighborObstacle(:,1)).^2 ...
            +(intermediatePoints(:,2)-tmpCopyOfNeighborObstacle(:,2)).^2);
    end
    

    
end

function boundaries = findBoundaries(checkPoints)

    pointA = checkPoints(1,:);
    pointB = checkPoints(3,:);
    
    minX = min(pointA(1),pointB(1))-1;
    maxX = max(pointA(1),pointB(1))+1;
    minY = min(pointA(2),pointB(2))-1;
    maxY = max(pointA(2),pointB(2))+1;
    
    boundaries = [minX maxX minY maxY];
end

function neighborObstacles = getNeighboringObstacles(boundaries, whichList);
    neighborObstacles = [];
    
    minX = boundaries(1);
    maxX = boundaries(2);
    minY = boundaries(3);
    maxY = boundaries(4);   
 
    for x=minX:maxX
            for y=minY:maxY
                if whichList(x,y) == 4;
                    %Add it to the obstacle list
                    neighborObstacles = [neighborObstacles; x, y];
                end
            end
    end
        
end

function plotGrid(checkPoints, whichList, boundaries, neighborObstacles, numObstacles, ...
    slopeOfLine)
        clf;
        pointA = checkPoints(1,:);
        pointB = checkPoints(3,:);
        pointInt = checkPoints(2,:);
        
        minX = boundaries(1);
        maxX = boundaries(2);
        minY = boundaries(3);
        maxY = boundaries(4);
        

        axis equal
        
        axis([minX-0.5 maxX+0.5 minY-0.5 maxY+0.5]);
        %% Plot the points
        
        hold on;
        % Point A
        patch([pointA(1)-0.5; pointA(1)+0.5; pointA(1)+0.5; pointA(1)-0.5], ...
            [pointA(2)-0.5; pointA(2)-0.5; pointA(2)+0.5; pointA(2)+0.5], 'b', 'EdgeColor', 'k');
        
        % Point B
        patch([pointB(1)-0.5; pointB(1)+0.5; pointB(1)+0.5; pointB(1)-0.5], ...
            [pointB(2)-0.5; pointB(2)-0.5; pointB(2)+0.5; pointB(2)+0.5], 'b', 'EdgeColor', 'k');
        
        % In-between point
        patch([pointInt(1)-0.5; pointInt(1)+0.5; pointInt(1)+0.5; pointInt(1)-0.5], ...
            [pointInt(2)-0.5; pointInt(2)-0.5; pointInt(2)+0.5; pointInt(2)+0.5], 'r', 'EdgeColor', 'k');
        
        %% Plot obstacles
        
        for i=1:numObstacles
            x = neighborObstacles(i,1);
            y = neighborObstacles(i,2);
            patch([x-0.5; x+0.5; x+0.5; x-0.5], ...
                [y-0.5; y-0.5; y+0.5; y+0.5], [-99; -99; -99; -99],'k', 'EdgeColor', 'k' ...
                );
            circle(x,y,0.6,'m',100);

        end
        
        % Plot the starting point
        plot(pointA(1),pointA(2),'g.')
        
        % Plot the line
        plot([pointA(1) pointB(1)],[pointA(2) pointB(2)],'g')
        
        % Make an arrow to indicate direction of travel
        % Surprisingly non-trivial!
        % plot(pointB(1),pointB(2),'g>')  <- wasn't satisfied with this!

        phi = atand(slopeOfLine); %Angle defined by slope of line (degrees)
        widthOfArrow = 15; % This is the angle that determines the how narrow or wide the arrow is (degrees)
        slopeUpper = tand(phi-widthOfArrow); % for upper part of arrow
        slopeLower = tand(phi+widthOfArrow); % for lower part of arrow
        scale = sqrt(slopeLower^2-slopeUpper^2+1); % a scaling factor is needed to adjust the lines to equal length
        lengthOfArrow = 0.2; % This determines the length of the lines in the arrow
        
        % finally, plot the arrow
        patch([pointB(1),pointB(1)-lengthOfArrow*scale,pointB(1)-lengthOfArrow],...
            [pointB(2),pointB(2)-lengthOfArrow*slopeUpper*scale,pointB(2)-lengthOfArrow*slopeLower],'g')
    
   



end

function circle(x,y,r,color,Z)
%x and y are the coordinates of the center of the circle
%r is the radius of the circle
%0.01 is the angle step, bigger values will draw the circle faster but
%you might notice imperfections (not very smooth)
ang=0:0.01:2*pi; 
xp=r*cos(ang);
yp=r*sin(ang);
plot(x+xp,y+yp,...
    'Color',color);
end

function stopFlag = plotIntermediatePoints(pointA, pointB, stepLength, intermediatePointX, intermediatePointY)
    %% PROBABLY DOESN'T WORK FOR DECREASING X CASES!!!!!!!!!!
    stopFlag = false;

    if ((abs(intermediatePointX-pointB(1))+4*eps) >= stepLength || ...
            (abs(intermediatePointY-pointB(2))+4*eps) >= stepLength)
          circle(intermediatePointX, intermediatePointY,0.1,'g',101);
        plot(intermediatePointX,intermediatePointY, 'y*');
    else
        stopFlag = true;
    end
    
end
function pathArray = deleteIntermediatePoint(pathArray, indexToDelete)
    pathArray = [pathArray(1:indexToDelete-1,:); ...
        pathArray(indexToDelete+1:end,:)];
end

