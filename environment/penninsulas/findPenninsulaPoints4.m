function [pennPoints, hList] = findPenninsulaPoints(borders, continents, start, finish)
    pointsToCheck = zeros(0,2);
    hList = NaN;
    pennPoints = zeros(0,3);
    checkPointStack = zeros(0,6);
%     minX = 0;
%     maxX = 176;
%     minY = 0;
%     maxY = 98;

    %borders = [minX, maxX, minY, maxY];

    %boundary = boundaries{8,1};
    %boundary = [boundary(:,2),maxY - boundary(:,1)+1];
    colormap spring
    cm = colormap;

    %fraction = 1;
    %for i = 1:size(boundary,1)
    %    plotPoint(boundary(i,:),'w');
    %end    
    %start = [80 80];
    %plotPoint(start,'g')
    %finish = [164 78];
    %plotPoint(finish,'g');
    
    numberOfObjects = length(continents);
    for k = 1:numberOfObjects
        boundary = continents{k,1};
            
        %pointsToCheck = findPointsToCheck(start,finish,boundary, borders);
%         if isempty(pointsToCheck)
%             continue
%         end
        j=1;
        checkPointStack = pushToStack(start,finish,0, 1000, checkPointStack);
        while (size(checkPointStack, 1) ~=0)
            [intPointA, intPointB, fractionA, fractionB, checkPointStack] = popFromStack(checkPointStack);
            
            [pointsToCheck, tempHList] = findPointsToCheck2(intPointA,intPointB,boundary,borders);
            if size(tempHList,1)>1
                hList = [hList; tempHList];
            end
            if isempty(pointsToCheck)
                break
            end
            [maxPoint, maxDist] = findPenninsulaPoint(start, intPointA,intPointB,pointsToCheck, borders,k);
            
            maxPoint = bisect(intPointA,maxPoint,intPointB);
            
            hList = [hList; plotPoint(maxPoint,'c')];
            hList = [hList; plotLine(intPointA,maxPoint,'c')];
            hList = [hList; plotLine(intPointB,maxPoint,'c')];

            
            
            pennPoints(j,1:2) = maxPoint(:,:);
            newFraction = (fractionB+fractionA)/2
            pennPoints(j,3) = newFraction;
%                 %% give ordering index
%                 if (indexPoint == 1)
%                     %intPointA = maxPoint;
%                     pennPoints(j,3) = j;
%                 else
%                     %plotLine(intPointB,maxPoint,'c');
%                     %intPointB = maxPoint;
%                     pennPoints(j,3) = 10000-j;
%                 end
             j = j+1;

            %% check the two legs for intersections
            [intersectX1, intersectY1] = intersections([intPointA(1) maxPoint(1)], [intPointA(2) maxPoint(2)], ...
            boundary(:,1), boundary(:,2));
            [intersectX1, intersectY1] =  removeDuplicates(intersectX1, intersectY1);
            if (size(intersectX1, 1) < 3)
               % check that intersects are either starting or ending point (intPointA or maxPoint 

                % if so, no need to further check this leg
            else
                distBetweenPoints = sqrt((maxPoint(1)-intPointA(1))^2+(maxPoint(2)-intPointA(2))^2)
                if distBetweenPoints > 2
                    %need to check this leg recursively.
                    checkPointStack = pushToStack(intPointA, maxPoint, fractionA, newFraction, checkPointStack);
                end
            end
            [intersectX2, intersectY2] = intersections([intPointB(1) maxPoint(1)], [intPointB(2) maxPoint(2)], ...
            boundary(:,1), boundary(:,2));
            [intersectX2, intersectY2] =  removeDuplicates(intersectX2, intersectY2);
            if (size(intersectX2, 1) < 3)
               % check that intersects are either starting or ending point (intPointA or maxPoint 

                % if so, no need to further check this leg
            else
                distBetweenPoints = sqrt((maxPoint(1)-intPointB(1))^2+(maxPoint(2)-intPointB(2))^2)
                if distBetweenPoints > 2
                    %need to check this leg recursively.
                    checkPointStack = pushToStack(maxPoint,intPointB, newFraction, fractionB, checkPointStack);
                end

            end
        end
    end
    
end
