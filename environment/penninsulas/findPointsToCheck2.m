function [pointsToCheck, hList] = findPointsToCheck(start, finish, boundary, borders)
    pointsToCheck = zeros(0,2);
    twoIntersectsFlag = false;
    hList = NaN;
    %hPatch = NaN;
    [intersectX, intersectY] = intersections([start(1) finish(1)], [start(2) finish(2)], ...
            boundary(:,1), boundary(:,2));
        numIntersects = size(intersectX,1);
        if numIntersects==0
            return
        elseif numIntersects==1;
            disp(intersectX);
        elseif numIntersects==2;
            twoIntersectsFlag = true;
        else
            twoIntersectsFlag = false;
        end
        hList = [hList; plotLine(start,finish,'b')];

        closestToIntersect = zeros(numIntersects,1);
        logicalIntersect = ones(numIntersects,1);
        d = zeros(numIntersects,1);
        distToStart = zeros(numIntersects,1);
        for i=1:numIntersects
            % find nearest point on boundary
            [d(i), closestToIntersect(i)] = p_poly_dist(intersectX(i), intersectY(i),boundary(:,1), boundary(:,2));
            distToStart(i) = sqrt((start(1)-boundary(closestToIntersect(i),1))^2+(start(2)-boundary(closestToIntersect(i),2))^2);
            plotPoint([boundary(closestToIntersect(i),1), boundary(closestToIntersect(i),2)],'m');
        end
        intersectSet = [closestToIntersect, distToStart, intersectX, intersectY];
        intersectSet = sortrows(intersectSet,1);
        [d2, closestToStart] = min(intersectSet(:,2));
        logicalIntersect(closestToStart) = false;
        indexStartingPoint = intersectSet(closestToStart,1);

        for i=1:numIntersects
            %if ishandle(hPatch)
            %    delete(hPatch)
            %end
            flag = false;
            correctFlag = false;
            
            if i == 1
                if closestToStart == numIntersects
                    nextIndex = intersectSet(1,1);
                    nextIndexInSet = 1;
                    logicalIntersect(1) = false;
                else
                    nextIndex = intersectSet(closestToStart+1,1);
                    nextIndexInSet = closestToStart+1;
                    logicalIntersect(closestToStart +1) = false;
                end
            else

                nextIndex = intersectSet(nextIndexInSet,1);
                logicalIntersect(nextIndexInSet) = false;
            end
            if nextIndex < indexStartingPoint
                    flag = true;
            end
            startingPoint = boundary(intersectSet(closestToStart,1),:);
            if flag
                polyToCheck = [boundary(indexStartingPoint:end,:);boundary(1:nextIndex,:)];
            else
                polyToCheck = boundary(indexStartingPoint:nextIndex,:);
            end
            plot(polyToCheck(:,1),polyToCheck(:,2),'r');
            
            
            if twoIntersectsFlag
               % do something
               %disp(polyToCheck)
               barrier = find((polyToCheck(:,1)==borders(1)+1 ...
                    | polyToCheck(:,1) == borders(2) | ...
                    polyToCheck(:,2)==borders(3)+1 | polyToCheck(:,2)==borders(4)));
               if size(barrier,1)>1
                   correctFlag = false;
               else
                   correctFlag = 1;
                   pointsToCheck = [pointsToCheck;polyToCheck];
                   hList = [hList; patch(polyToCheck(:,1),polyToCheck(:,2),'c')];
               end
            end
            
%             intersectsToCheckX = intersectSet(logical(logicalIntersect),3);
%             intersectsToCheckY = intersectSet(logical(logicalIntersect),4);
%             numIntersectsToCheck = size(intersectsToCheckX,1);
%             if numIntersectsToCheck == 0
%                 %%
%                 maxD = 0;
%                 disp('nothing to check')
%             else
%                 maxD = 0;
%                 for j = 1:numIntersectsToCheck
%                     [dI, index] = p_poly_dist(intersectsToCheckX(j),intersectsToCheckY(j), ...
%                         polyToCheck(:,1),polyToCheck(:,2));
%                     if dI > maxD
%                         maxD = dI;
%                     end
%                 end
%             end


            %disp(['large maxD: ' num2str(maxD)]);
            barrier = find((polyToCheck(:,1)==borders(1)+1 ...
                | polyToCheck(:,1) == borders(2) | ...
                polyToCheck(:,2)==borders(3)+1 | polyToCheck(:,2)==borders(4)));
            if size(barrier,1)>1
                correctFlag = false;
            else 
                P1.x=polyToCheck(:,1);
                P1.y=polyToCheck(:,2);
                P1.hole=0;

                P2.x=boundary(:,1);
                P2.y=boundary(:,2);
                P2.hole=0;
                P3=PolygonClip(P1,P2,1);
                if ~isempty(P3)
                    P4=PolygonClip(P1,P3,2);
                    if ~isempty(P4)
                        n = size(P4,2);
                        area = 0;
                        for k=1:n
                           area = polyarea(P4(k).x,P4(k).y) + area;
                        end
                        fractionOfTotal = area / polyarea(polyToCheck(:,1),polyToCheck(:,2));
                        if fractionOfTotal < 0.1
                            correctFlag = 1;
                            pointsToCheck = [pointsToCheck;polyToCheck];
                            hList = [hList; patch(polyToCheck(:,1),polyToCheck(:,2),'c')];
                        end
                    else
                        correctFlag = 1;
                        pointsToCheck = [pointsToCheck;polyToCheck];
                        hList = [hList; patch(polyToCheck(:,1),polyToCheck(:,2),'c')];
                    end
                end        
            %plot(P3.x,P3.y,'c');
            end
               
            indexStartingPoint = nextIndex;
            logicalIntersect = ones(numIntersects,1);
            logicalIntersect(nextIndexInSet) = false;

            if nextIndexInSet == numIntersects
                nextIndexInSet = 1;
            else
                nextIndexInSet = nextIndexInSet + 1;
            end
        end
        %if ishandle(hPatch)
        %    delete(hPatch)
        %end
        hList = [hList; plot(polyToCheck(:,1),polyToCheck(:,2),'b')]
end