
% Function to find path, gets start and finish coordinates
function [pathMatrix, pathArray, Gcost] = AStar(startX, startY, targetX, targetY, latitude, longitude, inverseSpeed, whichList, waypoints, delayOption, drawOption, smoothingOn)
    addpath algorithm/subroutines
    Fcost = NaN;
    minFcost = 10*sqrt((startX - targetX)^2 + (startY - targetY)^2);
    
    pathArray = [];
    
    iceBreakerDistThreshold = 1.5;
    
    speed = 1 ./ inverseSpeed;
    maxSpeed = max(max(speed));
    
    colormap spring
    cm = colormap;

    mapWidth = size(whichList, 1);
    mapHeight = size(whichList, 2);

    parentX = sparse(mapWidth,mapHeight);        % 2D array stores x coord of parent for every cell
    parentY = sparse(mapWidth,mapHeight);        % 2D array stores y coord of parent for every cell
    Gcost = sparse(mapWidth,mapHeight);          % 2D array stores G cost of each cell
    Hcost = zeros(1,mapWidth*mapHeight);        % 1D array stores H cost of open cell list
    pathLength = 0;                             % Will store pathlength                

    % Create variables
    newOpenListItemID = 0;
    parentXval = 0;
    parentYval = 0;
    a = 0; b = 0; m = 0; u = 0; v = 0;
    temp = 0;
    corner = 0;
    numberOfOpenListItems = 0;
    addedGCost = 0;
    tempGCost = 0;
    path = 0;
    
    % Variables used as constants
    walkable = 1;
    onOpenList = 2;
    onClosedList = 3;
    unwalkable = 4;
    found = 1;
    nonexistent = 2;
    
    
    fprintf('------------------------------- \n');
    fprintf('##### Astar Path Planning ##### \n');
    fprintf('------------------------------- \n\n');
    fprintf('Starting the simulation \n');
    fprintf('Using the following options: \n');
    fprintf('Start position: %d, %d \n', startX, startY);
    fprintf('Finish position: %d, %d \n', targetX, targetY);
    fprintf('Map size: %d \n\n\n', size(whichList,1));
    fprintf('Progress:\n');
    
    % Check if start and finish coordinates are the same
    if ( (startX == targetX) && (startY == targetY) )
        disp('Start and finish positions are the same');
        return;
    end
    
    % Add starting position to the open list
    numberOfOpenListItems = 1;
    openList(1) = 1;
    openX(1) = startX;
    openY(1) = startY;
   
    % Counter used for printing
    i = 1;
    
    % Create an inifinite loop (until goal is reached or defined unreachable
    while (1)
        
        % cause a delay (if specified)
        performDelay(delayOption)
        
        % Check if there are any members in the open list
        if (numberOfOpenListItems ~= 0)
            
            %% Get next node in open list and close it
            
            % Get the values of the first item in the list into current vals
            parentXval = openX(openList(1));
            parentYval = openY(openList(1));
            % set the coordinate to closed list
            whichList(parentXval, parentYval) = onClosedList;
            
            % Print out current position being analysed
            fprintf('%d. Current position being analysed: %d, %d \n', i, parentXval, parentYval);
            i = i + 1;
            
            % Draw current position in blue (unless it's the start or
            % target cell.
            if (drawOption && ~((parentXval == startX) && (parentYval == startY) || (parentXval == targetX) && (parentYval == targetY)) )
               plotPoint([parentXval, parentYval], 'b');
            end
            
            
            %% ## List sorting ##
            
            % Put last element into the slot of first one
            openList(1) = openList(numberOfOpenListItems);
            
            % Remove the first element from the open list
            numberOfOpenListItems = numberOfOpenListItems - 1;
            
            % Make a copy of open list excluding first element
            tempOpenList = openList(1:numberOfOpenListItems);
            
            % Clear openList
            clear openList;
            
            % Assign tempOpenList to openList, clear temp one
            openList = tempOpenList;
            clear tempOpenList;
            
            v = 1;
            % Put the item into it's appropriate slot by looking at the Fcost
            while (1)
                u = v;
                % If both children exist
                if (2*u+1 <= numberOfOpenListItems)
                    % Check if F cost of the parent is greater than each child
                    % Then select the lowest of the two children
                    if (Fcost(openList(u)) >= Fcost(openList(2*u)))
                        v = 2*u;
                    end
                    if (Fcost(openList(v)) >= Fcost(openList(2*u+1)))
                        v = 2*u+1;
                    end
                % If only 1 child exists    
                elseif (2*u <= numberOfOpenListItems)
                    % Check if F cost of the parent is greater than child
                    if (Fcost(openList(u)) >= Fcost(openList(2*u)))
                        v = 2*u;
                    end
                end
                
                % If parent's F is larger than one of its children, swap them
                if (u ~= v)
                    temp = openList(u);
                    openList(u) = openList(v);
                    openList(v) = temp;
                % Otherwise - break out of the loop
                else
                    break;
                end
            end
            
            %if ~isnan(Fcost)
            %    min(Fcost)
            %end

            
            %% Check the neighbours
            % a x-value of neighbor
            % b y-value of neighbor
            
            for j=1:56
                [a, b] = getNextNeighbor(j, parentXval, parentYval);
                
                    % Check if it is within bounds of a map
                    if ( (a > 0) && (b > 0) && (a <= mapWidth) && (b <= mapHeight) )
                        % Check if not on closed list
                        if (whichList(a,b) ~= onClosedList)
                            %  ---Check if not an obstacle
                            % Check if it is possible to navigate directly
                            % to this neighbor cell from the parent cell
                            if isNavigable(a, b, parentXval, parentYval, whichList)
                            %if (whichList(a,b) ~= unwalkable)
                                
                                corner = walkable;
                                % Corner detection to prevent cutting
                                %
                                %  For example, the following path shown as
                                %  the dots in the below diagram should not
                                %  be allowed (obstacles marked as "X")
                                %  ___________
                                %  | | |.| | |
                                %  |X|.|.| | |
                                %  |.|X| | | |
                                %  |.| |X| | |
                                %  |.| | | | |
                                %  -----------
                                corner = determineIfCornerIsWalkable(a, b, parentXval, parentYval, whichList, corner);

                                % If corner is walkable
                                if (corner == walkable)
                                    % If not on the open list, add it
                                    if (whichList(a,b) ~= onOpenList)
                                        % Create a new open list item
                                        newOpenListItemID = newOpenListItemID + 1;
                                        m = numberOfOpenListItems + 1;
                                        openList(m) = newOpenListItemID;
                                        openX(newOpenListItemID) = a;
                                        openY(newOpenListItemID) = b;
                                                 
                                        % Calculate its G cost
                                        addedGCost = calculateGCost3(parentXval, parentYval,a,b,j,inverseSpeed, waypoints, maxSpeed,iceBreakerDistThreshold);
                                        
%                                         if ( (abs(a - parentXval) == 1) && (abs(b - parentYval) == 1) )
%                                             addedGCost = 14;
%                                         else
%                                             addedGCost = 10;
%                                         end
                                        % Update Gcost map
                                        Gcost(a,b) = Gcost(parentXval,parentYval) + addedGCost;
                                        
                                        % Get H and F costs and parent
                                        %Hcost(openList(m)) = 10*(abs(a - targetX) + abs(b - targetY)); %Manhattan distance
                                        %Hcost(openList(m)) = 10*sqrt((a - targetX)^2 + (b - targetY)^2); %Euclidean distance
                                        %Hcost(openList(m)) = 700*(abs(a - targetX) + abs(b - targetY)); %Suggestion from SO
                                        % Hcost(openList(m)) = geoddistance(latitude(1,b),longitude(a,1),latitude(1,targetY),longitude(targetX,1)) / maxSpeed;
                                        Hcost(openList(m)) = 10*sqrt((a - targetX)^2 + (b - targetY)^2) / maxSpeed; %Euclidean distance
                                        Fcost(openList(m)) = Gcost(a,b) + Hcost(openList(m));
                                        parentX(a,b) = parentXval;
                                        parentY(a,b) = parentYval;
                                        
                                        % Draw current position, shade of
                                        % yellow/orange indicates Fcost
                                        if (drawOption && ~((a == startX) && (b == startY) || (a == targetX) && (b == targetY)) )
                                            colorIndex = round((minFcost / Fcost(openList(m)))^3*64)
                                            plotPoint([a, b], cm(colorIndex,:));
                                        end
                                        
                                        % Listing
                                        while (m ~= 1)
                                            % Check if child's F cost < parent's F cost. If so, swap them
                                            if (Fcost(openList(m)) <= Fcost(openList(round(m/2))))
                                                temp = openList(round(m/2));
                                                openList(round(m/2)) = openList(m);
                                                openList(m) = temp;
                                                m = round(m/2);
                                            else
                                                break;
                                            end
                                        end
                                        
                                        % Increment openlist items
                                        numberOfOpenListItems = numberOfOpenListItems + 1;
                                        % Change current node into open list
                                        whichList(a,b) = onOpenList;
                                    else % i.e. whichList(a,b) == onOpenList
                                        % Calculate its G cost
                                        addedGCost = calculateGCost3(parentXval, parentYval,a,b,j,inverseSpeed, waypoints, maxSpeed,iceBreakerDistThreshold);
                                        tempGcost = Gcost(parentXval,parentYval) + addedGCost;

                                        % If this path is shorter, change Gcost, Fcost and the parent cell
                                        if (tempGcost < Gcost(a,b)),
                                            parentX(a,b) = parentXval;
                                            parentY(a,b) = parentYval;
                                            Gcost(a,b) = tempGcost;

                                            % Changing G cost also changes F cost, sothe open list has to be updated and reordered
                                            % Look for item
                                            for x=1:numberOfOpenListItems
                                                % Identify the item
                                                if ( (openX(openList(x)) == a) && (openY(openList(x)) == b) )
                                                    % Change F cost
                                                    Fcost(openList(x)) = Gcost(a,b) + Hcost(openList(x));
                                                    
                                                    % Change the color shade of
                                                    % yellow/orange indicates Fcost
                                                    if (drawOption && ~((a == startX) && (b == startY) || (a == targetX) && (b == targetY)) )
                                                        colorIndex = round((minFcost / Fcost(openList(x)))^3*64)
                                                        plotPoint([a, b], cm(colorIndex,:));
                                                    end
                                                    
                                                    % Reorder the list if needed
                                                    m = x;
                                                    while (m ~= 1)
                                                        % If child < parent, swap them
                                                        if (Fcost(openList(m)) < Fcost(openList(round(m/2))))
                                                            temp = openList(round(m/2));
                                                            openList(round(m/2)) = openList(m);
                                                            openList(m) = temp;
                                                            m = round(m/2);
                                                        else
                                                            break;
                                                        end
                                                    end
                                                    % Exit the loop when found
                                                    break;
                                                end
                                            end % Loop through open list to find current cell
                                        end   % updating GCost 

                                    end % if-else statement checking whether neighbor is on open list    
                                else % corner is not walkable
                                    %fprintf('%d. Corner found, walking around it \n', i);
                                    i = i + 1;
                                end
                            end
                        end % if statement checking if neighbor is on closed list
                end % if statement checking that neighbor is on the map
            end %End of loop through the neighbors
                                
        % If open list is empty     
        else
            path = nonexistent;
            % Print out failure
            fprintf('%d. Path to the target could not be found \n', i);
            i = i + 1;
            break;
        end
        % If target is added to open list, path has been found
        if (whichList(targetX,targetY) == onOpenList)
            path = found;
            % Print out success
            fprintf('%d. Path to the target found! \n', i);
            i = i + 1;
            break;
        end
        
    end
    
    % If path was found
    if (path == found)
        
        % Initialize data structures
        pathMatrix = zeros(mapHeight,mapWidth);
        pathLength = 0;
        
        % Backtrack the path using parents
        pathX = targetX;
        pathY = targetY;
        % Print out backtracking
        fprintf('%d. Backtracing to find the shortest route \n', i);
        i = i + 1;
        
        % Pre-allocate pathArray to reasonable maximum size
        pathArray = zeros(mapWidth*mapHeight,2);
        
        % Loop until starting position is reached
        while(1)
            % Lookup parent of current cell
            tempx = parentX(pathX,pathY);
            pathY = parentY(pathX,pathY);
            pathX = tempx;
            
            % Increment the path length
            pathLength = pathLength + 1;
            
            pathMatrix(pathY,pathX) = 1;
            
            pathArray(pathLength,1) = pathX;
            pathArray(pathLength,2) = pathY;
        
            % Draw return path in magenta
            if (drawOption && ~((pathX == startX) && (pathY == startY) || (pathX == targetX) && (pathY == targetY)) )
                plotPoint([pathX, pathY], 'c');
            end
            

            
            % If starting position reached, break out of the loop
            if ( (pathX == startX) && (pathY == startY) )
                break;
            end
        end
        
        % Print out result
        fprintf('%d. Shortest route is shown in cyan. Total length: %d steps \n', i, pathLength);
        
    end
    
    pathArray = [targetX targetY; pathArray(1:pathLength,:)];
    plot(pathArray(:,1)-0.5,pathArray(:,2)-0.5,'Color','r','LineWidth',3);
    
    if (smoothingOn)
       performSmoothing(whichList, pathArray); 
    end

   %pathMatrix = flipud(pathMatrix);
end

function performDelay(delayOption)
        % Check the delay option
        % 1 - no delay
        % 2 - 0.1s delay
        % 3 - 0.15s delay
        % 4 - wait for input
        switch delayOption
            case 1
                % No delay
            case 2
                pause(0.1);
            case 3
                pause(0.15);
            case 4;
                disp('Press any key (for example space) for next step');
                pause;
        end
end
